/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#include "twi.h"

#include "nvm.h"
#include "util.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include <string.h>

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) ||                  \
    defined(__AVR_ATtiny85__)

#define SCL_BIT  (1 << PB2)
#define SDA_BIT  (1 << PB0)
#define TWI_BITS (SCL_BIT | SDA_BIT)

#define SET_OUTPUT(P) DDRB |= (P)
#define SET_INPUT(P)  DDRB &= ~(P)

#define USICR_AWAIT_START                                                      \
    ((1 << USISIE) | /* Start condition interrupt enabled */                   \
     (0 << USIOIE) | /* Counter overflow interrupt disabled */                 \
     (1 << USIWM1) | /* Two-wire mode */                                       \
     (0 << USIWM0) | /* Not holding SCL low on overflow flag */                \
     (1 << USICS1) | /* External clock source */                               \
     (0 << USICS0) | /* positive edge */                                       \
     (0 << USICLK) | (0 << USITC))

#define USICR_AWAIT_DATA                                                       \
    ((1 << USISIE) | /* Start condition interrupt enabled */                   \
     (1 << USIOIE) | /* Counter overflow interrupt enabled */                  \
     (1 << USIWM1) | /* Two-wire mode */                                       \
     (1 << USIWM0) | /* Hold SCL low on overflow flag */                       \
     (1 << USICS1) | /* External clock source */                               \
     (0 << USICS0) | /* positive edge */                                       \
     (0 << USICLK) | (0 << USITC))

#define USISR_RESET                                                            \
    ((1 << USISIF) | /* Reset start condition interrupt flag */                \
     (1 << USIOIF) | /* Reset counter overlow interrupt flag */                \
     (1 << USIPF) |  /* Reset stop condition flag */                           \
     (1 << USIDC) |  /* Reset data output collision */                         \
     (0 << USICNT3) | (0 << USICNT2) | (0 << USICNT1) | (0 << USICNT0))

#define USISR_SHIFT_1BIT                                                       \
    ((0 << USISIF) | /* Leaving start condition interrupt flag unchanged */    \
     (1 << USIOIF) | /* Reset counter overlow interrupt flag */                \
     (1 << USIPF) |  /* Reset stop condition flag */                           \
     (1 << USIDC) |  /* Reset data output collision */                         \
     (1 << USICNT3) | (1 << USICNT2) | (1 << USICNT1) | (0 << USICNT0))

#define USISR_SHIFT_8BIT                                                       \
    ((0 << USISIF) | /* Leaving start condition interrupt flag unchanged */    \
     (1 << USIOIF) | /* Reset counter overlow interrupt flag */                \
     (1 << USIPF) |  /* Reset stop condition flag */                           \
     (1 << USIDC) |  /* Reset data output collision */                         \
     (0 << USICNT3) | (0 << USICNT2) | (0 << USICNT1) | (0 << USICNT0))

static struct {
    /// Current state for TWI communication.
    enum {
        TWI_CHECK_ADDR,
        TWI_GET_CMD,
        TWI_STORE_CMD,
        TWI_GET_DATA,
        TWI_STORE_DATA,
        TWI_SEND_DATA,
        TWI_READ_ACK,
        TWI_CHECK_ACK,
        TWI_DONE,
    } state;
    /// Last received command.
    uint8_t cmd;
    /// Current active task.
    uint8_t task;
    /// Flag indicating TWI communication with this device.
    bool busy;
    /// Currently set temperature.
    int8_t temp;
    /// Index of next byt in buffer.
    uint8_t index;
    /// Number of valid bytes in buffer.
    uint8_t count;
    /// Send/receive buffer.
    uint8_t buf[15];
    /// Current weight accumulation and last send weight data.
    struct {
        uint32_t sum;
        uint8_t count;
    } weight, last;
} twi = {};

_Static_assert(sizeof(twi.buf) >= sizeof(calib_data),
               "Two wire interface buffer is too small");

static inline void twi_reset(void);

/// Initialize slave mode.
void twi_init(void) {
    twi_reset();

    // Set SDA and SCL high
    PORTB |= SCL_BIT | SDA_BIT;
    // Set SCL as output (for holding clock low until we are ready)
    // SDA is set as input
    DDRB = (DDRB & ~SDA_BIT) | SCL_BIT;
    // enable USI
    PRR &= ~(1 << PRUSI);
    // setup USI
    USICR = USICR_AWAIT_START;
    USISR = USISR_RESET;
}

uint8_t twi_get_task(void) {
    return twi.task;
}

void twi_reset_task(void) {
    twi.task = TWI_CMD_NONE;
}

void twi_reset_state(void) {
    twi.cmd = TWI_CMD_SLEEP;
    twi.task = TWI_CMD_NONE;
    twi.temp = TWI_TEMP_INVALID;
    twi.weight.sum = 0;
    twi.weight.count = 0;
}

bool twi_check_busy(void) {
    LOCKI();
    bool busy = twi.busy;
    twi.busy = false;
    UNLOCKI();
    return busy;
}

void twi_set_temperature(int8_t t) {
    twi.temp = t;
}

int8_t twi_get_temperature(void) {
    return twi.temp;
}

void twi_add_weight(uint32_t w) {
    LOCKI();
    uint32_t sum = twi.weight.sum + (uint32_t)w;
    uint8_t n = twi.weight.count;

    // This should never happen.
    // But just in case we squeeze previous data down.
    if (n == 0xff) {
        sum /= 2;
        // NOTE: Old counter is odd.
        // n / 2 + 1 == (n + 1) / 2
        n /= 2;
    }

    ++n;

    twi.weight.sum = sum;
    twi.weight.count = n;
    UNLOCKI();
}

uint8_t twi_get_weight(uint32_t *w) {
    LOCKI();
    *w = twi.weight.sum;
    uint8_t n = twi.weight.count;
    UNLOCKI();
    return n;
}

uint8_t twi_get_last(uint32_t *w) {
    LOCKI();
    *w = twi.last.sum;
    uint8_t n = twi.last.count;
    twi.last.count = 0;
    UNLOCKI();
    return n;
}

/// Start condition interrupt.
ISR(USI_START_vect) {
    // Ensure SDA is input (in case we got interrupted during transmission)
    SET_INPUT(SDA_BIT);

    // Wait for end of start condition
    uint8_t pins;
    do {
        pins = PINB & TWI_BITS;
    } while (pins == SCL_BIT);

    if ((pins & SCL_BIT) == 0) {
        USICR = USICR_AWAIT_DATA;
        twi.state = TWI_CHECK_ADDR;
    } else {
        // Stop condition occurred
        USICR = USICR_AWAIT_START;
    }

    USISR = USISR_RESET;
}

static inline void twi_reset(void) {
    SET_INPUT(SDA_BIT);
    USICR = USICR_AWAIT_START;
    USISR = USISR_SHIFT_8BIT;
}

static inline void twi_ack(void) {
    USIDR = 0;
    SET_OUTPUT(SDA_BIT);
    USISR = USISR_SHIFT_1BIT;
}

static inline void twi_nack(void) {
    SET_INPUT(SDA_BIT);
    USISR = USISR_SHIFT_1BIT;
}

static inline void twi_read_ack(void) {
    SET_INPUT(SDA_BIT);
    USIDR = 0;
    USISR = USISR_SHIFT_1BIT;
}

static inline void twi_send(void) {
    USIDR = twi.buf[twi.index];
    SET_OUTPUT(SDA_BIT);
    USISR = USISR_SHIFT_8BIT;
    ++twi.index;
}

static inline void twi_recv(void) {
    SET_INPUT(SDA_BIT);
    USISR = USISR_SHIFT_8BIT;
}

static inline bool prepare_send(void) {
    twi.index = 0;
    switch (twi.cmd) {
    case TWI_CMD_MEASURE_WEIGHT: {
        uint8_t n = twi.weight.count;
        if (n == 0) {
            twi.count = 0;
            return false;
        }
        uint32_t w = twi.weight.sum;
        twi.last.count = n;
        twi.last.sum = w;
        twi.weight.count = 0;
        twi.weight.sum = 0;
        twi.buf[0] = n;
        twi.buf[4] = (uint8_t)(w);
        twi.buf[3] = (uint8_t)(w >> 8);
        twi.buf[2] = (uint8_t)(w >> 16);
        twi.buf[1] = (uint8_t)(w >> 24);
        twi.count = 5;
        break;
    }
    case TWI_CMD_GET_TEMP:
        if (twi.temp == TWI_TEMP_INVALID) {
            twi.count = 0;
            return false;
        }
        twi.buf[0] = twi.temp;
        twi.count = 1;
        break;
    case TWI_CMD_GET_CALIB:
        memcpy(twi.buf, &calib_data, sizeof(calib_data));
        twi.count = sizeof(calib_data);
        break;
    default: twi.count = 0; return false;
    }
    return true;
}

static inline bool process_cmd(void) {
    twi.index = 0;
    switch (twi.cmd) {
    case TWI_CMD_SET_TEMP: twi.count = 1; break;
    case TWI_CMD_SET_CALIB: twi.count = sizeof(calib_data); break;
    case TWI_CMD_SET_ADDR: twi.count = 1; break;
    case TWI_CMD_CALIB_WRITE:
        // fallthrough
    case TWI_CMD_ADDR_WRITE:
        // fallthrough
    case TWI_CMD_DISABLE_WD: twi.count = 1; break;
    default:
        twi.task = twi.cmd;
        twi.count = 0;
        return false;
    }
    return true;
}

static inline bool push_byte(uint8_t byte) {
    if (twi.index < twi.count) {
        twi.buf[twi.index] = byte;
        ++twi.index;
    }
    return twi.index < twi.count;
}

static inline void store_data(void) {
    switch (twi.cmd) {
    case TWI_CMD_SET_TEMP:
        twi.temp = twi.buf[0];
        twi.task = twi.cmd;
        break;
    case TWI_CMD_SET_CALIB: memcpy(&calib_data, twi.buf, twi.index); break;
    case TWI_CMD_SET_ADDR: twi_addr = twi.buf[0]; break;
    case TWI_CMD_ADDR_WRITE:
        if (twi.buf[0] == TWI_CONFIRM_ADDR_WRITE)
            twi.task = twi.cmd;
        break;
    case TWI_CMD_CALIB_WRITE:
        if (twi.buf[0] == TWI_CONFIRM_CALIB_WRITE)
            twi.task = twi.cmd;
        break;
    case TWI_CMD_DISABLE_WD:
        if (twi.buf[0] == TWI_CONFIRM_DISABLE_WD)
            twi.task = twi.cmd;
        break;
    }
}

/// Counter overflow interrupt.
ISR(USI_OVF_vect) {
    switch (twi.state) {
    case TWI_CHECK_ADDR: {
        uint8_t data = USIDR;
        if ((data >> 1) == twi_addr) {
            if ((data & 0x01) == 0) {
                twi.state = TWI_GET_CMD;
                twi_ack();
            } else if (prepare_send()) {
                twi.state = TWI_SEND_DATA;
                twi_ack();
            } else {
                // no data to send
            }
        } else if (data == 0) {
            // TODO: general call
        } else {
            // not our business
        }

        if (twi.state != TWI_CHECK_ADDR) {
            twi.busy = true;
        } else {
            twi_reset();
            // go right back to sleep when we have nothing to do
            if (twi.cmd == TWI_CMD_SLEEP)
                twi.task = twi.cmd;
        }
        break;
    }

    case TWI_GET_CMD:
        twi.state = TWI_STORE_CMD;
        twi_recv();
        break;

    case TWI_STORE_CMD:
        twi.cmd = USIDR;
        twi.state = process_cmd() ? TWI_GET_DATA : TWI_DONE;
        twi_ack();
        break;

    case TWI_GET_DATA:
        twi.state = TWI_STORE_DATA;
        twi_recv();
        break;

    case TWI_STORE_DATA:
        if (push_byte(USIDR)) {
            twi.state = TWI_GET_DATA;
        } else {
            store_data();
            twi.state = TWI_DONE;
        }
        twi_ack();
        break;

    case TWI_CHECK_ACK:
        if (USIDR) {
            twi_reset();
            break;
        }
        // fallthrough
    case TWI_SEND_DATA:
        if (twi.index >= twi.count) {
            twi_reset();
            break;
        }
        twi_send();
        twi.state = TWI_READ_ACK;
        break;

    case TWI_READ_ACK:
        twi.state = TWI_CHECK_ACK;
        twi_read_ack();
        break;

    case TWI_DONE: twi_reset(); break;

    default: break;
    }
}

#else
#error "This AVR is not supported"
#endif
