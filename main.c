/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#include "debug.h"
#include "hx711.h"
#include "nvm.h"
#include "twi.h"
#include "util.h"

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <stdio.h>

#define START_WATCHDOG() wdt_enable(WDTO_8S)

#define VALVE_DDR  DDRB
#define VALVE_PORT PORTB
#define VALVE_BIT  (1 << PB1)

static void early_init(void) {
    cli();
    // configure clock to 8Mhz
    CLKPR = (1 << CLKPCE); // change protection
    CLKPR = 0;             // divide by 1

    // disable all peripherals
    PRR = (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRUSI) | (1 << PRADC);

    // disable digital inputs on led pin
    DIDR0 = 1 << AIN1D;
}

static void valve_init(void) {
    // Set valve pin low
    VALVE_PORT &= ~VALVE_BIT;
    // Set valve pin as output
    VALVE_DDR |= VALVE_BIT;
}

ISR(ADC_vect) {
    // we need this interrupt only for wake up during SLEEP_MODE_ADC
}

static int8_t measure_temperature(void) {
    // disable power reduction for ADC
    PRR &= ~(1 << PRADC);

    // select internal 1.1V reference
    // select temperature sensor
    ADMUX =
        (1 << REFS1) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);

    // choose prescaler for at least 50kHz
    uint8_t scale = 7;
    while (scale > 2 && (1 << scale) > (F_CPU / 50000))
        --scale;

    // enable ADC with prescaler
    ADCSRA = (1 << ADEN) | scale;

    // wait 1ms for internal reference voltage to settle
    _delay_ms(1);

    // wait while ADC is busy
    while ((ADCSRA & (1 << ADSC)) != 0)
        ;

    // start conversion
    ADCSRA |= (1 << ADSC) | (1 << ADIE);

    while (true) {
        set_sleep_mode(SLEEP_MODE_ADC);
        // check updates
        cli();
        if ((ADCSRA & (1 << ADSC)) != 0) {
            // sleep while conversion is running
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        } else {
            sei();
            break;
        }
    }

    int16_t val = ADCL;
    val |= ((uint16_t)ADCH) << 8;

    // disable ADC
    ADCSRA = 0; //~(1 << ADEN);
    PRR |= (1 << PRADC);

    uint8_t m = calib_data.temperature.scale;
    uint16_t b = calib_data.temperature.offset;

    return (int8_t)((val * m - b) >> 6);
}

static inline uint32_t calculate_weight(uint32_t result, uint8_t temp) {
    result += calib_data.hx711.pre_offset;
    result *= calib_data.hx711.scale + calib_data.hx711.temp_scale * temp;
    result += calib_data.hx711.post_offset;
    result /= 256UL;
    return result;
}

static void measure_weight(int8_t temp) {
    LOG("measure at: %d\n", temp);

    while (true) {
        uint8_t task = twi_get_task();
        if (task != TWI_CMD_MEASURE_WEIGHT && task != TWI_CMD_NONE)
            break;
        uint32_t w = calculate_weight(hx711_read(), temp);
        if (twi_check_busy())
            wdt_reset();
        if (((w >> 22) & 0x3) != 0x3) {
            twi_add_weight(w);
            LOG("w: %lu\n", w);
        } else {
            LOG("ovf: %lu\n", w);
        }
    }
    hx711_powerdown();
}

static inline void open_valve(void) {
#ifdef NO_SERIAL
    VALVE_PORT |= VALVE_BIT;
#else
    LOG("opening valve\n");
#endif
}

static inline void close_valve(void) {
#ifdef NO_SERIAL
    VALVE_PORT &= ~VALVE_BIT;
#else
    LOG("close valve\n");
#endif
}

static void powerdown(void) {
    // stop watchdog
    wdt_disable();

    // stop timer0
    TCCR0B = 0;
    // stop timer1
    TCCR1 = 0;

    // disable all peripherals except USI
    PRR = (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRADC);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();

    // restart watchdog
    START_WATCHDOG();

    debug_init();
}

static void loop(void) {

    bool updatetemp = true;

    START_WATCHDOG();

    for (;;) {

        if (twi_check_busy())
            wdt_reset();

        cli();
        uint8_t task = twi_get_task();
        if (task != TWI_CMD_MEASURE_WEIGHT)
            twi_reset_task();
        sei();

        bool needtemp = false;
        switch (task) {
        case TWI_CMD_MEASURE_WEIGHT: needtemp = true; break;
        case TWI_CMD_GET_TEMP: needtemp = true; break;
        case TWI_CMD_SET_TEMP: updatetemp = false; break;
        }

        LOG("task: 0x%x\n", task);

        uint32_t lastw;
        uint8_t lastn = twi_get_last(&lastw);
        if (lastn > 0) {
            LOG("tx: n: 0x%x, w: 0x%lx\n", lastn, lastw);
        }

        if (task == TWI_CMD_SLEEP) {
            close_valve();
            LOG("power down\n");
            debug_stop();
            cli();
            // check for eventually task update
            task = twi_get_task();
            if (task == TWI_CMD_SLEEP || task == TWI_CMD_NONE) {
                twi_reset_state();
                powerdown();
                // update temperature after wakeup
                updatetemp = true;
            } else {
                sei();
            }
        } else if (task == TWI_CMD_OPEN_VALVE) {
            open_valve();
        } else if (task == TWI_CMD_CLOSE_VALVE) {
            close_valve();
        } else if (task == TWI_CMD_CALIB_WRITE) {
            LOG("writing calibration data ...");
            debug_stop();
            wdt_disable();
            nvm_write_calib_data();
            START_WATCHDOG();
            debug_init();
            LOG(" done.\n");
        } else if (task == TWI_CMD_ADDR_WRITE) {
            LOG("writing addr ...");
            debug_stop();
            wdt_disable();
            nvm_write_twi_addr();
            START_WATCHDOG();
            debug_init();
            LOG(" done.\n");
        } else if (task == TWI_CMD_ENABLE_WD) {
            START_WATCHDOG();
            LOG("wd enabled\n");
        } else if (task == TWI_CMD_DISABLE_WD) {
            wdt_disable();
            LOG("wd disabled\n");
        } else if (task == TWI_CMD_NONE) {
            // waiting for task
            LOG("idle\n");
            debug_stop();
            cli();
            if (twi_get_task() == TWI_CMD_NONE) {
                set_sleep_mode(SLEEP_MODE_IDLE);
                sleep_enable();
                sei();
                sleep_cpu();
                sleep_disable();
            }
            sei();
            debug_init();
        } else {

            if (updatetemp && needtemp) {
                debug_stop();
                int8_t temp = measure_temperature();
                debug_init();
                LOG("T: %d\n", temp);
                twi_set_temperature(temp);
                updatetemp = false;
            }

            if (task == TWI_CMD_MEASURE_WEIGHT) {
                int8_t temp = twi_get_temperature();
                measure_weight(temp);
            }
        }
    }
}

int main(void) {
    // save reset reason
    uint8_t mcusr = MCUSR;
    // clear reset flags for next reset
    MCUSR = 0;

    early_init();

    wdt_disable();

    // init
    {
#ifdef NO_SERIAL
        valve_init();
#endif
        debug_init();
        nvm_init();
        twi_init();
        hx711_init();
        sei();

        debug_dump_trace();
        debug_init_trace();
    }

    LOG("\nreset: 0x%x\n", mcusr);
    LOG("addr: %d\n", twi_addr);
    loop();

    return 0; /* never reached */
}
