/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#include "hx711.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>

#define SERIAL_DDR          DDRB
#define SERIAL_PORT         PORTB
#define SERIAL_PIN          PINB
#define PD_SCK              (1 << PB3)
#define DOUT                (1 << PB4)
#define DOUT_PCINT          (1 << PCINT4)
#define MAX_PD_HIGH_TIME_US 50

void hx711_init(void) {
    SERIAL_DDR |= PD_SCK;
    SERIAL_DDR &= ~DOUT;

    // no pull up on DOUT
    SERIAL_PORT &= ~DOUT;

    hx711_powerdown();
}

void hx711_powerdown(void) {
    SERIAL_PORT |= PD_SCK;
    _delay_us(60);
}

ISR(PCINT0_vect) {
    // required for wake up during sleep
}

static inline void wait_for_dout(void) {
    // enable pin change interrupt on DOUT pin
    PCMSK = DOUT_PCINT;
    // enable pin change interrupt
    GIMSK |= (1 << PCIE);

    cli();
    // wait for DOUT to go low
    while ((SERIAL_PIN & DOUT) != 0) {
        // sleep while waiting for measurement
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
        cli();
    };
    sei();
    // disable pin change interrupt
    GIMSK &= ~(1 << PCIE);
}

static inline bool pdwatch_timeout(void) {
    return (TIFR & (1 << OCF0A)) != 0;
}

static inline void pdwatch_stop(void) {
    // timer sync mode
    GTCCR |= (1 << TSM) | (1 << PSR0);
}

static inline void pdwatch_restart(void) {
    TCNT0 = 0;
    GTCCR &= ~(1 << TSM);
}

/**
 *  Initialize timer0 to set compare match at given time.
 *
 * @param timeout in 1/8th clocks.
 */
static inline void pdwatch_timer_init(uint8_t timeout) {
    // enable timer0
    PRR &= ~(1 << PRTIM0);
    // disable timer0 interrupts
    TIMSK &= ~((1 << TOIE0) | (1 << OCIE0A) | (1 << OCIE0B));

    pdwatch_stop();
    // COM0A and COM0B are set to 0, disconnecting OC0A and OC0B
    // WGM01 and WGM00 are set to 0 for normal mode
    TCCR0A = 0;
    // WGM02 = 0
    // prescaler 1/8: CS02 = 0, CS01 = 1, CS00 = 0
    TCCR0B = (1 << CS01);

    OCR0A = timeout;
    TCNT0 = 0;
    // clear timer0 flags
    TIFR = (1 << OCF0A) | (1 << OCF0B) | (1 << TOV0);
}

static void pdwatch_timer_deinit(void) {
    // stop timer0
    TCCR0B = 0;
    // disable timer sync
    GTCCR &= ~(1 << TSM);
    // disable timer0
    PRR |= (1 << PRTIM0);
}

uint32_t hx711_read(void) {
    // power up
    SERIAL_PORT &= ~PD_SCK;

    // wait for hx711 to become ready
    wait_for_dout();

    _delay_us(0.1);

    // A timer is used to measure how long PD_SCK was held high.
    // In case we exceed 50us the state of HX711 is unreliable.
    // Start timer with compare match at 50us.
    pdwatch_timer_init(MAX_PD_HIGH_TIME_US * F_CPU / 8000000UL);

    uint32_t result = 0;
    uint8_t i;
    // we are interested in all 24 bits
    for (i = 0; i < 24 && !pdwatch_timeout(); ++i) {
        pdwatch_restart();
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);

        result <<= 1;

        if ((SERIAL_PIN & DOUT) != 0)
            result |= 1;

        // flip first bit for unsigned value
        if (i == 0)
            result ^= 1;

        SERIAL_PORT &= ~PD_SCK;
        pdwatch_stop();
        _delay_us(0.2);
    }

    // pulse 25 times
    for (; i < 25 && !pdwatch_timeout(); ++i) {
        pdwatch_restart();
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);
        SERIAL_PORT &= ~PD_SCK;
        pdwatch_stop();
        _delay_us(0.2);
    }

    return pdwatch_timeout() ? HX711_INVALID : result;
}
