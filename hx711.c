/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#include "hx711.h"

#include <avr/io.h>

#include <util/delay.h>

#include <stdbool.h>
#include <stdio.h>

#define SERIAL_DDR  DDRB
#define SERIAL_PORT PORTB
#define SERIAL_PIN  PINB
#define PD_SCK      (1 << PB3)
#define DOUT        (1 << PB4)

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

uint32_t hx711_read(void) {
    // power up
    SERIAL_PORT &= ~PD_SCK;

    // wait for hx711 to become ready
    while ((SERIAL_PIN & DOUT) != 0)
        ;

    _delay_us(0.1);

    uint32_t result = 0;
    uint8_t i;
    // we are interested in all 24 bits
    for (i = 0; i < 24; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);

        result <<= 1;

        if ((SERIAL_PIN & DOUT) != 0)
            result |= 1;

        // flip first bit for unsigned value
        if (i == 0)
            result ^= 1;

        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    // pulse 25 times
    for (; i < 25; ++i) {
        SERIAL_PORT |= PD_SCK;
        _delay_us(0.2);
        SERIAL_PORT &= ~PD_SCK;
        _delay_us(0.2);
    }

    return result;
}
