/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#pragma once

#include <stdint.h>

#define HX711_INVALID 0xFFFFFFFFUL

/// Initialize pins for communication with HX711.
void hx711_init(void);
/// Power down HX711
void hx711_powerdown(void);
/// Power up HX711 and read measurement.
uint32_t hx711_read(void);
