/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#pragma once

#include <stdint.h>

#pragma pack(push, 1)
struct calib_data {
    struct {
        int32_t pre_offset;
        int32_t post_offset;
        uint16_t scale;
        int16_t temp_scale;
    } hx711;

    struct {
        int16_t offset;
        uint8_t scale;
    } temperature;
};
#pragma pack(pop)

extern uint8_t twi_addr;
extern struct calib_data calib_data;

void nvm_init(void);
void nvm_write_calib_data(void);
void nvm_write_twi_addr(void);
