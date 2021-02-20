/*
    SPDX-FileCopyrightText: 2021 Mathias Fiedler
    SPDX-License-Identifier: MIT
*/

#pragma once

#include <stdbool.h>
#include <stdint.h>

/// Reset value of temperature.
#define TWI_TEMP_INVALID (-128)
/// Confirmation byte for TWI_CMD_CALIB_WRITE.
#define TWI_CONFIRM_CALIB_WRITE 0x3A
/// Confirmation byte for TWI_CMD_ADDR_WRITE.
#define TWI_CONFIRM_ADDR_WRITE 0x6A
/// Confirmation byte for TWI_CMD_DISABLE_WD.
#define TWI_CONFIRM_DISABLE_WD 0x9A

/// TWI commands.
enum {
    TWI_CMD_SLEEP = 0x00,
    TWI_CMD_MEASURE_WEIGHT = 0x50,
    TWI_CMD_OPEN_VALVE = 0x51,
    TWI_CMD_CLOSE_VALVE = 0x52,
    TWI_CMD_GET_TEMP = 0x53,
    TWI_CMD_SET_TEMP = 0x54,
    TWI_CMD_GET_CALIB = 0x55,
    TWI_CMD_SET_CALIB = 0x56,
    TWI_CMD_ENABLE_WD = 0x57,
    TWI_CMD_CALIB_WRITE = 0xA0,
    TWI_CMD_SET_ADDR = 0xA3,
    TWI_CMD_ADDR_WRITE = 0xA6,
    TWI_CMD_DISABLE_WD = 0xA9,
    TWI_CMD_NONE = 0xFF,
};

/// Initialize TWI.
void twi_init(void);
/// Check if this device was addressed since the last call of this function.
bool twi_check_busy(void);
/// Get current task.
uint8_t twi_get_task(void);
/// Reset task to TWI_CMD_NONE.
void twi_reset_task(void);
/// Reset command, task, weight and temperature.
void twi_reset_state(void);
/// Add sample to weight accumulation.
void twi_add_weight(uint32_t w);
/// Set temperature.
void twi_set_temperature(int8_t t);
/// Get currently set temperature.
int8_t twi_get_temperature(void);
/// Get accumulated weight.
uint8_t twi_get_weight(uint32_t *w);
/// Get last send weight accumulation.
uint8_t twi_get_last(uint32_t *w);
