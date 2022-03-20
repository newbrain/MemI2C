/*
 * Copyright (c) 2022 Federico Zuccardi Merli.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MEMI2C_H
#define _MEMI2C_H

#include "hardware/i2c.h"
#include "hardware/structs/i2c.h"
#include "pico.h"
#include "pico/time.h"

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_I2C, Enable/disable assertions in the I2C module, type=bool, default=0, group=hardware_i2c
#ifndef PARAM_ASSERTIONS_ENABLED_I2C
#define PARAM_ASSERTIONS_ENABLED_I2C 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

int i2c_write_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *src, size_t len);
int i2c_read_mem_blocking(i2c_inst_t *i2c, uint32_t i2caddr, uint32_t addr, size_t addrsize, uint8_t *dst, size_t len);

#ifdef __cplusplus
}
#endif

#endif