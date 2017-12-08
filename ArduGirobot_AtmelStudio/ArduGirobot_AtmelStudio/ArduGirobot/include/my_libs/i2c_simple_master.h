/**
 * \file
 *
 * \brief I2C Simple master driver.
 *
 *
 * Copyright (C) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 *
 */

/**
 * \defgroup doc_driver_i2c_simple_master I2C Simple Master Driver
 * \ingroup doc_driver_i2c
 *
 * \section doc_driver_i2c_simple_master_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#ifndef I2C_SIMPLE_MASTER_H
#define I2C_SIMPLE_MASTER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <i2c_types.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t I2C_MASTER_read1ByteRegister(i2c_address_t address, uint8_t reg);
uint16_t I2C_MASTER_read2ByteRegister(i2c_address_t address, uint8_t reg);
void I2C_MASTER_write1ByteRegister(i2c_address_t address, uint8_t reg, uint8_t data);
void I2C_MASTER_write2ByteRegister(i2c_address_t address, uint8_t reg, uint16_t data);

void I2C_MASTER_writeNBytes(i2c_address_t address, void *data, size_t len);
void I2C_MASTER_readDataBlock(i2c_address_t address, uint8_t reg, void *data, size_t len);
void I2C_MASTER_readNBytes(i2c_address_t address, void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* I2C_SIMPLE_MASTER_H_INCLUDED */
