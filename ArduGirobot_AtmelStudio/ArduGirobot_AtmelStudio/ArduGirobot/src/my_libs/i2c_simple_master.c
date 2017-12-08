/**
 * \file
 *
 * \brief I2C master driver.
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

#include <i2c_master.h>
#include <i2c_simple_master.h>

static i2c_operations_t wr1RegCompleteHandler(void *p)
{
	I2C_MASTER_set_buffer(p, 1);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_continue;
}

void I2C_MASTER_write1ByteRegister(i2c_address_t address, uint8_t reg, uint8_t data)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(wr1RegCompleteHandler, &data);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

void I2C_MASTER_writeNBytes(i2c_address_t address, void *data, size_t len)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_buffer(data, len);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

static i2c_operations_t rd1RegCompleteHandler(void *p)
{
	I2C_MASTER_set_buffer(p, 1);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

uint8_t I2C_MASTER_read1ByteRegister(i2c_address_t address, uint8_t reg)
{
	uint8_t     d2 = 42;
	i2c_error_t e;
	int         x;

	for (x = 2; x != 0; x--) {
		while (!I2C_MASTER_open(address))
			; // sit here until we get the bus..
		I2C_MASTER_set_data_complete_callback(rd1RegCompleteHandler, &d2);
		I2C_MASTER_set_buffer(&reg, 1);
		I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
		I2C_MASTER_master_write();
		while (I2C_BUSY == (e = I2C_MASTER_close()))
			; // sit here until finished.
		if (e == I2C_NOERR)
			break;
	}

	return d2;
}

static i2c_operations_t rd2RegCompleteHandler(void *p)
{
	I2C_MASTER_set_buffer(p, 2);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

uint16_t I2C_MASTER_read2ByteRegister(i2c_address_t address, uint8_t reg)
{
	// result is little endian
	uint16_t result;

	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(rd2RegCompleteHandler, &result);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.

	return (result << 8 | result >> 8);
}

/****************************************************************/
static i2c_operations_t wr2RegCompleteHandler(void *p)
{
	I2C_MASTER_set_buffer(p, 2);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_continue;
}

void I2C_MASTER_write2ByteRegister(i2c_address_t address, uint8_t reg, uint16_t data)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(wr2RegCompleteHandler, &data);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

/****************************************************************/
typedef struct {
	size_t len;
	char * data;
} buf_t;

static i2c_operations_t rdBlkRegCompleteHandler(void *p)
{
	I2C_MASTER_set_buffer(((buf_t *)p)->data, ((buf_t *)p)->len);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

void I2C_MASTER_readDataBlock(i2c_address_t address, uint8_t reg, void *data, size_t len)
{
	// result is little endian
	buf_t d;
	d.data = data;
	d.len  = len;

	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(rdBlkRegCompleteHandler, &d);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

void I2C_MASTER_readNBytes(i2c_address_t address, void *data, size_t len)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_buffer(data, len);
	I2C_MASTER_master_read();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}
