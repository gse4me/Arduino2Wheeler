/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/*
 This file provides some basic blocking helper functions for common operations on the i2c API
 */

#include "i2c_simple.h"

typedef struct {
	size_t len;
	char * data;
} buf_t;

static i2c_operations_t I2C_SIMPLE_MASTER_0_wr_1_reg_complete_handler(void *p)
{
	I2C_MASTER_set_buffer(p, 1);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);

	return i2c_continue;
}

void I2C_SIMPLE_MASTER_0_write_1_byte_register(i2c_address_t address, uint8_t reg, uint8_t data)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(I2C_SIMPLE_MASTER_0_wr_1_reg_complete_handler, &data);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

static i2c_operations_t I2C_SIMPLE_MASTER_0_rd_1_reg_complete_handler(void *p)
{
	I2C_MASTER_set_buffer(p, 1);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

uint8_t I2C_SIMPLE_MASTER_0_read_1_byte_register(i2c_address_t address, uint8_t reg)
{
	uint8_t     d2 = 42;
	i2c_error_t e;

	for (int x = 2; x != 0; x--) {
		while (!I2C_MASTER_open(address))
			; // sit here until we get the bus..
		I2C_MASTER_set_data_complete_callback(I2C_SIMPLE_MASTER_0_rd_1_reg_complete_handler, &d2);
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

/****************************************************************/
static i2c_operations_t I2C_SIMPLE_MASTER_0_rd_2_reg_complete_handler(void *p)
{
	I2C_MASTER_set_buffer(p, 2);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

uint16_t I2C_SIMPLE_MASTER_0_read_2_byte_register(i2c_address_t address, uint8_t reg)
{
	// result is little endian
	uint16_t result;

	while (I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(I2C_SIMPLE_MASTER_0_rd_2_reg_complete_handler, &result);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.

	return (result << 8 | result >> 8);
}

/****************************************************************/
static i2c_operations_t I2C_SIMPLE_MASTER_0_wr_2_reg_complete_handler(void *p)
{
	I2C_MASTER_set_buffer(p, 2);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_continue;
}

void I2C_SIMPLE_MASTER_0_write_2_byte_register(i2c_address_t address, uint8_t reg, uint16_t data)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_data_complete_callback(I2C_SIMPLE_MASTER_0_wr_2_reg_complete_handler, &data);
	I2C_MASTER_set_buffer(&reg, 1);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

static i2c_operations_t I2C_SIMPLE_MASTER_0_rd_blk_reg_complete_handler(void *p)
{
	I2C_MASTER_set_buffer(((buf_t *)p)->data, ((buf_t *)p)->len);
	I2C_MASTER_set_data_complete_callback(NULL, NULL);
	return i2c_restart_read;
}

void I2C_SIMPLE_MASTER_0_write_block(i2c_address_t address, uint8_t *data, size_t len)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_buffer(data, len);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_write();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}

void I2C_SIMPLE_MASTER_0_read_block(i2c_address_t address, uint8_t *data, size_t len)
{
	while (!I2C_MASTER_open(address))
		; // sit here until we get the bus..
	I2C_MASTER_set_buffer(data, len);
	I2C_MASTER_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_MASTER_master_read();
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until finished.
}
