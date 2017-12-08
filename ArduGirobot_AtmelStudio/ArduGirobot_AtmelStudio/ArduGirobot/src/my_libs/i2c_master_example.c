/**
 * \file
 *
 * \brief I2C Master driver example.
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

#include <atmel_start.h>
#include <i2c_types.h>
#include <utils/atomic.h>

#define slave_adr 0x4f
#define slave_reg_adr 0x0

uint8_t read_data[2];

/** Structure passed into read_handler to describe the actions to be performed by the handler */
typedef struct {
	uint8_t *data;
	uint8_t  size;
} transfer_descriptor_t;

/** This callback is called when the initial write of the pointer register has finished.
    This callback controls the second phase of the I2C transaction, the read of the
    targeted register after a REPEATED START.
*/
i2c_operations_t I2C_MASTER_read_handler(void *d)
{
	transfer_descriptor_t *desc = (transfer_descriptor_t *)d;
	I2C_MASTER_set_buffer((void *)desc->data, desc->size);
	// Set callback to terminate transfer and send STOP after read is complete
	I2C_MASTER_set_data_complete_callback(i2c_cb_return_stop, NULL);
	return i2c_restart_read; // Send REPEATED START before read
}

/** Performs the following transfer sequence:
1. Send SLA+W, Data1
2. Send RepeatedStart, SLA+R, Read Data1, Read Data2
3. Send Stop

This transfer sequence is typically done to first write to the slave the address in
the slave to read from, thereafter to read N bytes from this address.
*/
void I2C_MASTER_do_transfer(uint8_t adr, uint8_t *data, uint8_t size)
{
	transfer_descriptor_t d = {data, size};
	while (!I2C_MASTER_open(slave_adr))
		; // sit here until we get the bus..

	// This callback specifies what to do after the first write operation has completed
	// The parameters to the callback are bundled together in the aggregate data type d.
	I2C_MASTER_set_data_complete_callback((void *)I2C_MASTER_read_handler, &d);
	// If we get an address NACK, then try again by sending SLA+W
	I2C_MASTER_set_address_nack_callback((void *)i2c_cb_restart_write, NULL);
	// Transmit specified number of bytes
	I2C_MASTER_set_buffer((void *)&adr, 1);
	// Start a Write operation
	I2C_MASTER_master_operation(false);
	while (I2C_BUSY == I2C_MASTER_close())
		; // sit here until the entire chained operation has finished
}

uint8_t I2C_MASTER_test_i2c_master(void)
{

	I2C_MASTER_do_transfer(slave_reg_adr, read_data, 2);
	// If we get here, everything was OK
	return 1;
}
