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
#include <i2c_types.h>
#include <driver_init.h>
#include <stdbool.h>
#include <stdlib.h>
// #include "timeout.h"  // TODO: Add timeout integration

/***************************************************************************/
// I2C STATES
typedef enum {
	I2C_IDLE = 0,
	I2C_SEND_ADR_READ,
	I2C_SEND_ADR_WRITE,
	I2C_TX,
	I2C_RX,
	I2C_TX_EMPTY,
	I2C_SEND_RESTART_READ,
	I2C_SEND_RESTART_WRITE,
	I2C_SEND_RESTART,
	I2C_SEND_STOP,
	I2C_RX_DO_ACK,
	I2C_TX_DO_ACK,
	I2C_RX_DO_NACK_STOP,
	I2C_RX_DO_NACK_RESTART,
	I2C_RESET,
	I2C_ADDRESS_NACK,
	I2C_BUS_COLLISION,
	I2C_BUS_ERROR
} i2c_fsm_states_t;

// I2C Event Callback List
typedef enum {
	i2c_dataComplete = 0,
	i2c_writeCollision,
	i2c_addressNACK,
	i2c_dataNACK,
	i2c_timeOut,
	i2c_NULL
} i2c_callback_index;

// I2C Status Structure
typedef struct {
	unsigned         busy : 1;
	unsigned         inUse : 1;
	unsigned         bufferFree : 1;
	unsigned         addressNACKCheck : 1;
	i2c_address_t    address;       /// The I2C Address
	uint8_t *        data_ptr;      /// pointer to a data buffer
	size_t           data_length;   /// Bytes in the data buffer
	uint16_t         timeout;       /// I2C Timeout Counter between I2C Events.
	uint16_t         timeout_value; /// Reload value for the timeouts
	i2c_fsm_states_t state;         /// Driver State
	i2c_error_t      error;
	/*if timeoutDriverEnabled
	timerStruct_t timeout;
	*/
	i2c_callback callbackTable[6];
	void *       callbackPayload[6]; ///  each callback can have a payload
} i2c_status_t;

typedef i2c_fsm_states_t(stateHandlerFunction)(void);

i2c_status_t I2C_MASTER_status = {0};

static void I2C_MASTER_set_callback(i2c_callback_index idx, i2c_callback cb, void *p);
static i2c_operations_t I2C_MASTER_return_stop(void *p);
static i2c_operations_t I2C_MASTER_return_reset(void *p);
static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_ADR_READ(void);
static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_ADR_WRITE(void);
static void             I2C_MASTER_master_isr(void);

/*if timeoutDriverEnabled>
ABSOLUTETIME_t ${i2cMasterFunctions["timeoutHandler"]}(void *p);

// place this function someplace in a periodic interrupt
ABSOLUTETIME_t ${i2cMasterFunctions["timeoutHandler"]}(void *p)
{
    ${msspI2cFunctions["disableIRQ"]}();
    ${i2cMasterFunctions["status"]}.state = I2C_RESET; // Jump to the Timeout state
    ${msspI2cFunctions["enableIRQ"]}();
    ${msspI2cFunctions["setIRQ"]}(); // force an interrupt to handle the timeout
    return 0;
}
*/

void I2C_MASTER_set_data_complete_callback(i2c_callback cb, void *p)
{
	I2C_MASTER_set_callback(i2c_dataComplete, cb, p);
}

void I2C_MASTER_set_write_collision_callback(i2c_callback cb, void *p)
{
	I2C_MASTER_set_callback(i2c_writeCollision, cb, p);
}

void I2C_MASTER_set_address_nack_callback(i2c_callback cb, void *p)
{
	I2C_MASTER_set_callback(i2c_addressNACK, cb, p);
}

void I2C_MASTER_set_data_nack_callback(i2c_callback cb, void *p)
{
	I2C_MASTER_set_callback(i2c_dataNACK, cb, p);
}

void I2C_MASTER_set_timeout_callback(i2c_callback cb, void *p)
{
	I2C_MASTER_set_callback(i2c_timeOut, cb, p);
}

void I2C_MASTER_init()
{

	/* Enable TWI */
	PRR &= ~(1 << PRTWI);

	TWCR = (1 << TWEN)   /* TWI: enabled */
	       | (1 << TWIE) /* TWI Interrupt: enabled */
	       | (1 << TWEA) /* TWI Acknowledge: enabled */;

	/* SCL bitrate = F_CPU / (16 + 2 * TWBR * TWPS value) */
	/* Configured bit rate is 400.000kHz, based on CPU frequency 8.000MHz */
	TWBR = 0x02;          /* SCL bit rate: 400.000kHZ before prescaling */
	TWSR = 0x00 << TWPS0; /* SCL precaler: 1, effective bitrate = 400.000kHz */
}

// when you call open, you supply a device address.
// if you get the bus, the function returns true
i2c_error_t I2C_MASTER_open(i2c_address_t address)
{
	i2c_error_t ret = I2C_BUSY;

	if (!I2C_MASTER_status.inUse) {
		I2C_MASTER_status.address          = address;
		I2C_MASTER_status.busy             = 0;
		I2C_MASTER_status.inUse            = 1;
		I2C_MASTER_status.addressNACKCheck = 0;
		I2C_MASTER_status.state            = I2C_RESET;
		I2C_MASTER_status.timeout_value    = 500; // MCC should determine a reasonable starting value here.
		I2C_MASTER_status.bufferFree       = 1;
		/*
        <#if timeoutDriverEnabled>
        I2C_MASTER_status.timeout.callbackPtr = ${i2cMasterFunctions["timeoutHandler"]};
        </#if>
*/
		// set all the call backs to a default of sending stop
		I2C_MASTER_status.callbackTable[i2c_dataComplete]     = I2C_MASTER_return_stop;
		I2C_MASTER_status.callbackPayload[i2c_dataComplete]   = NULL;
		I2C_MASTER_status.callbackTable[i2c_writeCollision]   = I2C_MASTER_return_stop;
		I2C_MASTER_status.callbackPayload[i2c_writeCollision] = NULL;
		I2C_MASTER_status.callbackTable[i2c_addressNACK]      = I2C_MASTER_return_stop;
		I2C_MASTER_status.callbackPayload[i2c_addressNACK]    = NULL;
		I2C_MASTER_status.callbackTable[i2c_dataNACK]         = I2C_MASTER_return_stop;
		I2C_MASTER_status.callbackPayload[i2c_dataNACK]       = NULL;
		I2C_MASTER_status.callbackTable[i2c_timeOut]          = I2C_MASTER_return_reset;
		I2C_MASTER_status.callbackPayload[i2c_timeOut]        = NULL;

		// Reset bus by sending STOP
		TWCR = ((1 << TWSTO) | (1 << TWINT));
		// Reset module
		TWCR = (1 << TWINT) | (1 << TWEN);

		// uncomment the IRQ enable for an interrupt driven driver.
		TWCR |= (1 << TWIE);

		ret = I2C_NOERR;
	}
	return ret;
}

void I2C_MASTER_set_address(i2c_address_t address)
{
	I2C_MASTER_status.address = address;
}

// close the bus if it is not busy
i2c_error_t I2C_MASTER_close(void)
{
	i2c_error_t ret = I2C_BUSY;

	if (!I2C_MASTER_status.busy) {
		I2C_MASTER_status.inUse = 0;
		// close it down
		I2C_MASTER_status.address = 0xff; // 8-bit address is invalid so this is FREE
		// Clearing INT flag starts operation
		TWCR |= (1 << TWINT);
		TWCR &= ~(1 << TWIE);
		ret = I2C_MASTER_status.error;
	}
	return ret;
}

void I2C_MASTER_set_timeout(uint8_t to)
{
	TWCR &= ~(1 << TWIE);
	I2C_MASTER_status.timeout_value = to;
	TWCR |= (1 << TWIE);
}

void I2C_MASTER_set_buffer(void *buffer, size_t bufferSize)
{
	if (I2C_MASTER_status.bufferFree) {
		I2C_MASTER_status.data_ptr    = buffer;
		I2C_MASTER_status.data_length = bufferSize;
		I2C_MASTER_status.bufferFree  = false;
	}
}
i2c_error_t I2C_MASTER_master_operation(bool read)
{
	i2c_error_t ret = I2C_BUSY;
	if (!I2C_MASTER_status.busy) {
		I2C_MASTER_status.busy = true;
		ret                    = I2C_NOERR;

		if (read) {
			I2C_MASTER_status.state = I2C_SEND_ADR_READ;
		} else {
			I2C_MASTER_status.state = I2C_SEND_ADR_WRITE;
		}
		TWCR |= ((1 << TWSTA) | (1 << TWINT));
	}
	return ret;
}

i2c_error_t I2C_MASTER_master_read(void)
{
	return I2C_MASTER_master_operation(true);
}

i2c_error_t I2C_MASTER_master_write(void)
{
	return I2C_MASTER_master_operation(false);
}

/************************************************************************/
/* Helper Functions                                                     */
/************************************************************************/

static i2c_fsm_states_t I2C_MASTER_do_I2C_RESET(void)
{
	// Reset bus by sending STOP
	TWCR                    = ((1 << TWSTO) | (1 << TWINT));
	I2C_MASTER_status.busy  = false; // Bus Free
	I2C_MASTER_status.error = I2C_NOERR;
	return I2C_RESET; // park the FSM on reset
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_IDLE(void)
{
	I2C_MASTER_status.busy  = false; // Bus Free
	I2C_MASTER_status.error = I2C_NOERR;
	return I2C_IDLE; // park the FSM on IDLE
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_RESTART_READ(void)
{
	TWCR |= ((1 << TWSTA) | (1 << TWINT));
	return I2C_SEND_ADR_READ;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_RESTART_WRITE(void)
{
	TWCR |= ((1 << TWSTA) | (1 << TWINT));
	return I2C_SEND_ADR_WRITE;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_RESTART(void)
{
	TWCR |= ((1 << TWSTA) | (1 << TWINT));
	return I2C_SEND_ADR_READ;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_STOP(void)
{
	TWCR |= ((1 << TWSTO) | (1 << TWINT));
	return I2C_MASTER_do_I2C_IDLE();
}

// TODO: probably need 2 addressNACK's one from read and one from write.
//       the do NACK before RESTART or STOP is a special case that a new state simplifies.
static i2c_fsm_states_t I2C_MASTER_do_I2C_DO_ADDRESS_NACK(void)
{

	I2C_MASTER_status.error = I2C_FAIL;
	switch (I2C_MASTER_status.callbackTable[i2c_addressNACK](I2C_MASTER_status.callbackPayload[i2c_addressNACK])) {
	case i2c_restart_read:
		return I2C_MASTER_do_I2C_SEND_RESTART_READ();
	case i2c_restart_write:
		return I2C_MASTER_do_I2C_SEND_RESTART_WRITE();
	default:
		return I2C_MASTER_do_I2C_SEND_STOP();
	}
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_ADR_READ(void)
{
	// Check if START or REPSTART was successfully sent
	if (!((((TWSR & 0xF8) == 0x08) || ((TWSR & 0xF8) == 0x10))))
		return I2C_MASTER_do_I2C_RESET();

	TWDR = I2C_MASTER_status.address << 1 | 1;
	// Clear STArt bit
	TWCR &= ~(1 << TWSTA);
	// Clearing INT flag starts operation
	TWCR |= (1 << TWINT);
	return I2C_RX_DO_ACK;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_SEND_ADR_WRITE(void)
{
	// Check if START or REPSTART was successfully sent
	if (!((((TWSR & 0xF8) == 0x08) || ((TWSR & 0xF8) == 0x10))))
		return I2C_MASTER_do_I2C_RESET();

	TWDR = I2C_MASTER_status.address << 1;
	// Clear STArt bit
	TWCR &= ~(1 << TWSTA);
	// Clearing INT flag starts operation
	TWCR |= (1 << TWINT);
	return I2C_TX_DO_ACK;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_RX_DO_ACK(void)
{
	// Check if SLA+R received NACK
	if (!(((TWSR & 0xF8) == 0x40)))
		return I2C_MASTER_do_I2C_DO_ADDRESS_NACK();

	if (I2C_MASTER_status.data_length == 1)
		TWCR &= ~(1 << TWEA);
	else
		TWCR |= (1 << TWEA);
	return I2C_RX;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_TX_DO_ACK(void)
{
	// Check if SLA+W received NACK
	if (!(((TWSR & 0xF8) == 0x18)))
		return I2C_MASTER_do_I2C_DO_ADDRESS_NACK();
	return I2C_TX;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_DO_NACK_STOP(void)
{
	TWCR |= ((1 << TWSTO) | (1 << TWINT));
	return I2C_MASTER_do_I2C_IDLE();
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_DO_NACK_RESTART(void)
{
	TWCR |= ((1 << TWSTA) | (1 << TWINT));
	return I2C_SEND_RESTART;
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_TX(void)
{
	if ((((TWSR & 0xF8) == 0x30) || ((TWSR & 0xF8) == 0x48) || ((TWSR & 0xF8) == 0x20))) // Slave replied with NACK
	{
		switch (I2C_MASTER_status.callbackTable[i2c_dataNACK](I2C_MASTER_status.callbackPayload[i2c_dataNACK])) {
		case i2c_restart_read:
			return I2C_MASTER_do_I2C_SEND_RESTART_READ();
		case i2c_restart_write:
			return I2C_MASTER_do_I2C_SEND_RESTART_WRITE();
		default:
		case i2c_continue:
		case i2c_stop:
			return I2C_MASTER_do_I2C_SEND_STOP();
		}
	} else {

		TWDR = *I2C_MASTER_status.data_ptr++;
		// Clearing INT flag starts operation
		TWCR |= (1 << TWINT);
		return (--I2C_MASTER_status.data_length) ? I2C_TX : I2C_TX_EMPTY;
	}
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_RX(void)
{

	if (((TWSR & 0xF8) != 0x50) && ((TWSR & 0xF8) != 0x58))
		return I2C_MASTER_do_I2C_RESET(); // Unexpected status code, reset bus

	// Data byte has been received, ACK or NACK has been returned

	if (I2C_MASTER_status.data_length == 2)
		TWCR &= ~(1 << TWEA); // Next byte will be last to be received, setup NACK
	else
		TWCR |= (1 << TWEA); // More bytes to receive, setup ACK

	if (--I2C_MASTER_status.data_length) {
		*I2C_MASTER_status.data_ptr = TWDR;
		I2C_MASTER_status.data_ptr++;
		// Clearing INT flag starts operation
		TWCR |= (1 << TWINT);
		return I2C_RX;
	} else {
		*I2C_MASTER_status.data_ptr = TWDR;
		I2C_MASTER_status.data_ptr++;
		I2C_MASTER_status.bufferFree = true;
		switch (
		    I2C_MASTER_status.callbackTable[i2c_dataComplete](I2C_MASTER_status.callbackPayload[i2c_dataComplete])) {
		case i2c_restart_write:
		case i2c_restart_read:
			return I2C_MASTER_do_I2C_DO_NACK_RESTART();
		default:
		case i2c_continue:
		case i2c_stop:
			return I2C_MASTER_do_I2C_DO_NACK_STOP();
		}
	}
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_TX_EMPTY(void)
{
	if ((((TWSR & 0xF8) == 0x30) || ((TWSR & 0xF8) == 0x48) || ((TWSR & 0xF8) == 0x20))) // Slave replied with NACK
	{
		switch (I2C_MASTER_status.callbackTable[i2c_dataNACK](I2C_MASTER_status.callbackPayload[i2c_dataNACK])) {
		case i2c_restart_read:
			return I2C_MASTER_do_I2C_SEND_RESTART_READ();
		case i2c_restart_write:
			return I2C_MASTER_do_I2C_SEND_RESTART_WRITE();
		default:
		case i2c_continue:
		case i2c_stop:
			return I2C_MASTER_do_I2C_SEND_STOP();
		}
	} else {
		I2C_MASTER_status.bufferFree = true;
		switch (
		    I2C_MASTER_status.callbackTable[i2c_dataComplete](I2C_MASTER_status.callbackPayload[i2c_dataComplete])) {
		case i2c_restart_read:
			return I2C_MASTER_do_I2C_SEND_RESTART_READ();
		case i2c_restart_write:
			return I2C_MASTER_do_I2C_SEND_RESTART_WRITE();
		case i2c_continue:
			return I2C_MASTER_do_I2C_TX();
		default:
		case i2c_stop:
			return I2C_MASTER_do_I2C_SEND_STOP();
		}
	}
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_BUS_COLLISION(void)
{
	;
	I2C_MASTER_status.error = I2C_FAIL;
	switch (
	    I2C_MASTER_status.callbackTable[i2c_writeCollision](I2C_MASTER_status.callbackPayload[i2c_writeCollision])) {
	case i2c_restart_read:
		return I2C_MASTER_do_I2C_SEND_RESTART_READ();
	case i2c_restart_write:
		return I2C_MASTER_do_I2C_SEND_RESTART_WRITE();
	default:
		return I2C_MASTER_do_I2C_RESET();
	}
}

static i2c_fsm_states_t I2C_MASTER_do_I2C_BUS_ERROR(void)
{
	// Reset bus by sending STOP
	TWCR                    = ((1 << TWSTO) | (1 << TWINT));
	I2C_MASTER_status.busy  = false;
	I2C_MASTER_status.error = I2C_FAIL;
	return I2C_RESET; // park the FSM on reset
}

stateHandlerFunction *I2C_MASTER_fsmStateTable[] = {
    I2C_MASTER_do_I2C_IDLE,               // I2C_IDLE
    I2C_MASTER_do_I2C_SEND_ADR_READ,      // I2C_SEND_ADR_READ
    I2C_MASTER_do_I2C_SEND_ADR_WRITE,     // I2C_SEND_ADR_WRITE
    I2C_MASTER_do_I2C_TX,                 // I2C_TX
    I2C_MASTER_do_I2C_RX,                 // I2C_RX
    I2C_MASTER_do_I2C_TX_EMPTY,           // I2C_TX_EMPTY
    I2C_MASTER_do_I2C_SEND_RESTART_READ,  // I2C_SEND_RESTART_READ
    I2C_MASTER_do_I2C_SEND_RESTART_WRITE, // I2C_SEND_RESTART_WRITE
    I2C_MASTER_do_I2C_SEND_RESTART,       // I2C_SEND_RESTART
    I2C_MASTER_do_I2C_SEND_STOP,          // I2C_SEND_STOP
    I2C_MASTER_do_I2C_RX_DO_ACK,          // I2C_RX_DO_ACK
    I2C_MASTER_do_I2C_TX_DO_ACK,          // I2C_TX_DO_ACK
    I2C_MASTER_do_I2C_DO_NACK_STOP,       // I2C_RX_DO_NACK_STOP
    I2C_MASTER_do_I2C_DO_NACK_RESTART,    // I2C_RX_DO_NACK_RESTART
    I2C_MASTER_do_I2C_RESET,              // I2C_RESET
    I2C_MASTER_do_I2C_DO_ADDRESS_NACK,    // I2C_ADDRESS_NACK
    I2C_MASTER_do_I2C_BUS_COLLISION,      // I2C_BUS_COLLISION
    I2C_MASTER_do_I2C_BUS_ERROR           // I2C_BUS_ERROR
};

ISR(TWI_vect)
{
	I2C_MASTER_master_isr();
}

void I2C_MASTER_master_isr(void)
{

	// NOTE: We are ignoring the Write Collision flag.

	// Bus arbitration lost to another master, override next state
	if (((TWSR & 0xF8) == 0x38) || ((TWSR & 0xF8) == 0x68) || ((TWSR & 0xF8) == 0x78) || ((TWSR & 0xF8) == 0xB0)) {
		I2C_MASTER_status.state = I2C_BUS_COLLISION; // State Override
	}

	I2C_MASTER_status.state = I2C_MASTER_fsmStateTable[I2C_MASTER_status.state]();
}

/************************************************************************/
/* Helper Functions                                                     */
/************************************************************************/
static i2c_operations_t I2C_MASTER_return_stop(void *p)
{
	return i2c_stop;
}

static i2c_operations_t I2C_MASTER_return_reset(void *p)
{
	return i2c_reset_link;
}

static void I2C_MASTER_set_callback(i2c_callback_index idx, i2c_callback cb, void *p)
{
	if (cb) {
		I2C_MASTER_status.callbackTable[idx]   = cb;
		I2C_MASTER_status.callbackPayload[idx] = p;
	} else {
		I2C_MASTER_status.callbackTable[idx]   = I2C_MASTER_return_stop;
		I2C_MASTER_status.callbackPayload[idx] = NULL;
	}
}
