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

/**
 * \defgroup doc_driver_i2c_master I2C Master Driver
 * \ingroup doc_driver_i2c
 *
 * \section doc_driver_i2c_master_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */

#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <i2c_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize I2C interface
 * If module is configured to disabled state, the clock to the I2C is disabled
 * if this is supported by the device's clock system.
 *
 * \return Initialization status.
 * \retval 0 the init was successful
 * \retval 1 the init was not successful
 */
void I2C_MASTER_init(void);

/**
 * \brief Open the I2C for communication
 *
 * \param[in] address The slave address to use in the transfer
 *
 * \return Initialization status.
 * \retval I2C_NOERR The I2C open was successful
 * \retval I2C_BUSY  The I2C open failed because the interface is busy
 * \retval I2C_FAIL  The I2C open failed with an error
 */
i2c_error_t I2C_MASTER_open(i2c_address_t address);

/**
 * \brief Close the I2C interface
 *
 * \return Status of close operation.
 * \retval I2C_NOERR The I2C open was successful
 * \retval I2C_BUSY  The I2C open failed because the interface is busy
 * \retval I2C_FAIL  The I2C open failed with an error
 */
i2c_error_t I2C_MASTER_close(void);

/**
 * \brief Start an operation on an opened I2C interface
 *
 * \param[in] read Set to true for read, false for write
 *
 * \return Status of operation
 * \retval I2C_NOERR The I2C open was successful
 * \retval I2C_BUSY  The I2C open failed because the interface is busy
 * \retval I2C_FAIL  The I2C open failed with an error
 */
i2c_error_t I2C_MASTER_master_operation(bool read);

/**
 * \brief Identical to I2C_MASTER_master_operation(false);
 */
i2c_error_t I2C_MASTER_master_write(void); // to be depreciated

/**
 * \brief Identical to I2C_MASTER_master_operation(true);
 */
i2c_error_t I2C_MASTER_master_read(void); // to be depreciated

/**
 * \brief Set timeout to be used for I2C operations. Uses the Timeout driver.
 *
 * \param[in] to Timeout in ticks
 *
 * \return Nothing
 */
void I2C_MASTER_set_timeout(uint8_t to);

/**
 * \brief Sets up the data buffer to use, and number of bytes to transfer
 *
 * \param[in] buffer Pointer to data buffer to use for read or write data
 * \param[in] bufferSize Number of bytes to read or write from slave
 *
 * \return Nothing
 */
void I2C_MASTER_set_buffer(void *buffer, size_t bufferSize);

// Event Callback functions.

/**
 * \brief Set callback to be called when all specifed data has been transferred.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] p  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C_MASTER_set_data_complete_callback(i2c_callback cb, void *p);

/**
 * \brief Set callback to be called when there has been a bus collision and arbitration was lost.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] p  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C_MASTER_set_write_collision_callback(i2c_callback cb, void *p);

/**
 * \brief Set callback to be called when the transmitted address was NACK'ed.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] p  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C_MASTER_set_address_nack_callback(i2c_callback cb, void *p);

/**
 * \brief Set callback to be called when the transmitted data was NACK'ed.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] p  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C_MASTER_set_data_nack_callback(i2c_callback cb, void *p);

/**
 * \brief Set callback to be called when there was a bus timeout.
 *
 * \param[in] cb Pointer to callback function
 * \param[in] p  Pointer to the callback function's parameters
 *
 * \return Nothing
 */
void I2C_MASTER_set_timeout_callback(i2c_callback cb, void *p);

#ifdef __cplusplus
}
#endif

#endif /* I2C_MASTER_H_INCLUDED */
