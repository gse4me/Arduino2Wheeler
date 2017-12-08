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

#ifndef I2C_SIMPLE_MASTER_H
#define I2C_SIMPLE_MASTER_H

#include <stdint.h>
#include <stdlib.h>

#include <i2c_master.h>

uint8_t I2C_SIMPLE_MASTER_0_read_1_byte_register(i2c_address_t address, uint8_t reg);
uint16_t I2C_SIMPLE_MASTER_0_read_2_byte_register(i2c_address_t address, uint8_t reg);
void I2C_SIMPLE_MASTER_0_write_1_byte_register(i2c_address_t address, uint8_t reg, uint8_t data);
void I2C_SIMPLE_MASTER_0_write_2_byte_register(i2c_address_t address, uint8_t reg, uint16_t data);

void I2C_SIMPLE_MASTER_0_write_block(i2c_address_t address, uint8_t *data, size_t len);
void I2C_SIMPLE_MASTER_0_read_block(i2c_address_t address, uint8_t *data, size_t len);

#endif /* I2C_SIMPLE_MASTER_H */
