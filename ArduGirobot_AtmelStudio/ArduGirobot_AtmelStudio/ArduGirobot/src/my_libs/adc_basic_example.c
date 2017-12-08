/**
 * \file
 *
 * \brief ADC Basic driver example.
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
#include <adc_basic_example.h>
#include <adc_basic.h>
#include <atomic.h>

volatile bool         ADC_0_isr_executed = false;
volatile adc_result_t ADC_0_measurement;
volatile uint8_t      ADC_0_measurement_normalized;

void ADC_0_adc_handler_cb(void)
{
	ADC_0_measurement            = ADC_0_get_conversion_result();
	ADC_0_measurement_normalized = ADC_0_measurement >> (ADC_0_get_resolution() - 8);
	ADC_0_isr_executed           = true;
}

uint8_t ADC_0_test_adc_basic(void)
{

	// Test driver functions, assume that an AIN channel is enabled and that
	// the Result Ready IRQ is enabled.

	// Test polled mode

	// Get conversion from specified ADC channel
	ADC_0_measurement = ADC_0_get_conversion(0);

	// Get 8 MSB of conversion result
	ADC_0_measurement_normalized = ADC_0_measurement >> (ADC_0_get_resolution() - 8);

	// Test IRQ mode

	ENABLE_INTERRUPTS();

	ADC_0_register_callback(ADC_0_adc_handler_cb);

	// make sure flag is false
	ADC_0_isr_executed = false;

	// Start conversion from ADC CH0
	ADC_0_start_conversion(0);

	// Wait for ISR to be execued
	while (!ADC_0_isr_executed)
		;

	DISABLE_INTERRUPTS();
	return 1;
}
