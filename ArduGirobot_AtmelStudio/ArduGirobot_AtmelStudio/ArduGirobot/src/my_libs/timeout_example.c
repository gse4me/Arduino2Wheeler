/**
 * \file
 *
 * \brief Timeout driver example.
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

#include <stdio.h>
#include <string.h>
#include <atmel_start.h>
#include <timeout_example.h>
#include <timeout.h>
#include <atomic.h>

static volatile uint8_t  a = 0;
static volatile uint8_t  b = 0;
static volatile uint16_t cycles;

static absolutetime_t foo(void *payload)
{
	uint8_t *p = (uint8_t *)payload;
	*p         = *p + 1;
	if (*p < 11) // Run timer 10 times
	{
		return 10000; // Reschedule the timer after 10000 ticks
	}
	a = 1;
	return 0; // Stop the timer
}

static absolutetime_t bar()
{
	b = 1;    // Mark that timer has executed
	return 0; // Stop the timer
}

uint8_t TIMER_0_test_timeout(void)
{
	// Create payload parameter for foo
	uint8_t timer_param_a = 1;

	// Initialize the two first elements in the struct: callback_ptr & payload,
	// the remaining elements are initialized by TIMER_0_timeout_add()
	timer_struct_t foo_timer = {foo, (void *)&timer_param_a};
	timer_struct_t bar_timer = {bar, NULL};

	ENABLE_INTERRUPTS();

	// Let's test scheduler mode
	// Schedule foo() to be called in 10000 cycles
	TIMER_0_timeout_create(&foo_timer, 10000);
	// Schedule bar() to be called in 200000 cycles
	TIMER_0_timeout_create(&bar_timer, 200000);

	// Wait for both foo and bar to have executed at least once.
	while ((a == 0) && (b == 0)) // Wait for both foo and bar to have executed at least once.
	{
		// Returns immediately if no callback is ready to execute
		TIMER_0_timeout_call_next_callback();
	}

	// bar() hasn't timed out yet, let's delete it from
	// the schedule queue
	TIMER_0_timeout_delete(&bar_timer);

	// Let's test stopwatch mode, measuring the number of
	// cycles needed to execute a small loop
	TIMER_0_timeout_start_timer(&foo_timer);
	for (a = 0; a < 200; a++)
		;
	cycles = TIMER_0_timeout_stop_timer(&foo_timer);

	return 1;
}
