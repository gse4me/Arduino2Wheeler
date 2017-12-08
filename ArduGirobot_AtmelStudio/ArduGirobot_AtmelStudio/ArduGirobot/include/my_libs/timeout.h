/**
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
 * \file timeout.h
 *
 */

/**
 \defgroup doc_driver_timer_timeout Timeout Driver
 \ingroup doc_driver_timer

 \section doc_driver_timer_timeout_rev Revision History
 - v0.0.0.1 Initial Commit

@{
*/

#ifndef _TIMEOUTDRIVER_H
#define _TIMEOUTDRIVER_H

#include <compiler.h>
#include <stdint.h>
#include <stdbool.h>

/** Datatype used to hold the number of ticks until a timer expires */
typedef uint32_t absolutetime_t;

/** Typedef for the function pointer for the timeout callback function */
typedef absolutetime_t (*timercallback_ptr_t)(void *payload);

/** Data structure completely describing one timer */
typedef struct timer_struct_s {
	timercallback_ptr_t    callback_ptr; ///< Pointer to a callback function that is called when this timer expires
	void *                 payload; ///< Pointer to data that user would like to pass along to the callback function
	struct timer_struct_s *next;    ///< Pointer to a linked list of all timers that have expired and whose callback
	                                ///functions are due to be called
	absolutetime_t absolute_time;   ///< The number of ticks the timer will count before it expires
} timer_struct_t;

/**
 * \brief Initialize the Timeout driver
 *
 * \return Nothing
 */
void TIMER_0_timeout_init(void);

//********************************************************
// The following functions form the API for scheduler mode.
//********************************************************

/**
 * \brief Schedule the specified timer task to execute at the specified time
 *
 * \param[in] timer Pointer to struct describing the task to execute
 * \param[in] timeout Number of ticks to wait before executing the task
 *
 * \return Nothing
 */
void TIMER_0_timeout_create(timer_struct_t *timer, absolutetime_t timeout);

/**
 * \brief Delete the specified timer task so it won't be executed
 *
 * \param[in] timer Pointer to struct describing the task to execute
 *
 * \return Nothing
 */
void TIMER_0_timeout_delete(timer_struct_t *timer);

/**
 * \brief Delete all scheduled timer tasks
 *
 * \return Nothing
 */
void TIMER_0_timeout_flush_all(void);

/**
 * \brief Execute the next timer task that has been scheduled for execution.
 *
 * If no task has been scheduled for execution, the function
 * returns immediately, so there is no need for any polling.
 *
 * \return Nothing
 */
void TIMER_0_timeout_call_next_callback(void);

//********************************************************
// The following functions form the API for stopwatch mode.
//********************************************************

/**
 * \brief Start a stopwatch using the specified timer struct
 *
 * Start a timer with (maximum range)/2. You cannot time more than
 * this and the timer will stop after this time elapses.
 *
 * \param[in] timer Struct holding the stopwatch
 *
 * \return Nothing
 */
void TIMER_0_timeout_start_timer(timer_struct_t *timer);

/**
 * \brief Stop the specified stopwatch and return the elapsed number of ticks.
 *
 * \param[in] timer Struct holding the stopwatch
 *
 * \return The number of ticks passed since starting the stopwatch
 */
absolutetime_t TIMER_0_timeout_stop_timer(timer_struct_t *timer);

void TIMER_0_print_list(void);

#endif

/** @}*/
