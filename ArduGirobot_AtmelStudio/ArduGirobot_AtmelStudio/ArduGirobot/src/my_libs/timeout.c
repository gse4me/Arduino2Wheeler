/**
 * \file
 *
 * \brief Timeout driver.
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
#include "timeout.h"
#include "atomic.h"

absolutetime_t TIMER_0_dummy_handler(void *payload)
{
	return 0;
};

void TIMER_0_start_timer_at_head(void);
void TIMER_0_enqueue_callback(timer_struct_t *timer);
void TIMER_0_set_timer_duration(absolutetime_t duration);
absolutetime_t TIMER_0_make_absolute(absolutetime_t timeout);
absolutetime_t TIMER_0_rebase_list(void);

timer_struct_t *TIMER_0_list_head          = NULL;
timer_struct_t *TIMER_0_execute_queue_head = NULL;

timer_struct_t          TIMER_0_dummy                         = {TIMER_0_dummy_handler};
volatile absolutetime_t TIMER_0_absolute_time_of_last_timeout = 0;
volatile absolutetime_t TIMER_0_last_timer_load               = 0;
volatile bool           TIMER_0_is_running                    = false;

void TIMER_0_timeout_init(void)
{

	/* Enable TC1 */
	PRR &= ~(1 << PRTIM1);

	// TCCR1A = (0 << COM1A1) | (0 << COM1A0) /* Normal port operation, OCA disconnected */
	//		 | (0 << COM1B1) | (0 << COM1B0) /* Normal port operation, OCB disconnected */
	//		 | (0 << WGM11) | (0 << WGM10); /* Timeout */

	TCCR1B = (0 << WGM13) | (0 << WGM12)                /* Timeout */
	         | 0 << ICNC1                               /* Input Capture Noise Canceler: disabled */
	         | 0 << ICES1                               /* Input Capture Edge Select: disabled */
	         | (0 << CS12) | (1 << CS11) | (0 << CS10); /* IO clock divided by 8 */

	TIMSK1 = 0 << OCIE1B   /* Output Compare B Match Interrupt Enable: disabled */
	         | 0 << OCIE1A /* Output Compare A Match Interrupt Enable: disabled */
	         | 0 << ICIE1  /* Input Capture Interrupt Enable: disabled */
	         | 1 << TOIE1; /* Overflow Interrupt Enable: enabled */
}

void TIMER_0_stop_timeouts(void)
{
	TIMSK1 &= ~(1 << TOIE1);

	TIMER_0_absolute_time_of_last_timeout = 0;
	TIMER_0_is_running                    = 0;
}

inline void TIMER_0_set_timer_duration(absolutetime_t duration)
{
	TIMER_0_last_timer_load = 65535 - duration;
	TCNT1                   = TIMER_0_last_timer_load;
}

inline absolutetime_t TIMER_0_make_absolute(absolutetime_t timeout)
{
	timeout += TIMER_0_absolute_time_of_last_timeout;
	timeout += TIMER_0_is_running ? TCNT1 - TIMER_0_last_timer_load : 0;

	return timeout;
}

inline absolutetime_t TIMER_0_rebase_list(void)
{
	timer_struct_t *base_point = TIMER_0_list_head;
	absolutetime_t  base       = TIMER_0_list_head->absolute_time;

	while (base_point != NULL) {
		base_point->absolute_time -= base;
		base_point = base_point->next;
	}

	TIMER_0_absolute_time_of_last_timeout -= base;
	return base;
}

inline void TIMER_0_print_list(void)
{
	timer_struct_t *base_point = TIMER_0_list_head;
	while (base_point != NULL) {
		printf("%ld -> ", (uint32_t)base_point->absolute_time);
		base_point = base_point->next;
	}
	printf("NULL\n");
}

// Returns true if the insert was at the head, false if not
bool TIMER_0_sorted_insert(timer_struct_t *timer)
{
	absolutetime_t  timer_absolute_time = timer->absolute_time;
	uint8_t         at_head             = 1;
	timer_struct_t *insert_point        = TIMER_0_list_head;
	timer_struct_t *prev_point          = NULL;
	timer->next                         = NULL;

	if (timer_absolute_time < TIMER_0_absolute_time_of_last_timeout) {
		timer_absolute_time += 65535 - TIMER_0_rebase_list() + 1;
		timer->absolute_time = timer_absolute_time;
	}

	while (insert_point != NULL) {
		if (insert_point->absolute_time > timer_absolute_time) {
			break; // found the spot
		}
		prev_point   = insert_point;
		insert_point = insert_point->next;
		at_head      = 0;
	}

	if (at_head == 1) // the front of the list.
	{
		TIMER_0_set_timer_duration(65535);
		TIFR1 |= (1 << TOV1);

		timer->next       = (TIMER_0_list_head == &TIMER_0_dummy) ? TIMER_0_dummy.next : TIMER_0_list_head;
		TIMER_0_list_head = timer;
		return true;
	} else // middle of the list
	{
		timer->next = prev_point->next;
	}
	prev_point->next = timer;
	return false;
}

void TIMER_0_start_timer_at_head(void)
{
	TIMSK1 &= ~(1 << TOIE1);

	if (TIMER_0_list_head == NULL) // no timeouts left
	{
		TIMER_0_stop_timeouts();
		return;
	}

	absolutetime_t period = ((TIMER_0_list_head != NULL) ? (TIMER_0_list_head->absolute_time) : 0)
	                        - TIMER_0_absolute_time_of_last_timeout;

	// Timer is too far, insert dummy and schedule timer after the dummy
	if (period > 65535) {
		TIMER_0_dummy.absolute_time = TIMER_0_absolute_time_of_last_timeout + 65535;
		TIMER_0_dummy.next          = TIMER_0_list_head;
		TIMER_0_list_head           = &TIMER_0_dummy;
		period                      = 65535;
	}

	TIMER_0_set_timer_duration(period);

	TIMSK1 |= (1 << TOIE1);

	TIMER_0_is_running = 1;
}

void TIMER_0_timeout_flush_all(void)
{
	TIMER_0_stop_timeouts();
	TIMER_0_list_head = NULL;
}

void TIMER_0_timeout_delete(timer_struct_t *timer)
{
	if (TIMER_0_list_head == NULL)
		return;

	// Guard in case we get interrupted, we cannot safely compare/search and get interrupted
	TIMSK1 &= ~(1 << TOIE1);

	// Special case, the head is the one we are deleting
	if (timer == TIMER_0_list_head) {
		TIMER_0_list_head = TIMER_0_list_head->next; // Delete the head
		TIMER_0_start_timer_at_head();               // Start the new timer at the head
	} else {                                         // More than one timer in the list, search the list.
		timer_struct_t *find_timer = TIMER_0_list_head;
		timer_struct_t *prev_timer = NULL;
		while (find_timer != NULL) {
			if (find_timer == timer) {
				prev_timer->next = find_timer->next;
				break;
			}
			prev_timer = find_timer;
			find_timer = find_timer->next;
		}
		TIMSK1 |= (1 << TOIE1);
	}
}

inline void TIMER_0_enqueue_callback(timer_struct_t *timer)
{
	timer_struct_t *tmp;
	timer->next = NULL;

	// Special case for empty list
	if (TIMER_0_execute_queue_head == NULL) {
		TIMER_0_execute_queue_head = timer;
		return;
	}

	// Find the end of the list and insert the next expired timer at the back of the queue
	tmp = TIMER_0_execute_queue_head;
	while (tmp->next != NULL)
		tmp = tmp->next;

	tmp->next = timer;
}

void TIMER_0_timeout_call_next_callback(void)
{

	if (TIMER_0_execute_queue_head == NULL)
		return;

	// Critical section needed if TIMER_0_timeout_call_next_callback()
	// was called from polling loop, and not called from ISR.
	ENTER_CRITICAL(T);
	timer_struct_t *callback_timer = TIMER_0_execute_queue_head;

	// Done, remove from list
	TIMER_0_execute_queue_head = TIMER_0_execute_queue_head->next;

	EXIT_CRITICAL(T); // End critical section

	absolutetime_t reschedule = callback_timer->callback_ptr(callback_timer->payload);

	// Do we have to reschedule it? If yes then add delta to absolute for reschedule
	if (reschedule) {
		TIMER_0_timeout_create(callback_timer, reschedule);
	}
}

void TIMER_0_timeout_create(timer_struct_t *timer, absolutetime_t timeout)
{
	TIMSK1 &= ~(1 << TOIE1);

	timer->absolute_time = TIMER_0_make_absolute(timeout);

	// We only have to start the timer at head if the insert was at the head
	if (TIMER_0_sorted_insert(timer)) {
		TIMER_0_start_timer_at_head();
	} else {
		if (TIMER_0_is_running)
			TIMSK1 |= (1 << TOIE1);
	}
}

// NOTE: assumes the callback completes before the next timer tick
ISR(TIMER1_OVF_vect)
{
	timer_struct_t *next                  = TIMER_0_list_head->next;
	TIMER_0_absolute_time_of_last_timeout = TIMER_0_list_head->absolute_time;
	TIMER_0_last_timer_load               = 0;

	if (TIMER_0_list_head != &TIMER_0_dummy)
		TIMER_0_enqueue_callback(TIMER_0_list_head);

	// Remove expired timer for the list now (it is always the one at the head)
	TIMER_0_list_head = next;

	TIMER_0_start_timer_at_head();
}

// These methods are for calculating the elapsed time in stopwatch mode.
// TIMER_0_timeout_start_timer will start a
// timer with (maximum range)/2. You cannot time more than
// this and the timer will stop after this time elapses
void TIMER_0_timeout_start_timer(timer_struct_t *timer)
{
	absolutetime_t i = -1;
	TIMER_0_timeout_create(timer, i >> 1);
}

// This funciton stops the "stopwatch" and returns the elapsed time.
absolutetime_t TIMER_0_timeout_stop_timer(timer_struct_t *timer)
{
	absolutetime_t now = TIMER_0_make_absolute(0); // Do this as fast as possible for accuracy
	absolutetime_t i   = -1;
	i >>= 1;

	TIMER_0_timeout_delete(timer);

	absolutetime_t diff = timer->absolute_time - now;

	// This calculates the (max range)/2 minus (remaining time) which = elapsed time
	return (i - diff);
}
