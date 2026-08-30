/*
 * Copyright (c) 2026 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _led_h_
#define _led_h_

/* Status LED (PC13): solid while the Pi is running, slow blink while it is
 * gated off. */
int led_init(void);

/* Stop the blink and drive the LED off. Must be called before sleeping — STOP
 * retains I/O state, so an LED left lit burns ~3 mA for the whole sleep. */
void led_off(void);

#endif
