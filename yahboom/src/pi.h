/*
 * Copyright (c) 2026 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _pi_h_
#define _pi_h_

#include <stdbool.h>
#include <stdint.h>

/* Configure the RUN sense line. GLOBAL_EN is claimed earlier, by a
 * PRE_KERNEL_1 hook that holds it low so the Pi does not boot on power-up. */
int pi_init(void);

/* True while the Pi's 3.3 V rail is up, read from the RUN pin. */
bool pi_is_on(void);

/* True while GLOBAL_EN is held low (Pi gated off). */
bool pi_is_held(void);

/* hwinfo reset flags for this boot, latched before they were cleared. */
uint32_t pi_reset_cause(void);

/* Pulse GLOBAL_EN low for assert_ms, release it, and wait for RUN to rise.
 * The assert is needed even when already held: a Pi halted with
 * POWER_OFF_ON_HALT=1 latches its PMIC off and only restarts on a rising edge.
 * Returns -EALREADY if already powered, -ETIMEDOUT if RUN never rises. */
int pi_power_on(uint32_t assert_ms);

/* Assert GLOBAL_EN low and hold. Hard cut — no filesystem sync. Use the
 * SHUTDOWN_REQ path for a graceful shutdown, then call this to latch it off.
 * Returns -ETIMEDOUT if RUN never drops. */
int pi_power_off(void);

#endif
