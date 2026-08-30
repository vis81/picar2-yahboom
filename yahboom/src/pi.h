/*
 * Copyright (c) 2026 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _pi_h_
#define _pi_h_

#include <stdbool.h>
#include <stdint.h>

/* Default grace before the Pi is cut. Generous: with the motors already
 * stopped the Pi draws ~1 W, so a minute costs ~0.02 Wh against a 33 Wh pack
 * — far cheaper than cutting a filesystem mid-write. */
#define PI_SHUTDOWN_GRACE_MS 60000

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

/* Bring the Pi down, waiting up to grace_ms for RUN to drop before cutting
 * power anyway. Until a SHUTDOWN_REQ line exists nothing can ask the Pi to
 * stop, so the wait only covers a shutdown someone else started; the cut is
 * what actually ends it. Returns 0 if it went down on its own, -ETIMEDOUT if
 * it had to be cut. */
int pi_shutdown(uint32_t grace_ms);

#endif
