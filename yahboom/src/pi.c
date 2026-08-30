/*
 * Copyright (c) 2026 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <errno.h>
#include <stdlib.h>

#include "pi.h"

LOG_MODULE_REGISTER(pi, LOG_LEVEL_INF);

/* GLOBAL_EN must be driven low before anything else runs, so the Pi does not
 * boot just because the battery was plugged in. Must sort after the GPIO
 * driver itself; the window between STM32 reset and this hook (tens of ms) is
 * the one time the Pi can start on its own, which is early enough in its boot
 * to be harmless. */
#define PI_INIT_PRIO 99
BUILD_ASSERT(PI_INIT_PRIO > CONFIG_GPIO_INIT_PRIORITY,
	     "pi_early_hold must run after the GPIO driver");

/* Assert time before releasing GLOBAL_EN. Long enough for the PMIC to register
 * the low; a few ms would do. */
#define PI_ASSERT_MS     100
/* The 3.3 V rail comes up well inside this; not a boot-complete wait. */
#define PI_RAIL_WAIT_MS  3000
#define PI_POLL_MS       50

static const struct gpio_dt_spec global_en =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pi_global_en_gpios);
static const struct gpio_dt_spec run =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pi_run_gpios);

/* True while we hold GLOBAL_EN low. The pin itself cannot be read back for
 * this — released and Hi-Z it follows the Pi's pull-up, which is dead when the
 * Pi is unpowered. */
static bool global_en_held;

/* Reset flags are sticky, so they are read and cleared exactly once, here.
 * Cached because clearing them destroys them for every later reader. */
static uint32_t reset_cause;

static int pi_early_hold(void)
{
	int rc;

	if (!gpio_is_ready_dt(&global_en) || !gpio_is_ready_dt(&run)) {
		return -ENODEV;
	}

	/* No pull: an internal pull-up would source current into the Pi's dead
	 * 3.3 V rail whenever it is powered off. Configured before GLOBAL_EN
	 * because the decision below depends on reading it. */
	rc = gpio_pin_configure_dt(&run, GPIO_INPUT);
	if (rc) {
		return rc;
	}

	if (hwinfo_get_reset_cause(&reset_cause) != 0) {
		reset_cause = 0;
	}
	hwinfo_clear_reset_cause();

	/* A power-on — or a standby wake, which is equivalent here — means the
	 * Pi is at most tens of ms into its own boot, so gating it costs
	 * nothing. Any other reset (sys reboot, NRST, watchdog, a firmware
	 * flash) can happen with the Pi long since up, and cutting it there
	 * would corrupt its filesystem. An unknown cause is treated as the
	 * risky case: leave a running Pi alone. */
	bool cold = (reset_cause & (RESET_POR | RESET_LOW_POWER_WAKE)) != 0;
	bool hold = cold || (gpio_pin_get_dt(&run) <= 0);

	/* Open drain throughout. Never driven high — while the Pi is off its
	 * rails are down and 3.3 V pushed in back-feeds the board. No logging
	 * here; the kernel is not up yet. */
	rc = gpio_pin_configure_dt(&global_en,
				   (hold ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE)
				   | GPIO_OPEN_DRAIN);
	if (rc == 0) {
		global_en_held = hold;
	}
	return rc;
}
SYS_INIT(pi_early_hold, PRE_KERNEL_1, PI_INIT_PRIO);

int pi_init(void)
{
	if (!gpio_is_ready_dt(&run) || !gpio_is_ready_dt(&global_en)) {
		LOG_ERR("GPIO port not ready — Pi power control inactive");
		return -ENODEV;
	}

	LOG_INF("pi power control — Pi %s, GLOBAL_EN %s, reset cause 0x%08x",
		pi_is_on() ? "on" : "off",
		global_en_held ? "held" : "released", reset_cause);
	return 0;
}

uint32_t pi_reset_cause(void)
{
	return reset_cause;
}

bool pi_is_on(void)
{
	return gpio_pin_get_dt(&run) > 0;
}

bool pi_is_held(void)
{
	return global_en_held;
}

int pi_power_on(uint32_t assert_ms)
{
	if (pi_is_on()) {
		return -EALREADY;
	}

	/* Assert then release unconditionally, rather than only releasing. The
	 * Pi may be off for two different reasons: held low by us, or halted
	 * with POWER_OFF_ON_HALT=1, which latches the PMIC off while GLOBAL_EN
	 * sits high. Only a fresh rising edge starts it in the second case. */
	gpio_pin_set_dt(&global_en, 1);
	global_en_held = true;
	k_msleep(assert_ms);
	gpio_pin_set_dt(&global_en, 0);
	global_en_held = false;

	for (int waited = 0; waited < PI_RAIL_WAIT_MS; waited += PI_POLL_MS) {
		if (pi_is_on()) {
			return 0;
		}
		k_msleep(PI_POLL_MS);
	}
	return -ETIMEDOUT;
}

int pi_shutdown(uint32_t grace_ms)
{
	if (!pi_is_on()) {
		return 0;
	}

	/* No SHUTDOWN_REQ line is fitted yet, so nothing here can ask the Pi to
	 * stop — the window only lets a shutdown started by someone else
	 * finish, and gives an operator who sees the warning time to run
	 * poweroff. Once the wire exists, assert it here and this becomes a
	 * genuine graceful shutdown with the cut as the fallback. */
	LOG_WRN("waiting up to %u ms for the Pi to go down", grace_ms);

	for (uint32_t waited = 0; waited < grace_ms; waited += PI_POLL_MS) {
		if (!pi_is_on()) {
			LOG_INF("Pi down after %u ms", waited);
			return 0;
		}
		k_msleep(PI_POLL_MS);
	}

	LOG_ERR("Pi still up after %u ms — cutting power, filesystem not synced",
		grace_ms);
	pi_power_off();
	return -ETIMEDOUT;
}

int pi_power_off(void)
{
	gpio_pin_set_dt(&global_en, 1);
	global_en_held = true;

	for (int waited = 0; waited < PI_RAIL_WAIT_MS; waited += PI_POLL_MS) {
		if (!pi_is_on()) {
			return 0;
		}
		k_msleep(PI_POLL_MS);
	}
	return -ETIMEDOUT;
}

#ifdef CONFIG_SHELL
static int cmd_pi_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int level = gpio_pin_get_dt(&run);

	if (level < 0) {
		shell_error(sh, "RUN read failed (%d)", level);
		return level;
	}

	shell_print(sh, "pi        %s (RUN=%d)", level ? "on" : "off", level);
	shell_print(sh, "GLOBAL_EN %s", global_en_held ? "held low" : "released");
	shell_print(sh, "reset     0x%08x%s%s%s%s%s", reset_cause,
		    (reset_cause & RESET_POR)             ? " por"    : "",
		    (reset_cause & RESET_PIN)             ? " pin"    : "",
		    (reset_cause & RESET_SOFTWARE)        ? " soft"   : "",
		    (reset_cause & RESET_WATCHDOG)        ? " wdog"   : "",
		    (reset_cause & RESET_LOW_POWER_WAKE)  ? " lpwake" : "");
	return 0;
}

static int cmd_pi_on(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t assert_ms = PI_ASSERT_MS;

	if (argc > 1) {
		assert_ms = strtoul(argv[1], NULL, 10);
		if (assert_ms == 0 || assert_ms > 5000) {
			shell_error(sh, "assert_ms out of range (1..5000)");
			return -EINVAL;
		}
	}

	if (pi_is_on()) {
		shell_print(sh, "pi already on");
		return 0;
	}

	shell_print(sh, "GLOBAL_EN low %u ms, then release", assert_ms);

	int rc = pi_power_on(assert_ms);

	if (rc == -ETIMEDOUT) {
		shell_warn(sh, "RUN still low after %d ms — check GLOBAL_EN wiring",
			   PI_RAIL_WAIT_MS);
		return rc;
	}
	shell_print(sh, "pi on");
	return rc;
}

static int cmd_pi_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (pi_is_on()) {
		shell_warn(sh, "cutting power to a running Pi — filesystem is not synced");
	}

	int rc = pi_power_off();

	if (rc == -ETIMEDOUT) {
		shell_error(sh, "RUN still high after %d ms — GLOBAL_EN not taking effect",
			    PI_RAIL_WAIT_MS);
		return rc;
	}
	shell_print(sh, "pi off, GLOBAL_EN held low");
	return rc;
}

static int cmd_pi_shutdown(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t grace_s = PI_SHUTDOWN_GRACE_MS / 1000;

	if (argc > 1) {
		grace_s = strtoul(argv[1], NULL, 10);
		if (grace_s == 0 || grace_s > 300) {
			shell_error(sh, "grace_s out of range (1..300)");
			return -EINVAL;
		}
	}

	if (!pi_is_on()) {
		shell_print(sh, "pi already off");
		return 0;
	}

	shell_print(sh, "waiting up to %u s for the Pi to go down, then cutting",
		    grace_s);

	int rc = pi_shutdown(grace_s * 1000);

	if (rc == -ETIMEDOUT) {
		shell_warn(sh, "grace expired — power cut, filesystem not synced");
		return rc;
	}
	shell_print(sh, "pi down");
	return rc;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pi,
	SHELL_CMD(status, NULL, "print Pi power state (RUN) and GLOBAL_EN hold", cmd_pi_status),
	SHELL_CMD_ARG(on, NULL, "release GLOBAL_EN to power on [assert_ms]",
		      cmd_pi_on, 1, 1),
	SHELL_CMD(off, NULL, "hold GLOBAL_EN low — hard cut, NOT a clean shutdown",
		  cmd_pi_off),
	SHELL_CMD_ARG(shutdown, NULL, "wait for the Pi to go down, then cut [grace_s]",
		      cmd_pi_shutdown, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pi, &sub_pi, "Raspberry Pi power control", NULL);
#endif
