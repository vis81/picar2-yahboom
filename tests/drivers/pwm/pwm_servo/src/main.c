/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the pwm-servo driver.
 *
 * Board: native_sim
 *   Servo: min=1000 µs, max=2000 µs, init_value=50
 *
 * Tests cover three areas:
 *   1. Pulse-width mapping  — value→ns arithmetic at boundaries and midpoints.
 *   2. Read-back            — current_value cache correctness.
 *   3. Error handling       — invalid input and PWM failure recovery.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm_servo.h>
#include <zephyr/ztest.h>
#include "fake_pwm.h"

#define SERVO_NODE    DT_NODELABEL(servo_test)
#define FAKE_PWM_NODE DT_NODELABEL(fake_pwm0)

/* ns equivalents for the test servo's range */
#define PULSE_MIN_NS  PWM_USEC(1000)   /* 1 000 000 ns */
#define PULSE_MAX_NS  PWM_USEC(2000)   /* 2 000 000 ns */
#define PULSE_MID_NS  PWM_USEC(1500)   /* 1 500 000 ns */
#define PULSE_25_NS   PWM_USEC(1250)   /* 1 250 000 ns */
#define PULSE_75_NS   PWM_USEC(1750)   /* 1 750 000 ns */

static const struct device *servo;
static const struct device *fake_pwm;

static void *suite_setup(void)
{
	servo    = DEVICE_DT_GET(SERVO_NODE);
	fake_pwm = DEVICE_DT_GET(FAKE_PWM_NODE);

	zassert_true(device_is_ready(fake_pwm), "fake_pwm not ready");
	zassert_true(device_is_ready(servo),    "servo not ready");
	return NULL;
}

static void before_each(void *unused)
{
	/* Reset to a known state before every test. */
	fake_pwm_inject_err(fake_pwm, 0);
	servo_write(servo, 50);
}

ZTEST_SUITE(pwm_servo, NULL, suite_setup, before_each, NULL, NULL);

/* ── 1. Pulse-width mapping ──────────────────────────────────────────────── */

ZTEST(pwm_servo, test_init_sets_init_value_pulse)
{
	/*
	 * init_value=50 → MAP(50, 1000000, 2000000) = 1500000 ns.
	 * The driver calls channel_set during init, so the fake PWM should
	 * have seen this pulse before the first test runs.
	 */
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PULSE_MID_NS,
		"init pulse mismatch: got %u, want %u",
		fake_pwm_last_pulse_ns(fake_pwm), PULSE_MID_NS);
}

ZTEST(pwm_servo, test_write_min_maps_to_min_pulse)
{
	zassert_ok(servo_write(servo, 0));
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PULSE_MIN_NS,
		"value=0 pulse mismatch: got %u, want %u",
		fake_pwm_last_pulse_ns(fake_pwm), PULSE_MIN_NS);
}

ZTEST(pwm_servo, test_write_max_maps_to_max_pulse)
{
	zassert_ok(servo_write(servo, 100));
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PULSE_MAX_NS,
		"value=100 pulse mismatch: got %u, want %u",
		fake_pwm_last_pulse_ns(fake_pwm), PULSE_MAX_NS);
}

ZTEST(pwm_servo, test_write_midpoint_maps_to_mid_pulse)
{
	zassert_ok(servo_write(servo, 50));
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PULSE_MID_NS,
		"value=50 pulse mismatch: got %u, want %u",
		fake_pwm_last_pulse_ns(fake_pwm), PULSE_MID_NS);
}

ZTEST(pwm_servo, test_write_quarter_maps_correctly)
{
	zassert_ok(servo_write(servo, 25));
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PULSE_25_NS,
		"value=25 pulse mismatch: got %u, want %u",
		fake_pwm_last_pulse_ns(fake_pwm), PULSE_25_NS);
}

ZTEST(pwm_servo, test_write_three_quarter_maps_correctly)
{
	zassert_ok(servo_write(servo, 75));
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PULSE_75_NS,
		"value=75 pulse mismatch: got %u, want %u",
		fake_pwm_last_pulse_ns(fake_pwm), PULSE_75_NS);
}

/* ── 2. Read-back ────────────────────────────────────────────────────────── */

ZTEST(pwm_servo, test_read_returns_last_written_value)
{
	uint8_t val;

	zassert_ok(servo_write(servo, 73));
	zassert_ok(servo_read(servo, &val));
	zassert_equal(val, 73, "read back %u, want 73", val);
}

ZTEST(pwm_servo, test_sequential_writes_update_readback)
{
	uint8_t val;

	zassert_ok(servo_write(servo, 10));
	zassert_ok(servo_write(servo, 90));
	zassert_ok(servo_read(servo, &val));
	zassert_equal(val, 90, "read back %u after two writes, want 90", val);
}

/* ── 3. Error handling ───────────────────────────────────────────────────── */

ZTEST(pwm_servo, test_write_above_100_returns_einval)
{
	zassert_equal(servo_write(servo, 101), -EINVAL);
}

ZTEST(pwm_servo, test_write_invalid_does_not_change_readback)
{
	uint8_t val;

	zassert_ok(servo_write(servo, 30));
	zassert_equal(servo_write(servo, 101), -EINVAL);
	zassert_ok(servo_read(servo, &val));
	zassert_equal(val, 30, "invalid write corrupted readback: got %u", val);
}

ZTEST(pwm_servo, test_pwm_error_preserves_cached_value)
{
	/*
	 * Regression: channel_set used to update current_value before calling
	 * pwm_set_pulse_dt. On a PWM error the cache would reflect a position
	 * the hardware never reached.
	 *
	 * After the fix, current_value must only be updated on success.
	 */
	uint8_t val;

	zassert_ok(servo_write(servo, 30));
	fake_pwm_inject_err(fake_pwm, -EIO);
	zassert_equal(servo_write(servo, 70), -EIO);

	zassert_ok(servo_read(servo, &val));
	zassert_equal(val, 30,
		"cache corrupted on PWM error: got %u, want 30", val);
}

ZTEST(pwm_servo, test_pwm_error_does_not_change_pulse)
{
	/* Hardware state must also be unchanged after an injected error. */
	zassert_ok(servo_write(servo, 30));
	uint32_t pulse_before = fake_pwm_last_pulse_ns(fake_pwm);

	fake_pwm_inject_err(fake_pwm, -EIO);
	servo_write(servo, 70);

	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), pulse_before,
		"pulse changed despite PWM error");
}

ZTEST(pwm_servo, test_recovery_after_pwm_error)
{
	/* Driver must work normally after a transient PWM failure. */
	uint8_t val;

	fake_pwm_inject_err(fake_pwm, -EIO);
	servo_write(servo, 70);           /* fails */

	zassert_ok(servo_write(servo, 80));   /* must succeed */
	zassert_ok(servo_read(servo, &val));
	zassert_equal(val, 80, "got %u after recovery, want 80", val);
	zassert_equal(fake_pwm_last_pulse_ns(fake_pwm), PWM_USEC(1800));
}
