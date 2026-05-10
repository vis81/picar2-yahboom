/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the STM32 software PWM driver (st,stm32-pwm-sw).
 *
 * Board: yahboom_ros_stm32f103
 *   Timer:   TIM2, prescaler = 1000  →  ~71 928 ticks/s
 *   Channel: 1
 *   GPIO:    PC13 (ACTIVE_HIGH)
 *
 * The tests verify three categories:
 *   1. Parameter validation — the driver rejects bad arguments.
 *   2. Timing arithmetic  — get_cycles_per_sec, boundary values.
 *   3. GPIO state         — zero/full duty and the ISR ordering fix
 *                           (commit 12efcde: "fix GPIO state when ISR
 *                           is delayed past pulse end").
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <stdint.h>

/* PWM controller and channel derived from the test overlay alias. */
#define PWM_NODE  DT_PWMS_CTLR(DT_ALIAS(pwm_sw_test))
#define PWM_CHAN  DT_PWMS_CHANNEL(DT_ALIAS(pwm_sw_test))

/*
 * GPIO spec for the single sw-pwm output declared in the overlay.
 * Used to read back the physical pin state independently of the driver.
 */
static const struct gpio_dt_spec sw_gpio =
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(pwm_sw_t2), gpios, 0);

static const struct device *pwm_dev;

/* ── helpers ─────────────────────────────────────────────────────────────── */

static int gpio_read(void)
{
	return gpio_pin_get_raw(sw_gpio.port, sw_gpio.pin);
}

/* ── suite lifecycle ─────────────────────────────────────────────────────── */

static void *suite_setup(void)
{
	pwm_dev = DEVICE_DT_GET(PWM_NODE);
	zassert_true(device_is_ready(pwm_dev),      "PWM device not ready");
	zassert_true(gpio_is_ready_dt(&sw_gpio),    "GPIO device not ready");
	return NULL;
}

/* Leave the GPIO inactive between tests so each one starts clean. */
static void after_each(void *unused)
{
	pwm_set_cycles(pwm_dev, PWM_CHAN, 1000, 0, 0);
	k_sleep(K_MSEC(2));
}

ZTEST_SUITE(pwm_stm32_sw, NULL, suite_setup, NULL, after_each, NULL);

/* ── 1. Parameter validation ─────────────────────────────────────────────── */

ZTEST(pwm_stm32_sw, test_channel_zero_rejected)
{
	/* Channel numbers are 1-based; 0 is always invalid. */
	zassert_equal(-EINVAL, pwm_set_cycles(pwm_dev, 0, 1000, 500, 0));
}

ZTEST(pwm_stm32_sw, test_channel_above_max_rejected)
{
	/* STM32F1 timers have at most 4 channels. */
	zassert_equal(-EINVAL, pwm_set_cycles(pwm_dev, 5, 1000, 500, 0));
}

ZTEST(pwm_stm32_sw, test_channel_not_in_dts_rejected)
{
	/*
	 * The overlay maps only channel 1 (channels = <1>).
	 * Channel 2 is a valid TIM2 channel but has no associated GPIO,
	 * so the driver cannot produce output and must refuse.
	 */
	zassert_equal(-EINVAL, pwm_set_cycles(pwm_dev, 2, 1000, 500, 0));
}

ZTEST(pwm_stm32_sw, test_period_overflow_16bit_rejected)
{
	/*
	 * All STM32F1 timers are 16-bit.  The driver rejects any period
	 * greater than UINT16_MAX + 1 with -ENOTSUP.
	 *
	 * UINT16_MAX + 1 = 65536 (boundary, accepted)
	 * UINT16_MAX + 2 = 65537 (over limit, rejected)
	 */
	zassert_equal(-ENOTSUP,
		pwm_set_cycles(pwm_dev, PWM_CHAN, UINT16_MAX + 2u, 1000, 0),
		"period > UINT16_MAX+1 must be rejected on 16-bit timer");
}

ZTEST(pwm_stm32_sw, test_period_boundary_16bit_accepted)
{
	/* UINT16_MAX + 1 = 65536 is exactly at the limit and must succeed. */
	zassert_ok(pwm_set_cycles(pwm_dev, PWM_CHAN, UINT16_MAX + 1u, 1000, 0),
		"period == UINT16_MAX+1 must be accepted");
}

/* ── 2. Timing arithmetic ────────────────────────────────────────────────── */

ZTEST(pwm_stm32_sw, test_get_cycles_per_sec)
{
	/*
	 * TIM2 on STM32F103 at 72 MHz (APB1 timer clock), prescaler 1000:
	 *   cycles/s = 72_000_000 / (1000 + 1) ≈ 71 928 Hz
	 *
	 * Accept ±5 kHz to tolerate minor clock-tree variations.
	 */
	uint64_t cycles;

	zassert_ok(pwm_get_cycles_per_sec(pwm_dev, PWM_CHAN, &cycles));
	zassert_true(cycles >= 66000ULL && cycles <= 78000ULL,
		"unexpected cycles/sec: %llu (expected ~71928)", cycles);
}

/* ── 3. GPIO state ───────────────────────────────────────────────────────── */

ZTEST(pwm_stm32_sw, test_zero_duty_sets_gpio_low)
{
	/*
	 * pulse_cycles = 0: the driver immediately drives GPIO LOW and does
	 * not arm the CC interrupt.  No timer period needs to elapse.
	 */
	zassert_ok(pwm_set_cycles(pwm_dev, PWM_CHAN, 1000, 0, 0));
	k_sleep(K_MSEC(2));

	zassert_equal(0, gpio_read(),
		"GPIO must be LOW for zero duty cycle");
}

ZTEST(pwm_stm32_sw, test_full_duty_holds_gpio_high)
{
	/*
	 * pulse == period in UP-counter mode:
	 *   ARR  = period - 1 = 999
	 *   CCR1 = pulse      = 1000   (> ARR → CC event never fires)
	 *
	 * The driver detects pulse_cycles > period_cycles as 100 % duty,
	 * immediately asserts GPIO HIGH, and does NOT arm CC.  UPDATE fires
	 * every period and re-asserts HIGH (pulse_cycles != 0 guard in ISR).
	 *
	 * Period ≈ 1000 / 71928 Hz ≈ 13.9 ms.  Waiting 35 ms guarantees
	 * at least two UPDATE events have fired.
	 */
	zassert_ok(pwm_set_cycles(pwm_dev, PWM_CHAN, 1000, 1000, 0));
	k_sleep(K_MSEC(35));

	zassert_equal(1, gpio_read(),
		"GPIO must be HIGH for 100%% duty cycle");
}

ZTEST(pwm_stm32_sw, test_isr_ordering_gpio_returns_low)
{
	/*
	 * Regression test for the ISR ordering fix (commit 12efcde):
	 * "fix GPIO state when ISR is delayed past pulse end".
	 *
	 * The ISR snapshots SR once then processes UPDATE first (GPIO HIGH)
	 * followed by CC (GPIO LOW).  When a delayed ISR sees both UIF and
	 * CC1IF set simultaneously, the CC handler runs last and GPIO ends
	 * up LOW.  Before the fix the UPDATE handler could run last, leaving
	 * GPIO stuck HIGH.
	 *
	 * Setup: period = 1000, pulse = 1  (0.1 % duty)
	 *   HIGH phase ≈  1 / 71928 Hz ≈  13.9 µs
	 *   LOW  phase ≈ 999 / 71928 Hz ≈ 13.9 ms
	 *
	 * In steady state the GPIO is LOW for 99.9 % of every period.
	 * Polling every millisecond, we must observe LOW within 100 ms.
	 * If GPIO were stuck HIGH (ordering bug) we would never find LOW.
	 */
	zassert_ok(pwm_set_cycles(pwm_dev, PWM_CHAN, 1000, 1, 0));

	/* Let the PWM stabilise for a few periods before sampling. */
	k_sleep(K_MSEC(50));

	bool found_low = false;
	int64_t deadline = k_uptime_get() + 100;

	while (k_uptime_get() < deadline) {
		if (gpio_read() == 0) {
			found_low = true;
			break;
		}
		k_sleep(K_MSEC(1));
	}

	zassert_true(found_low,
		"GPIO never returned LOW: possible ISR ordering regression");
}
