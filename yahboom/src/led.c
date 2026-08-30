/*
 * Copyright (c) 2026 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <errno.h>

#include "led.h"
#include "pi.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

/* Half-period: 1 s on, 1 s off. */
#define LED_BLINK_MS 1000

/* PC13 is in the backup domain (TAMPER-RTC) — about 3 mA of drive and a slow
 * slew rate. Fine for an indicator, not for anything else. */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void led_work_fn(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(led_work, led_work_fn);

static bool blink_on;

static void led_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	if (pi_is_on()) {
		blink_on = true;
	} else {
		blink_on = !blink_on;
	}
	gpio_pin_set_dt(&led, blink_on);

	k_work_reschedule(&led_work, K_MSEC(LED_BLINK_MS));
}

int led_init(void)
{
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("GPIO port not ready");
		return -ENODEV;
	}

	int rc = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	if (rc) {
		LOG_ERR("configure failed (%d)", rc);
		return rc;
	}

	k_work_reschedule(&led_work, K_NO_WAIT);
	return 0;
}

void led_off(void)
{
	/* Cancel first, or the next tick relights it. Both this and the callers
	 * of led_off() run on the system workqueue, so the handler cannot be
	 * mid-flight here. */
	k_work_cancel_delayable(&led_work);
	blink_on = false;
	gpio_pin_set_dt(&led, 0);
}
