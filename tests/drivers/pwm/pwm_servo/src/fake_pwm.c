/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT test_fake_pwm

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include "fake_pwm.h"

struct fake_pwm_data {
	uint32_t last_pulse;   /* cycles; == ns because clk = NSEC_PER_SEC */
	int      inject_err;
};

static int fake_set_cycles(const struct device *dev, uint32_t channel,
			    uint32_t period, uint32_t pulse, pwm_flags_t flags)
{
	struct fake_pwm_data *data = dev->data;

	if (data->inject_err) {
		int err = data->inject_err;

		data->inject_err = 0;
		return err;
	}
	data->last_pulse = pulse;
	return 0;
}

static int fake_get_cycles_per_sec(const struct device *dev, uint32_t channel,
				   uint64_t *cycles)
{
	/* 1 cycle == 1 ns  →  pulse_cycles == pulse_ns in assertions */
	*cycles = NSEC_PER_SEC;
	return 0;
}

static const struct pwm_driver_api fake_pwm_api = {
	.set_cycles         = fake_set_cycles,
	.get_cycles_per_sec = fake_get_cycles_per_sec,
};

uint32_t fake_pwm_last_pulse_ns(const struct device *dev)
{
	return ((const struct fake_pwm_data *)dev->data)->last_pulse;
}

void fake_pwm_inject_err(const struct device *dev, int err)
{
	((struct fake_pwm_data *)dev->data)->inject_err = err;
}

static int fake_pwm_init(const struct device *dev)
{
	return 0;
}

#define FAKE_PWM_DEFINE(n) \
	static struct fake_pwm_data fake_pwm_data_##n; \
	DEVICE_DT_INST_DEFINE(n, fake_pwm_init, NULL, \
			      &fake_pwm_data_##n, NULL, \
			      POST_KERNEL, 60, &fake_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(FAKE_PWM_DEFINE)
