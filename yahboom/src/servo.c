/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>

#define SERVO0 DT_NODELABEL(servo0)
#define SERVO1 DT_NODELABEL(servo1)
#define SERVO2 DT_NODELABEL(servo2)

static const struct pwm_dt_spec servo_spec[] = {
	PWM_DT_SPEC_GET(SERVO0),
	PWM_DT_SPEC_GET(SERVO1),
	PWM_DT_SPEC_GET(SERVO2),
};

static const uint16_t servo_min_us[] = {
	DT_PROP(SERVO0, min_pulse_us),
	DT_PROP(SERVO1, min_pulse_us),
	DT_PROP(SERVO2, min_pulse_us),
};

static const uint16_t servo_max_us[] = {
	DT_PROP(SERVO0, max_pulse_us),
	DT_PROP(SERVO1, max_pulse_us),
	DT_PROP(SERVO2, max_pulse_us),
};

static const uint16_t servo_init_us[] = {
	DT_PROP(SERVO0, init_value),
	DT_PROP(SERVO1, init_value),
	DT_PROP(SERVO2, init_value),
};

#define NUM_SERVOS ARRAY_SIZE(servo_spec)

static uint16_t current_us[NUM_SERVOS];

int servo_init(void)
{
	for (int i = 0; i < NUM_SERVOS; i++) {
		if (!pwm_is_ready_dt(&servo_spec[i])) {
			printk("Error: PWM device %s not ready\n",
			       servo_spec[i].dev->name);
		}
	}
	return 0;
}

int servo_write_id(int id, uint16_t us)
{
	if (id < 0 || id >= NUM_SERVOS)
		return -EINVAL;
	uint16_t clamped = CLAMP(us, servo_min_us[id], servo_max_us[id]);
	int ret = pwm_set_pulse_dt(&servo_spec[id], PWM_USEC(clamped));
	if (!ret)
		current_us[id] = clamped;
	return ret;
}

int servo_read_id(int id, uint16_t *us)
{
	if (id < 0 || id >= NUM_SERVOS)
		return -EINVAL;
	*us = current_us[id];
	return 0;
}

void servo_neutral_all(void)
{
	for (int i = 0; i < NUM_SERVOS; i++)
		servo_write_id(i, servo_init_us[i]);
}

/* tenths of degrees → µs for servo0 */
int servo_steer(int16_t tenths_deg)
{
	int32_t half_range = (servo_max_us[0] - servo_min_us[0]) / 2;
	int32_t us = (int32_t)servo_init_us[0] +
	             (int32_t)tenths_deg * half_range / 900;
	return servo_write_id(0, (uint16_t)CLAMP(us,
	                          (int32_t)servo_min_us[0],
	                          (int32_t)servo_max_us[0]));
}

/* µs → tenths of degrees for servo0 */
int servo_get(int16_t *tenths_deg)
{
	int32_t half_range = (servo_max_us[0] - servo_min_us[0]) / 2;
	int32_t offset = (int32_t)current_us[0] - (int32_t)servo_init_us[0];
	*tenths_deg = (int16_t)(offset * 900 / half_range);
	return 0;
}

#ifdef CONFIG_SHELL

static int cmd_servo_pulse(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t id;

	if (argc < 2 || sscanf(argv[1], "%u", &id) != 1 || id >= NUM_SERVOS) {
		shell_help(sh);
		return -EINVAL;
	}
	if (argc == 2) {
		uint16_t val = 0;
		servo_read_id(id, &val);
		shell_print(sh, "%u us", val);
		return 0;
	}
	uint32_t us;
	if (sscanf(argv[2], "%u", &us) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	if (us < servo_min_us[id] || us > servo_max_us[id]) {
		shell_error(sh, "pulse for servo %u must be %u–%u µs",
			    id, servo_min_us[id], servo_max_us[id]);
		return -EINVAL;
	}
	shell_print(sh, "servo %u: %u µs", id, us);
	return servo_write_id(id, (uint16_t)us);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
	SHELL_CMD(pulse, NULL,
		  "servo pulse <id> [us]  — get/set pulse width in µs\n",
		  cmd_servo_pulse),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
#endif
