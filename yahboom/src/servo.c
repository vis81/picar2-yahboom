/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>
#include <zephyr/settings/settings.h>
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

/* Runtime-adjustable center per servo — persisted via settings "servo/center_N" */
static uint16_t center_us[NUM_SERVOS];

static int servo_settings_set(const char *name, size_t len,
			       settings_read_cb read_cb, void *cb_arg)
{
	uint32_t id;
	uint16_t val;

	if (sscanf(name, "center_%u", &id) == 1 &&
	    id < NUM_SERVOS && len == sizeof(val)) {
		read_cb(cb_arg, &val, sizeof(val));
		center_us[id] = val;
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(servo, "servo", NULL, servo_settings_set, NULL, NULL);

int servo_init(void)
{
	for (int i = 0; i < NUM_SERVOS; i++) {
		if (!pwm_is_ready_dt(&servo_spec[i])) {
			printk("Error: PWM device %s not ready\n",
			       servo_spec[i].dev->name);
		}
		center_us[i] = servo_init_us[i];
	}
	settings_load_subtree("servo");
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
		servo_write_id(i, center_us[i]);
}

/* tenths of degrees → µs for servo0 */
int servo_steer(int16_t tenths_deg)
{
	int32_t half_range = (servo_max_us[0] - servo_min_us[0]) / 2;
	int32_t us = (int32_t)center_us[0] +
	             (int32_t)tenths_deg * half_range / 900;
	return servo_write_id(0, (uint16_t)CLAMP(us,
	                          (int32_t)servo_min_us[0],
	                          (int32_t)servo_max_us[0]));
}

/* µs → tenths of degrees for servo0 */
int servo_get(int16_t *tenths_deg)
{
	int32_t half_range = (servo_max_us[0] - servo_min_us[0]) / 2;
	int32_t offset = (int32_t)current_us[0] - (int32_t)center_us[0];
	*tenths_deg = (int16_t)(offset * 900 / half_range);
	return 0;
}

int servo_set_center(int id, uint16_t us)
{
	if (id < 0 || id >= NUM_SERVOS)
		return -EINVAL;
	if (us < servo_min_us[id] || us > servo_max_us[id])
		return -EINVAL;
	center_us[id] = us;
	char key[32];
	snprintf(key, sizeof(key), "servo/center_%d", id);
	settings_save_one(key, &center_us[id], sizeof(center_us[id]));
	return servo_write_id(id, us);
}

int servo_get_center(int id, uint16_t *us)
{
	if (id < 0 || id >= NUM_SERVOS)
		return -EINVAL;
	*us = center_us[id];
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

static int cmd_servo_center(const struct shell *sh, size_t argc, char **argv)
{
	/* servo center               — show all */
	if (argc == 1) {
		for (int i = 0; i < NUM_SERVOS; i++) {
			shell_print(sh, "servo %d: center %u us  (default %u us)",
				    i, center_us[i], servo_init_us[i]);
		}
		return 0;
	}

	uint32_t id;
	if (sscanf(argv[1], "%u", &id) != 1 || id >= NUM_SERVOS) {
		shell_error(sh, "servo id must be 0–%d", NUM_SERVOS - 1);
		return -EINVAL;
	}

	/* servo center <id>          — show one */
	if (argc == 2) {
		shell_print(sh, "servo %u: center %u us  (default %u us)",
			    id, center_us[id], servo_init_us[id]);
		return 0;
	}

	/* servo center <id> reset    — restore default */
	if (strcmp(argv[2], "reset") == 0) {
		char key[32];
		snprintf(key, sizeof(key), "servo/center_%u", id);
		settings_delete(key);
		center_us[id] = servo_init_us[id];
		servo_write_id(id, center_us[id]);
		shell_print(sh, "servo %u: center reset to %u us", id, center_us[id]);
		return 0;
	}

	/* servo center <id> <us>     — set and save */
	uint32_t us;
	if (sscanf(argv[2], "%u", &us) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	int ret = servo_set_center(id, (uint16_t)us);
	if (ret) {
		shell_error(sh, "servo %u: center must be %u–%u µs",
			    id, servo_min_us[id], servo_max_us[id]);
		return ret;
	}
	shell_print(sh, "servo %u: center set to %u us (saved)", id, us);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
	SHELL_CMD(pulse, NULL,
		  "servo pulse <id> [us]  — get/set pulse width in µs",
		  cmd_servo_pulse),
	SHELL_CMD(center, NULL,
		  "servo center [<id> [<us>|reset]]  — get/set/reset center pulse per servo",
		  cmd_servo_center),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
#endif
