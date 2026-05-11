/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/input/sbusreceiver.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/rc.h>
#include <zephyr/drivers/uart.h>
#include "zephyr/drivers/motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_compat.h>
//#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include "battery.h"
#include "buzzer.h"
#include "motor.h"
#include "servo.h"

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define RC_IN DT_NODELABEL(rc)
#define MODE_MANUAL		0
#define MODE_AUTO 		1

// Simple functions for mapping a value betwen [0, 100] to the
// range [min, max] and vice versa.
#define MAP(value, min, max) ((value) * ((max) - (min))/100 + (min))
#define PAM(value, min, max) (((value) - (min)) * 100 / ((max) - (min)))

#define PAM2(x, min_src, max_src, min_dst, max_dst) \
	((min_dst) + ((float)((max_dst) - (min_dst)) / (float)((max_src) - (min_src))) * ((x) - (min_src)))

static const struct device *receiver = DEVICE_DT_GET(RC_IN);

static int rc_debug = 0;
static int rc_enable = 1;
static int rc_mode = MODE_MANUAL;

void rc_disable(void)
{
	rc_enable = 0;
}

int rc_init() {
	if (!device_is_ready(receiver)) {
		printk("RC IN device not ready.\n");
		return -EBUSY;
	}
	return 0;
}

static void callback(struct input_event *evt) {
	int ret;
	static uint32_t pulse_width = 50;
	static uint32_t throttle = 0;
	static int32_t vel = 0;
	static uint32_t dir = DIR_STOP;
	static bool brake = false;

	switch (evt->code) {
	case INPUT_ABS_RUDDER:
		pulse_width = PAM(evt->value, 0xF0, 0x720);
		break;
	case INPUT_ABS_GAS:
		int32_t c1 = evt->value - 0x3e8;
		throttle = PAM(abs(c1), 0, 0x720/2);
		vel = PAM2(c1, -0x720 / 2, 0x720 / 2, -3300, 3300);
		if (abs(c1) < 0x50)
			dir = DIR_STOP;
		else if (c1 < 0)
			dir = DIR_BACKWARD;
		else
			dir = DIR_FORWARD;
		break;
	case INPUT_ABS_BRAKE:
		brake = evt->value > 300;
		break;
	case INPUT_BTN_MODE:
		rc_mode = evt->value < 0x720/2 ? MODE_MANUAL : MODE_AUTO;
		break;
	}
	if (!evt->sync)
		return;

	if (brake)
		dir = DIR_BREAK;

	if (abs(vel) < 100)
		vel = 0;

	if (rc_debug) {
		printk("pw %u d %u thr %u vel %d brk %u mode %u\n", pulse_width, dir, throttle, vel, brake, rc_mode);
	}
	if (rc_enable) {
		ret = servo_steer(pulse_width);
		if (ret < 0) {
			printk("Error %d: failed to set pulse width\n", ret);
		}
		if (rc_mode == MODE_MANUAL || dir == DIR_BREAK) {
			motor_throttle(MOTOR_R, dir, throttle);
			motor_throttle(MOTOR_L, dir, throttle);
		} else {
			motor_speed(MOTOR_R, vel);
			motor_speed(MOTOR_L, vel);
		}
	}
};

INPUT_CALLBACK_DEFINE( DEVICE_DT_GET(RC_IN), callback);


#ifdef CONFIG_SHELL

static int cmd_rc_debug(const struct shell *sh, size_t argc,
			      char **argv)
{
	if (argc < 2) {
		shell_help(sh);
		return -EINVAL;
	}
	rc_debug = atoi(argv[1]);
	return 0;
}

static int cmd_rc_enable(const struct shell *sh, size_t argc,
			      char **argv)
{
	if (argc < 2) {
		shell_help(sh);
		return -EINVAL;
	}
	rc_enable = atoi(argv[1]);
	return 0;
}

static int cmd_rc_stats(const struct shell *sh, size_t argc,
			      char **argv)
{
	struct sbus_stats stats;
	sbus_read_stats(receiver, &stats);
	shell_print(sh, "bytes        : %d", stats.rx_bytes);
	shell_print(sh, "bytes dropped: %d", stats.rx_bytes_dropped);
	shell_print(sh, "good         : %d", stats.rx_good);
	shell_print(sh, "bad          : %d", stats.rx_bad);
	shell_print(sh, "discarded    : %d (%d%%)", stats.rx_discarded, 
				stats.rx_good ? stats.rx_discarded * 100 / stats.rx_good : 0);
	shell_print(sh, "last_ts      : %lld", stats.last_ts);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_rc,
	SHELL_CMD(debug, NULL, " 1|0\n", cmd_rc_debug),
	SHELL_CMD(enable, NULL, " 1|0\n", cmd_rc_enable),
	SHELL_CMD(stats, NULL, " stats ", cmd_rc_stats),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(rc, &sub_rc, "rc commands", NULL);
#endif
