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
#include "zephyr/drivers/pwm_servo.h"
#include "zephyr/drivers/motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/input/input.h>
#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define RC_IN DT_NODELABEL(rc)
#define SERVO DT_NODELABEL(servo0)
#define MOTOR_R DT_NODELABEL(motor_r)
#define MOTOR_L DT_NODELABEL(motor_l)

// Simple functions for mapping a value betwen [0, 100] to the
// range [min, max] and vice versa.
#define MAP(value, min, max) ((value) * ((max) - (min))/100 + (min))
#define PAM(value, min, max) (((value) - (min)) * 100 / ((max) - (min)))

static const struct device *receiver = DEVICE_DT_GET(RC_IN);
static const struct device *servo = DEVICE_DT_GET(SERVO);
static const struct device *motor_r = DEVICE_DT_GET(MOTOR_R);
static const struct device *motor_l = DEVICE_DT_GET(MOTOR_L);

static int rc_debug = 0;
static int rc_enable = 1;


void process_events() {
	uint64_t ts;
	uint16_t chan_val[16];
	uint8_t flags;
	uint32_t pulse_width = 0;

	int ret = rc_read_all(receiver, 16, chan_val, &flags, &ts);
	if (ret)
		return;

	pulse_width = PAM(chan_val[0],0xF0 , 0x720);	
	int32_t c1 = chan_val[1] - 0x3e8;
	uint32_t throttle = PAM(abs(c1), 0, 0x720/2);
	uint32_t dir;
	if (flags & SBUS_FLAGS_FRAME_LOST || chan_val[2] > 300)
		dir = DIR_BREAK;
	else if (abs(c1) < 0x50)
		dir = DIR_STOP;
	else if(chan_val[2] > 300)
		dir = DIR_BREAK;
	else if (c1 < 0)
		dir = DIR_BACKWARD;
	else
		dir = DIR_FORWARD;
	if (rc_debug) {
		printk("%lld: ", ts);
		for (int i = 0; i < 16; i++)
			printk("%04x ", chan_val[i]);
		printk("%02x, pw %u d %u thr %u\n", flags, pulse_width, dir, throttle);

	}
	if (rc_enable) {
		ret = servo_write(servo, pulse_width);
		if (ret < 0) {
			printk("Error %d: failed to set pulse width\n", ret);
		}
		ret = motor_write(motor_r, dir, throttle);
		if (ret < 0) {
			printk("Error %d: failed to set motor_r\n", ret);
		}
		ret = motor_write(motor_l, dir, throttle);
		if (ret < 0) {
			printk("Error %d: failed to set motor_l\n", ret);
		}
	}
}


static void callback(struct input_event *evt) {
	int ret;
	static uint32_t pulse_width = 50;
	static uint32_t throttle = 0;
	static uint32_t dir = DIR_STOP;
	static uint32_t brake = false;

	switch (evt->code) {
	case INPUT_ABS_RUDDER:
		pulse_width = PAM(evt->value, 0xF0, 0x720);
		break;
	case INPUT_ABS_GAS:
		int32_t c1 = evt->value - 0x3e8;
		throttle = PAM(abs(c1), 0, 0x720/2);
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
	}
	if (!evt->sync)
		return;

	if (brake)
		dir = DIR_BREAK;

	if (rc_debug) {
		printk("pw %u d %u thr %u brk %u\n", pulse_width, dir, throttle, brake);
	}
	if (rc_enable) {
		ret = servo_write(servo, pulse_width);
		if (ret < 0) {
			printk("Error %d: failed to set pulse width\n", ret);
		}
		ret = motor_write(motor_r, dir, throttle);
		if (ret < 0) {
			printk("Error %d: failed to set motor_r\n", ret);
		}
		ret = motor_write(motor_l, dir, throttle);
		if (ret < 0) {
			printk("Error %d: failed to set motor_l\n", ret);
		}
	}
};

INPUT_CALLBACK_DEFINE( DEVICE_DT_GET(RC_IN), callback);

int main(void)
{
	printk("Yahboom demo\n");

	if (!device_is_ready(servo)) {
		printk("Error: PWM device %s is not ready\n", servo->name);
		return 1;
	}
	if (!device_is_ready(receiver)) {
		printk("RC IN device not ready.\n");
		return 1;
	}

	if (!device_is_ready(motor_r)) {
		printk("motor_r not ready.\n");
		return 1;
	}

	if (!device_is_ready(motor_l)) {
		printk("motor_l not ready.\n");
		return 1;
	}
	return 0;
	while (1)  {
		process_events();
        k_sleep(K_MSEC(50));
    }
	return 0;
}
#ifdef CONFIG_SHELL
static int cmd_motor_set(const struct shell *sh, size_t argc,
			      char **argv)
{
	uint32_t dir, throttle;
	char m;
	int ret;

	if (argc == 1) {
		return 0;
	}
	if (argc < 4)
		return -EINVAL;

	if (sscanf(argv[1],"%c", &m) != 1 || sscanf(argv[2],"%u", &dir) != 1 || sscanf(argv[3],"%u", &throttle) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	if (m != 'l' && m != 'r') {
		shell_help(sh);
		return -EINVAL;
	}

	const struct device *motor = (m == 'l' ? motor_l : motor_r);

	ret = motor_write(motor, dir, throttle);
	if (ret) {
		printk("Error %d: failed to set motor\n", ret);
	}
	return ret;
}

static int cmd_servo_pulse(const struct shell *sh, size_t argc,
			      char **argv)
{
	uint32_t pulse;
	if (argc == 1) {
		uint8_t val;
		servo_read(servo, &val);
		shell_print(sh, "%u", val);
		return 0;
	}
	if (sscanf(argv[1],"%u", &pulse) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	if (pulse > 100) {
		shell_error(sh, "pulse width between %d & %d please", 0, 100);
		return -EINVAL;
	}
	shell_print(sh, "Set servo pulse %u", pulse);
	int ret = servo_write(servo, (uint8_t)pulse);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
	}
	return ret;
}

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

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD(set, NULL, "dir throttle", cmd_motor_set),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
	SHELL_CMD(pulse, NULL, "Cammand using getopt in non thread safe way"
	  " looking for: \"abhc:\".\n", cmd_servo_pulse),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_rc,
	SHELL_CMD(debug, NULL, " 1|0\n", cmd_rc_debug),
	SHELL_CMD(enable, NULL, " 1|0\n", cmd_rc_enable),
	SHELL_CMD(stats, NULL, " stats ", cmd_rc_stats),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(motor, &sub_motor, "motor commands", NULL);
SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
SHELL_CMD_REGISTER(rc, &sub_rc, "rc commands", NULL);
#endif
