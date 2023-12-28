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
//#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include "battery.h"
#include "buzzer.h"
#include "motor.h"
#include "rc.h"

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define SERVO0 DT_NODELABEL(servo0)
#define SERVO1 DT_NODELABEL(servo1)
#define SERVO2 DT_NODELABEL(servo2)


static const struct device *servo[] = {
	DEVICE_DT_GET(SERVO0),
	DEVICE_DT_GET(SERVO1),
	DEVICE_DT_GET(SERVO2),
};
#define NUM_SERVOS (sizeof(servo)/sizeof(servo[0]))

int servo_init(void)
{
	for (int i = 0; i < NUM_SERVOS; i++)
		if (!device_is_ready(servo[i])) {
			printk("Error: PWM device %s is not ready\n", servo[i]->name);
		}
	return 0;
}

int servo_steer(int32_t val) {
	return servo_write(servo[0], val);
}

#ifdef CONFIG_SHELL

static int cmd_servo_pulse(const struct shell *sh, size_t argc,
			      char **argv)
{
	uint32_t pulse;
	uint32_t id;
	if (argc < 2 || sscanf(argv[1], "%u", &id) != 1 || id > 2) {
		shell_help(sh);
		return -EINVAL;
	}
	if (argc == 2) {
		uint8_t val;
		servo_read(servo[id], &val);
		shell_print(sh, "%u", val);
		return 0;
	}
	if (sscanf(argv[2],"%u", &pulse) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	if (pulse > 100) {
		shell_error(sh, "pulse width between %d & %d please", 0, 100);
		return -EINVAL;
	}
	shell_print(sh, "Set servo %u pulse %u", id, pulse);
	int ret = servo_write(servo[id], (uint8_t)pulse);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
	}
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
	SHELL_CMD(pulse, NULL, "Cammand using getopt in non thread safe way"
	  " looking for: \"abhc:\".\n", cmd_servo_pulse),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
#endif
