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
#include "servo.h"

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);


int main(void)
{
	printk("Yahboom demo\n");
	buzzer_init();
	battery_init();
	motor_init();
	servo_init();
	rc_init();

	//buzzer_play(BUZZER_FUNKYTOWN, 50);
}

#ifdef CONFIG_SHELL

static int cmd_m(const struct shell *sh, size_t argc,
			      char **argv)
{
	int32_t vel[MOTOR_CNT];
	int ret = 0;

	if ( argc != 3
		|| sscanf(argv[1],"%d", &vel[0]) != 1
		|| sscanf(argv[1],"%d", &vel[1]) != 1
		) {
		ret = -EINVAL;
		goto exit;
	}
	ret = motor_speed(MOTOR_L, vel[0]);
	ret |= motor_speed(MOTOR_R, vel[1]);
exit:
	shell_print(sh,"a:%d", ret);
	return ret;
}

static int cmd_p(const struct shell *sh, size_t argc,
			      char **argv)
{
	int32_t pos[MOTOR_CNT];
	int ret = 0;

	if ( argc != 1) {
		ret = -EINVAL;
		goto exit;
	}
	ret = motor_pos(MOTOR_L, &pos[0]);
	ret |= motor_pos(MOTOR_R, &pos[1]);
exit:
	shell_print(sh,"a:%d:%d,%d", ret, pos[0], pos[1]);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_c,
	SHELL_CMD(m, NULL, "", cmd_m),
	SHELL_CMD(p, NULL, "", cmd_p),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(c, &sub_c, "API commands", NULL);

#endif
