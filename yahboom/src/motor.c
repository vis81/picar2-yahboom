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
#include "zephyr/drivers/motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include "motor.h"

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define DEF_KO 100

static const struct device *motors[] = {
	[MOTOR_R] = DEVICE_DT_GET(DT_NODELABEL(motor_r)),
	[MOTOR_L] = DEVICE_DT_GET(DT_NODELABEL(motor_l)),
};

#define NUM_MOTORS (sizeof(motors)/sizeof(motors[0]))

int motor_init() {
	for (int i = 0; i < NUM_MOTORS; i++)
		if (!device_is_ready(motors[i])) {
			printk("Error: motor device %s is not ready\n", motors[i]->name);
		}
	return 0;
}

void motor_stop_all(void)
{
	for (int i = 0; i < NUM_MOTORS; i++) {
		motor_write(motors[i], DIR_STOP, 0);
	}
}

int motor_throttle(enum motor_id id, uint32_t dir, uint32_t throttle) {
	if (id > MOTOR_LAST)
		return -EINVAL;
	return motor_write(motors[id], dir, throttle);
}

int motor_speed(enum motor_id id, int32_t speed) {
	if (id > MOTOR_LAST)
		return -EINVAL;
	return motor_set_velocity(motors[id], speed);
}

int motor_pos(enum motor_id id, int32_t *pos) {
	if (id > MOTOR_LAST)
		return -EINVAL;
	return motor_get_position(motors[id], pos);
}

int motor_vel(enum motor_id id, int32_t *vel) {
	if (id > MOTOR_LAST)
		return -EINVAL;
	return motor_get_velocity(motors[id], vel);
}


#ifdef CONFIG_SHELL
static int cmd_motor_throttle(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t dir;
	uint32_t throttle;
	char m;
	int ret;

	if ( (argc != 2 && argc != 4)
		|| sscanf(argv[1],"%c", &m) != 1
		|| (m != 'l' && m != 'r')
		|| (argc == 4 && (sscanf(argv[2],"%u", &dir) != 1  || sscanf(argv[3],"%u", &throttle) != 1))
		) {
		shell_help(sh);
		return -EINVAL;
	}
	const struct device *motor = (m == 'r' ? motors[MOTOR_R] : motors[MOTOR_L]);
	shell_error(sh, "motor %s %s %c", motor->name, argv[1], m);
	if (argc == 2) {
		ret = motor_read(motor, (enum motor_dir *)&dir, &throttle);
		if (!ret)
			shell_print(sh,"%d %d", dir, throttle);
	} else
		ret = motor_write(motor, dir, throttle);

	if (ret)
		shell_error(sh, "error %d", ret);
	return ret;
}

static int cmd_motor_vel(const struct shell *sh, size_t argc,
			      char **argv)
{
	int32_t vel;
	char m;
	int ret = 0;

	if ( (argc != 2 && argc != 3)
		|| sscanf(argv[1],"%c", &m) != 1
		|| (m != 'l' && m != 'r')
		|| (argc == 3 && (sscanf(argv[2],"%d", &vel) != 1))
		) {
		shell_help(sh);
		return -EINVAL;
	}

	const struct device *motor = (m == 'r' ? motors[MOTOR_R] : motors[MOTOR_L]);
	if (argc == 2) {
		ret = motor_get_velocity(motor, &vel);
		if (!ret)
			shell_print(sh,"%d", vel);
	} else
		ret = motor_set_velocity(motor, vel);
	if (ret)
		shell_error(sh, "error %d", ret);
	return ret;
}

static int cmd_motor_pid(const struct shell *sh, size_t argc,
			      char **argv)
{
	float kp, ki, kd;
	char m;

	if (argc < 5
		|| sscanf(argv[1],"%c", &m) != 1
		|| (m != 'l' && m != 'r')
		|| sscanf(argv[2],"%f", &kp) != 1
		|| sscanf(argv[3],"%f", &kd) != 1
		|| sscanf(argv[4],"%f", &ki) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	const struct device *motor = (m == 'r' ? motors[MOTOR_R] : motors[MOTOR_L]);
	int ret = motor_set_pid(motor, kp * DEF_KO, kd * DEF_KO, ki * DEF_KO, DEF_KO);
	if (ret)
		shell_error(sh, "error %d", ret);
	return 0;
}

static int cmd_motor_pos(const struct shell *sh, size_t argc,
			      char **argv)
{
	int32_t pos;
	char m;
	int ret;

	if (argc < 2 || sscanf(argv[1],"%c", &m) != 1 || (m != 'l' && m != 'r')) {
		shell_help(sh);
		return -EINVAL;
	}
	const struct device *motor = (m == 'r' ? motors[MOTOR_R] : motors[MOTOR_L]);

	ret = motor_get_position(motor, &pos);
	if (ret)
		shell_error(sh, "error %d", ret);
	else
		shell_print(sh,"%d", pos);
	return ret;
}

static int cmd_motor_debug(const struct shell *sh, size_t argc,
			      char **argv)
{
	char m;
	int ret;
	int val;

	if (argc < 3
		|| sscanf(argv[1],"%c", &m) != 1
		|| (m != 'l' && m != 'r')
		|| sscanf(argv[2],"%d", &val) != 1 ) {
		shell_help(sh);
		return -EINVAL;
	}
	const struct device *motor = (m == 'r' ? motors[MOTOR_R] : motors[MOTOR_L]);
	struct motor_config cfg;
	cfg.flags.pid_debug = val;
	ret = motor_configure(motor, MOTOR_CFG_PID_DEBUG, &cfg);
	if (ret)
		shell_error(sh, "error %d", ret);
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
	SHELL_CMD(throttle, NULL, "l|r dir throttle", cmd_motor_throttle),
	SHELL_CMD(pos, NULL, "l|r", cmd_motor_pos),
	SHELL_CMD(vel, NULL, "l|r", cmd_motor_vel),
	SHELL_CMD(pid, NULL, "l|r", cmd_motor_pid),
	SHELL_CMD(debug, NULL, "l|r", cmd_motor_debug),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(motor, &sub_motor, "motor commands", NULL);
#endif
