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
#include <zephyr/drivers/uart_pipe.h>
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

LOG_MODULE_REGISTER(control, LOG_LEVEL_INF);

#define UART_PIPE_BUF_SIZE 64

static char uart_pipe_rx_buf[2][UART_PIPE_BUF_SIZE];
static char uart_pipe_tx_buf[UART_PIPE_BUF_SIZE];
static uint8_t cur_buf = 0;

static void control_work_handler(struct k_work *work);
static K_WORK_DEFINE(work, control_work_handler);


int cmd_servo_read(const char* buf, int32_t *pval) {
	int32_t idx;
	int ret;
	uint8_t val;
	if(sscanf(buf, "$r%d", &idx) != 1)
		return -EINVAL;
	ret = servo_get(&val);
	*pval = val;
	return ret ? ret : 1;
}

int cmd_servo_write(const char* buf) {
	int32_t idx;
	int val;
	if(sscanf(buf, "$w%d:%d", &idx, &val) != 2)
		return -EINVAL;
	return servo_steer(val);
}

int cmd_position(const char* buf, int32_t val[2]) {
	int32_t idx;
	int ret;
	if(sscanf(buf, "$p%d", &idx) != 1)
		return -EINVAL;
	ret = motor_pos(idx, &val[0]);
	return ret ? ret : 1;
}

int cmd_speed(const char* buf) {
	int32_t idx, val;

	if(sscanf(buf, "$s%d:%d", &idx, &val) != 2)
		return -EINVAL;
	return motor_speed(idx, val);
}

int cmd_throttle(const char* buf) {
	int32_t idx, thr, dir;

	if(sscanf(buf, "$t%d:%d,%d", &idx, &dir, &thr) != 3)
		return -EINVAL;
	return motor_throttle(idx, dir, thr);
}

int parse_cmd(const char* buf, char* resp) {
	int ret;
	int32_t retval[2];
	size_t len;

	if (buf[0] != '$') {
		LOG_ERR("No start byte");
		return -EINVAL;
	}
	if (buf[1] == 'r')
		ret = cmd_servo_read(buf, &retval[0]);
	else if (buf[1] == 'w')
		ret = cmd_servo_write(buf);
	else if (buf[1] == 'p')
		ret = cmd_position(buf, retval);
	else if (buf[1] == 's')
		ret = cmd_speed(buf);
	else if (buf[1] == 't')
		ret = cmd_throttle(buf);
	else if (buf[1] == 'e') {
		sprintf(resp, "@0:%s", &buf[3]);
		return 0;
	} else
		ret = -EINVAL;

	LOG_DBG("cmd ret %d", ret);
	resp[0] = '@';
	sprintf(&resp[1], "%d", ret);
	len = strlen(resp);
	if (ret > 0) {
		for (int i = 0; i < ret; i++) {
			const char* fmt = (i == 0 ? ":%d" : ",%d");
			sprintf(&resp[len], fmt, retval[i]);
			len = strlen(resp);
		}
	}
	sprintf(&resp[len], "\n");
	return 0;
}

static void control_work_handler(struct k_work *work) {
	char* buf = uart_pipe_rx_buf[!cur_buf];

	if (buf[0] != '$') {
		LOG_ERR("No start byte");
		return;
	}
	if (parse_cmd(buf, uart_pipe_tx_buf) == 0)
		uart_pipe_send(uart_pipe_tx_buf, strlen(uart_pipe_tx_buf));
}


static uint8_t *recv_cb(uint8_t *buf, size_t *off)
{
	if (*off >= UART_PIPE_BUF_SIZE) {
		LOG_ERR("rx overflow");
		*off = 0;
	}
	if (buf[*off - 1] == '\n') {
		buf[*off] = 0;
		LOG_DBG("rx %d: %s", *off, buf);
		*off = 0;
		cur_buf = !cur_buf;
		k_work_submit(&work);
		return uart_pipe_rx_buf[cur_buf];
	}

	return buf;
}


int control_init(void)
{
	uart_pipe_register(uart_pipe_rx_buf[0], sizeof(uart_pipe_rx_buf), recv_cb);
	LOG_INF("Control init Ok");
	return 0;
}


#ifdef CONFIG_SHELL

static int cmd_shell_control(const struct shell *sh, size_t argc,
			      char **argv)
{
	char ret_buf[UART_PIPE_BUF_SIZE];
	if ( argc != 2)
		return -EINVAL;
	int ret = parse_cmd(argv[1], ret_buf);
	if (ret == 0) {
		ret_buf[strlen(ret_buf)-1] = 0;
		shell_print(sh, "resp: %s", ret_buf);
	} else
		shell_error(sh,"error %d", ret);
	return ret;
}

SHELL_CMD_REGISTER(c, NULL, "API commands", cmd_shell_control);

#endif
