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

	battery_init();
	motor_init();
	servo_init();
	rc_init();

	buzzer_play(BUZZER_FUNKYTOWN, 50);
}

#ifdef CONFIG_SHELL

#endif
