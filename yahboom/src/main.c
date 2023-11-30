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
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/rc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/shell/shell.h>
#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>

#define RC_IN DT_NODELABEL(rc)
#define SERVO DT_NODELABEL(servo)

const struct device *receiver = DEVICE_DT_GET(RC_IN);
static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(SERVO);
static const uint32_t min_pulse = DT_PROP(SERVO, min_pulse);
static const uint32_t max_pulse = DT_PROP(SERVO, max_pulse);

static int rc_debug = 0;
int main(void)
{
	uint32_t pulse_width;
	int ret;

	printk("Yahboom demo\n");

	if (!pwm_is_ready_dt(&servo)) {
		printk("Error: PWM device %s is not ready\n", servo.dev->name);
		return 1;
	}

	if (!device_is_ready(receiver)) {
		printk("RC IN device not ready.\n");
		return 1;
	}
	
	while (1)  {
		struct Command rc_in;
        rc_update(receiver, &rc_in);
        if (1) {
			pulse_width = min_pulse + (max_pulse - min_pulse) / 2 + rc_in.yaw * (max_pulse - min_pulse);
			if (rc_debug) {
				printk("Got roll: %d, pitch: %d, thrust: %d, yaw: %d, armed: %d pw: %d\n",
					(int)(rc_in.roll*100), (int)(rc_in.pitch*100), 
					(int)(rc_in.thrust*100), (int)(rc_in.yaw*100),
					(int)(rc_in.armed), pulse_width);
			}
            ret = pwm_set_pulse_dt(&servo, pulse_width);
			if (ret < 0) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
        }
        k_sleep(K_MSEC(50));
    }
	return 0;
}

static int cmd_servo_pulse(const struct shell *sh, size_t argc,
			      char **argv)
{
	int pulse_us;
	if (sscanf(argv[1],"%d", &pulse_us) != 1) {
		shell_help(sh);
		return -EINVAL;
	}
	if (pulse_us < min_pulse / 1000 || pulse_us > max_pulse / 1000) {
		shell_error(sh, "pulse width between %d & %d please", min_pulse / 1000, max_pulse / 1000);
		return -EINVAL;
	}
	shell_print(sh, "Set servo pulse %dus", pulse_us);
	int ret = pwm_set_pulse_dt(&servo, PWM_USEC(pulse_us));
	if (ret < 0) {
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


SHELL_STATIC_SUBCMD_SET_CREATE(sub_servo,
	SHELL_CMD(pulse, NULL, "Cammand using getopt in non thread safe way"
	  " looking for: \"abhc:\".\n", cmd_servo_pulse),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_rc,
	SHELL_CMD(debug, NULL, " 1|0\n", cmd_rc_debug),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
SHELL_CMD_REGISTER(rc, &sub_rc, "rc commands", NULL);
