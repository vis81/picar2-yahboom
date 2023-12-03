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
#include <stdio.h>

#define RC_IN DT_NODELABEL(rc)
#define SERVO DT_NODELABEL(servo)

const struct device *receiver = DEVICE_DT_GET(RC_IN);
static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(SERVO);
static const uint32_t min_pulse = DT_PROP(SERVO, min_pulse);
static const uint32_t max_pulse = DT_PROP(SERVO, max_pulse);

static int rc_debug = 0;
static int rc_enable = 0;
int main(void)
{
	uint32_t pulse_width = 0;
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
		uint64_t ts;
		uint16_t chan_val[16];
		uint8_t flags;
        //for (int i = 0; i < 16; i++) 
		//	rc_read_channel(receiver, i, &chan_val[i], &ts);
		//rc_read_flags(receiver, &flags, &ts);
		rc_read_all(receiver, 16, chan_val, &flags, &ts);
        if (1) {
			//pulse_width = min_pulse + (max_pulse - min_pulse) / 2 + rc_in.yaw * (max_pulse - min_pulse);
			if (rc_debug) {
				printk("%lld: ", ts);
				for (int i = 0; i < 16; i++)
					printk("%04x ", chan_val[i]);
				printk("%02x\n", flags);

			}
			if (rc_enable) {
				ret = pwm_set_pulse_dt(&servo, pulse_width);
				if (ret < 0) {
					printk("Error %d: failed to set pulse width\n", ret);
				}
			}
        }
        k_sleep(K_MSEC(50));
    }
	return 0;
}
#ifdef CONFIG_SHELL
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
	struct rc_stats stats;
	rc_read_stats(receiver, &stats);
	shell_print(sh, "bytes        : %d", stats.rx_bytes);
	shell_print(sh, "bytes dropped: %d", stats.rx_bytes_dropped);
	shell_print(sh, "good         : %d", stats.rx_good);
	shell_print(sh, "bad          : %d", stats.rx_bad);
	shell_print(sh, "discarded    : %d (%d%%)", stats.rx_discarded, 
				stats.rx_good ? stats.rx_discarded * 100 / stats.rx_good : 0);
	shell_print(sh, "last_ts      : %lld", stats.last_ts);
	return 0;
}

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

SHELL_CMD_REGISTER(servo, &sub_servo, "servo commands", NULL);
SHELL_CMD_REGISTER(rc, &sub_rc, "rc commands", NULL);
#endif
