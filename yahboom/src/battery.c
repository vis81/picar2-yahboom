/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc_compat.h>
#include <zephyr/shell/shell.h>
//#include "buzzer.h"

#define ADC2VBAT(x)  ((x) * 4145 / 1000)

static const struct adc_dt_spec adc_ch_vbat =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static void battery_mon_func(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(battery_mon_work, battery_mon_func);

int battery_init() {
	if (!adc_is_ready_dt(&adc_ch_vbat)) {
		printk("ADC controller device %s not ready\n", adc_ch_vbat.dev->name);
		return -EBUSY;
	}

	int err = adc_channel_setup_dt(&adc_ch_vbat);
	if (err < 0) {
		printk("Could not setup channel #%d (%d)\n", adc_ch_vbat.channel_id, err);
		return err;
	}
	k_work_reschedule(&battery_mon_work, K_SECONDS(5));
	return 0;
}

int battery_read(int32_t *value)
{
	int err;
	uint16_t buf;
	int32_t val_mv;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};
	
	adc_sequence_init_dt(&adc_ch_vbat, &sequence);
	err = adc_read_dt(&adc_ch_vbat, &sequence);
	if (err)
		return err;
	val_mv = (int32_t)buf;
	adc_raw_to_millivolts_dt(&adc_ch_vbat, &val_mv);
	*value = ADC2VBAT(val_mv);
	return 0;
}

static void battery_mon_func(struct k_work *work)
{
	int val;
	int err = battery_read(&val);
	if (err) {
		printk("read battery voltage error %d\n", err);
		return;
	}
	printk("battery: %d mV\n", val);
	k_work_reschedule(&battery_mon_work, K_SECONDS(5));	
}


#ifdef CONFIG_SHELL

static int cmd_read(const struct shell *sh, size_t argc,
			      char **argv)
{
	int val;
	int err = battery_read(&val);
	if (err) {
		shell_error(sh, "error %d", err);
		return err;
	}
	shell_print(sh, "%d", val);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_battery,
	SHELL_CMD(read, NULL, "", cmd_read),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(battery, &sub_battery, "rc commands", NULL);
#endif
