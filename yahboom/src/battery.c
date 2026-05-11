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
#include "power.h"

#define ADC2VBAT(x)  ((x) * 4145 / 1000)

/* Set to 3 for 3S or 4 for 4S LiPo */
#define VBAT_CELLS    3

#define VBAT_MAX_MV  (4200 * VBAT_CELLS)  /* 4.2 V/cell — fully charged  */
#define VBAT_LOW_MV  (3500 * VBAT_CELLS)  /* 3.5 V/cell — warn threshold */
#define VBAT_CRIT_MV (3400 * VBAT_CELLS)  /* 3.4 V/cell — halt threshold */
#define VBAT_MIN_MV  (3300 * VBAT_CELLS)  /* 3.3 V/cell — practical empty */

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
		k_work_reschedule(&battery_mon_work, K_SECONDS(5));
		return;
	}
	if (val < VBAT_CRIT_MV) {
		printk("CRITICAL: battery %d mV — entering standby\n", val);
		system_shutdown();
		return;
	}
	if (val < VBAT_LOW_MV) {
		printk("WARNING: low battery: %d mV\n", val);
	}
	k_work_reschedule(&battery_mon_work, K_SECONDS(5));
}


#ifdef CONFIG_SHELL

static int cmd_voltage(const struct shell *sh, size_t argc, char **argv)
{
	int val;
	int err = battery_read(&val);
	if (err) {
		shell_error(sh, "error %d", err);
		return err;
	}
	shell_print(sh, "%d mV", val);
	return 0;
}

static int cmd_level(const struct shell *sh, size_t argc, char **argv)
{
	int val;
	int err = battery_read(&val);
	if (err) {
		shell_error(sh, "error %d", err);
		return err;
	}
	int pct = (val - VBAT_MIN_MV) * 100 / (VBAT_MAX_MV - VBAT_MIN_MV);
	pct = CLAMP(pct, 0, 100);
	shell_print(sh, "%d%%", pct);
	return 0;
}

static int cmd_info(const struct shell *sh, size_t argc, char **argv)
{
	int val;
	int err = battery_read(&val);
	if (err) {
		shell_error(sh, "error %d", err);
		return err;
	}
	int pct = CLAMP((val - VBAT_MIN_MV) * 100 / (VBAT_MAX_MV - VBAT_MIN_MV), 0, 100);

	shell_print(sh, "cells     %dS", VBAT_CELLS);
	shell_print(sh, "max       %d mV  (%.1f V/cell)", VBAT_MAX_MV,  4200 / 1000.0);
	shell_print(sh, "low       %d mV  (%.1f V/cell)", VBAT_LOW_MV,  3500 / 1000.0);
	shell_print(sh, "critical  %d mV  (%.1f V/cell)", VBAT_CRIT_MV, 3400 / 1000.0);
	shell_print(sh, "empty     %d mV  (%.1f V/cell)", VBAT_MIN_MV,  3300 / 1000.0);
	shell_print(sh, "voltage   %d mV", val);
	shell_print(sh, "level     %d%%", pct);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_battery,
	SHELL_CMD(info,    NULL, "Dump cell config, thresholds, voltage and level", cmd_info),
	SHELL_CMD(voltage, NULL, "Print battery voltage in mV", cmd_voltage),
	SHELL_CMD(level,   NULL, "Print battery charge level in %%", cmd_level),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(battery, &sub_battery, "Battery commands", NULL);
#endif
