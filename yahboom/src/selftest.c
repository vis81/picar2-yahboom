/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/motor.h>
#include "battery.h"
#include "imu.h"
#include "motor.h"
#include "servo.h"

#define MOTOR_SPIN_THROTTLE  30   /* % — low enough to be safe */
#define MOTOR_SPIN_MS        400  /* ms — long enough to see encoder ticks */
#define SERVO_DELAY_MS       500  /* ms — to see servo movement */

#define ITEM_W 22

#define _result(sh, ok, name, fmt, ...) \
	do { \
		if (ok) shell_print(sh, "  %-*s PASS  " fmt, ITEM_W, name, ##__VA_ARGS__); \
		else    shell_error(sh, "  %-*s FAIL  " fmt, ITEM_W, name, ##__VA_ARGS__); \
	} while (0)

static int cmd_selftest(const struct shell *sh, size_t argc, char **argv)
{
	int pass = 0, total = 0;
	int ret;

	shell_print(sh, "--- self-test ---");

	/* Battery ADC */
	total++;
	int32_t vbat;
	ret = battery_read(&vbat);
	if (ret) {
		_result(sh, false, "battery ADC", "read error %d", ret);
	} else if (vbat <= 0) {
		_result(sh, false, "battery ADC", "implausible %d mV", vbat);
	} else {
		_result(sh, true, "battery ADC", "%d mV", vbat);
		pass++;
	}

	/* IMU: I2C presence check via WHO_AM_I */
	total++;
	uint8_t who;
	ret = imu_whoami(&who);
	if (ret) {
		_result(sh, false, "IMU I2C", "error %d", ret);
	} else if (who != 0x71) {
		_result(sh, false, "IMU I2C", "WHO_AM_I=0x%02x (expect 0x71)", who);
	} else {
		_result(sh, true, "IMU I2C", "WHO_AM_I=0x%02x", who);
		pass++;
	}

	/* IMU: sensor data fetch */
	total++;
	ret = imu_sample();
	if (ret) {
		_result(sh, false, "IMU sample", "error %d", ret);
	} else {
		_result(sh, true, "IMU sample", "");
		pass++;
	}

	/* Servos: write 30, read back, restore to neutral */
	for (int i = 0; i < 3; i++) {
		char name[10];
		snprintk(name, sizeof(name), "servo %d", i);
		total++;

		uint8_t val = 0;
		ret = servo_write_id(i, 30);
		k_msleep(1000);
		if (!ret) {
			ret = servo_read_id(i, &val);
		}
		servo_write_id(i, 50);

		if (ret) {
			_result(sh, false, name, "error %d", ret);
		} else if (val != 30) {
			_result(sh, false, name, "readback %u, expect 30", val);
		} else {
			_result(sh, true, name, "");
			pass++;
		}
	}

	/* Motors: brief forward spin, verify encoder moves */
	const char *mname[MOTOR_CNT] = { "motor L spin", "motor R spin" };
	for (int i = 0; i < MOTOR_CNT; i++) {
		total++;
		int32_t pos_before, pos_after;

		motor_pos(i, &pos_before);
		motor_throttle(i, DIR_FORWARD, MOTOR_SPIN_THROTTLE);
		k_msleep(MOTOR_SPIN_MS);
		motor_throttle(i, DIR_STOP, 0);
		ret = motor_pos(i, &pos_after);

		if (ret) {
			_result(sh, false, mname[i], "pos read error %d", ret);
		} else if (pos_after == pos_before) {
			_result(sh, false, mname[i], "no encoder movement");
		} else {
			_result(sh, true, mname[i], "delta=%d ticks",
				pos_after - pos_before);
			pass++;
		}
	}

	shell_print(sh, "--- %d/%d passed ---", pass, total);
	return (pass == total) ? 0 : -EIO;
}

SHELL_CMD_REGISTER(selftest, NULL, "Run hardware self-test", cmd_selftest);
