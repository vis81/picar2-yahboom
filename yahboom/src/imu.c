/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include "imu.h"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_DBG);

#define MPU9250_I2C_ADDR     0x69
#define MPU9250_REG_WHO_AM_I 0x75
#define MPU9250_REG_DATA     0x3B  /* burst: accel(6) temp(2) gyro(6) */
#define MPU9250_REG_PWR_MGMT_1 0x6B
#define MPU9250_PWR_SLEEP    BIT(6)

#define CAL_DURATION_MS  15000
#define CAL_SAMPLE_MS    50

static const struct device *imu     = DEVICE_DT_GET(DT_NODELABEL(mpu9250));
static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c3));

/* Hard-iron offsets in sensor_value micro-units (µT × 1e6); zero = uncalibrated */
static int64_t mag_cal[3];

static int mag_cal_set(const char *name, size_t len,
		       settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "offsets") == 0 && len == sizeof(mag_cal)) {
		read_cb(cb_arg, mag_cal, sizeof(mag_cal));
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(imu, "imu", NULL, mag_cal_set, NULL, NULL);

static void apply_mag_cal(struct sensor_value *m)
{
	for (int i = 0; i < 3; i++) {
		int64_t v = (int64_t)m[i].val1 * 1000000LL + m[i].val2;

		v -= mag_cal[i];
		m[i].val1 = (int32_t)(v / 1000000);
		m[i].val2 = (int32_t)(v % 1000000);
	}
}

void imu_shutdown(void)
{
	i2c_reg_write_byte(i2c_bus, MPU9250_I2C_ADDR,
			   MPU9250_REG_PWR_MGMT_1, MPU9250_PWR_SLEEP);
}

int imu_init(void)
{
	if (!device_is_ready(imu)) {
		printk("MPU-9250 not ready\n");
		return -ENODEV;
	}

	settings_subsys_init();
	settings_load_subtree("imu");
	return 0;
}

#ifdef CONFIG_SHELL

static int cmd_imu_whoami(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t who;
	int ret = i2c_reg_read_byte(i2c_bus, MPU9250_I2C_ADDR, MPU9250_REG_WHO_AM_I, &who);

	if (ret) {
		shell_error(sh, "I2C error %d", ret);
		return ret;
	}
	shell_print(sh, "WHO_AM_I: 0x%02x (expect 0x71)", who);
	return 0;
}

static int cmd_imu_raw(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t buf[14];
	int ret = i2c_burst_read(i2c_bus, MPU9250_I2C_ADDR, MPU9250_REG_DATA, buf, 14);

	if (ret) {
		shell_error(sh, "I2C error %d", ret);
		return ret;
	}
	shell_print(sh,
		"raw: %02x%02x %02x%02x %02x%02x  %02x%02x  %02x%02x %02x%02x %02x%02x",
		buf[0],  buf[1],  buf[2],  buf[3],  buf[4],  buf[5],
		buf[6],  buf[7],
		buf[8],  buf[9],  buf[10], buf[11], buf[12], buf[13]);
	shell_print(sh, "     [ax]  [ay]  [az]  [temp]  [gx]  [gy]  [gz]");
	return 0;
}

static int cmd_imu_read(const struct shell *sh, size_t argc, char **argv)
{
	struct sensor_value accel[3], gyro[3], magn[3], temp;
	int ret;

	ret = sensor_sample_fetch(imu);
	if (ret) {
		shell_error(sh, "fetch error %d", ret);
		return ret;
	}

	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,  gyro);
	sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP,  &temp);
	sensor_channel_get(imu, SENSOR_CHAN_MAGN_XYZ,  magn);
	apply_mag_cal(magn);

	shell_print(sh, "accel  x: %d.%06d  y: %d.%06d  z: %d.%06d m/s²",
		accel[0].val1, abs(accel[0].val2),
		accel[1].val1, abs(accel[1].val2),
		accel[2].val1, abs(accel[2].val2));
	shell_print(sh, "gyro   x: %d.%06d  y: %d.%06d  z: %d.%06d rad/s",
		gyro[0].val1, abs(gyro[0].val2),
		gyro[1].val1, abs(gyro[1].val2),
		gyro[2].val1, abs(gyro[2].val2));
	shell_print(sh, "magn   x: %d.%06d  y: %d.%06d  z: %d.%06d uT",
		magn[0].val1, abs(magn[0].val2),
		magn[1].val1, abs(magn[1].val2),
		magn[2].val1, abs(magn[2].val2));
	shell_print(sh, "temp   %d.%02d °C",
		temp.val1, abs(temp.val2) / 10000);

	return 0;
}

/* ── Calibration ─────────────────────────────────────────────────────────── */

static int cmd_imu_cal_start(const struct shell *sh, size_t argc, char **argv)
{
	struct sensor_value magn[3];
	int64_t mn[3], mx[3];
	bool first = true;
	int prev_sec = -1;

	shell_print(sh, "Rotate board in all directions for %d seconds...",
		    CAL_DURATION_MS / 1000);

	int64_t end = k_uptime_get() + CAL_DURATION_MS;

	while (k_uptime_get() < end) {
		int sec = (int)((end - k_uptime_get() + 999) / 1000);

		if (sec != prev_sec) {
			shell_print(sh, "%d...", sec);
			prev_sec = sec;
		}

		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_MAGN_XYZ, magn) == 0) {
			for (int i = 0; i < 3; i++) {
				int64_t v = (int64_t)magn[i].val1 * 1000000LL + magn[i].val2;

				if (first) {
					mn[i] = mx[i] = v;
				} else {
					if (v < mn[i]) mn[i] = v;
					if (v > mx[i]) mx[i] = v;
				}
			}
			first = false;
		}

		k_msleep(CAL_SAMPLE_MS);
	}

	if (first) {
		shell_error(sh, "no samples collected");
		return -EIO;
	}

	for (int i = 0; i < 3; i++) {
		mag_cal[i] = (mn[i] + mx[i]) / 2;
	}

	int ret = settings_save_one("imu/offsets", mag_cal, sizeof(mag_cal));

	if (ret) {
		shell_error(sh, "settings save failed: %d", ret);
		return ret;
	}

	shell_print(sh, "done.  offsets  x: %d.%06d  y: %d.%06d  z: %d.%06d uT",
		(int32_t)(mag_cal[0] / 1000000), (int32_t)abs(mag_cal[0] % 1000000),
		(int32_t)(mag_cal[1] / 1000000), (int32_t)abs(mag_cal[1] % 1000000),
		(int32_t)(mag_cal[2] / 1000000), (int32_t)abs(mag_cal[2] % 1000000));
	return 0;
}

static int cmd_imu_cal_show(const struct shell *sh, size_t argc, char **argv)
{
	if (mag_cal[0] == 0 && mag_cal[1] == 0 && mag_cal[2] == 0) {
		shell_print(sh, "not calibrated");
		return 0;
	}
	shell_print(sh, "offsets  x: %d.%06d  y: %d.%06d  z: %d.%06d uT",
		(int32_t)(mag_cal[0] / 1000000), (int32_t)abs(mag_cal[0] % 1000000),
		(int32_t)(mag_cal[1] / 1000000), (int32_t)abs(mag_cal[1] % 1000000),
		(int32_t)(mag_cal[2] / 1000000), (int32_t)abs(mag_cal[2] % 1000000));
	return 0;
}

static int cmd_imu_cal_reset(const struct shell *sh, size_t argc, char **argv)
{
	mag_cal[0] = mag_cal[1] = mag_cal[2] = 0;
	settings_delete("imu/offsets");
	shell_print(sh, "calibration cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu_cal,
	SHELL_CMD(start, NULL, "Collect hard-iron offsets (rotate board for 15 s)", cmd_imu_cal_start),
	SHELL_CMD(show,  NULL, "Print current offsets", cmd_imu_cal_show),
	SHELL_CMD(reset, NULL, "Clear calibration", cmd_imu_cal_reset),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu,
	SHELL_CMD(read,   NULL, "Read accel, gyro, magn and temperature", cmd_imu_read),
	SHELL_CMD(raw,    NULL, "Dump raw 14-byte data burst", cmd_imu_raw),
	SHELL_CMD(whoami, NULL, "Print WHO_AM_I register", cmd_imu_whoami),
	SHELL_CMD(cal,    &sub_imu_cal, "Magnetometer calibration", NULL),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(imu, &sub_imu, "IMU commands", NULL);

#endif /* CONFIG_SHELL */
