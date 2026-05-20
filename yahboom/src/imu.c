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

#define CAL_DURATION_MS        15000
#define CAL_GYRO_DURATION_MS    5000
#define CAL_ACCEL_DURATION_MS   3000
#define CAL_SAMPLE_MS             50

static const struct device *imu     = DEVICE_DT_GET(DT_NODELABEL(mpu9250));
static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c3));

/* Hard-iron offsets in sensor_value micro-units (G × 1e6); zero = uncalibrated */
static int64_t mag_cal[3];

/* Gyro zero-rate bias in ×0.001 rad/s; zero = uncalibrated */
static int16_t gyro_cal[3];

/* Accel scale factor ×1000 (e.g. 1148 = 1.148×); zero = uncalibrated */
static int16_t accel_scale;

static int imu_settings_set(const char *name, size_t len,
			    settings_read_cb read_cb, void *cb_arg)
{
	if (strcmp(name, "mag_offsets") == 0 && len == sizeof(mag_cal)) {
		read_cb(cb_arg, mag_cal, sizeof(mag_cal));
	} else if (strcmp(name, "gyro_offsets") == 0 && len == sizeof(gyro_cal)) {
		read_cb(cb_arg, gyro_cal, sizeof(gyro_cal));
	} else if (strcmp(name, "accel_scale") == 0 && len == sizeof(accel_scale)) {
		read_cb(cb_arg, &accel_scale, sizeof(accel_scale));
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(imu, "imu", NULL, imu_settings_set, NULL, NULL);

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

int imu_whoami(uint8_t *who)
{
	return i2c_reg_read_byte(i2c_bus, MPU9250_I2C_ADDR,
				 MPU9250_REG_WHO_AM_I, who);
}

int imu_sample(void)
{
	return sensor_sample_fetch(imu);
}

int imu_get_data(struct imu_data *d)
{
	struct sensor_value a[3], g[3], m[3], t;
	int ret = sensor_sample_fetch(imu);

	if (ret) {
		return ret;
	}
	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, a);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,  g);
	sensor_channel_get(imu, SENSOR_CHAN_MAGN_XYZ,  m);
	sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP,  &t);
	apply_mag_cal(m);
	for (int i = 0; i < 3; i++) {
		int16_t raw_a = (int16_t)(a[i].val1 * 1000 + a[i].val2 / 1000);

		d->accel[i] = accel_scale
			? (int16_t)((int32_t)raw_a * accel_scale / 1000)
			: raw_a;
		d->gyro[i]  = (int16_t)(g[i].val1 * 1000 + g[i].val2 / 1000) - gyro_cal[i];
		d->magn[i]  = (int16_t)(m[i].val1 * 1000 + m[i].val2 / 1000);
	}
	d->temp = (int16_t)(t.val1 * 100 + t.val2 / 10000);
	return 0;
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
	struct imu_data d;
	int ret = imu_get_data(&d);

	if (ret) {
		shell_error(sh, "fetch error %d", ret);
		return ret;
	}

	shell_print(sh, "accel  x: %d.%03d  y: %d.%03d  z: %d.%03d m/s²",
		d.accel[0] / 1000, abs(d.accel[0] % 1000),
		d.accel[1] / 1000, abs(d.accel[1] % 1000),
		d.accel[2] / 1000, abs(d.accel[2] % 1000));
	shell_print(sh, "gyro   x: %d.%03d  y: %d.%03d  z: %d.%03d rad/s",
		d.gyro[0] / 1000, abs(d.gyro[0] % 1000),
		d.gyro[1] / 1000, abs(d.gyro[1] % 1000),
		d.gyro[2] / 1000, abs(d.gyro[2] % 1000));
	shell_print(sh, "magn   x: %d.%01d  y: %d.%01d  z: %d.%01d uT",
		d.magn[0] / 10, abs(d.magn[0] % 10),
		d.magn[1] / 10, abs(d.magn[1] % 10),
		d.magn[2] / 10, abs(d.magn[2] % 10));
	shell_print(sh, "temp   %d.%02d °C",
		d.temp / 100, abs(d.temp % 100));

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

	int ret = settings_save_one("imu/mag_offsets", mag_cal, sizeof(mag_cal));

	if (ret) {
		shell_error(sh, "settings save failed: %d", ret);
		return ret;
	}

	shell_print(sh, "done.  offsets  x: %d.%01d  y: %d.%01d  z: %d.%01d µT",
		(int32_t)(mag_cal[0] / 10000), (int32_t)abs((int32_t)((mag_cal[0] / 1000) % 10)),
		(int32_t)(mag_cal[1] / 10000), (int32_t)abs((int32_t)((mag_cal[1] / 1000) % 10)),
		(int32_t)(mag_cal[2] / 10000), (int32_t)abs((int32_t)((mag_cal[2] / 1000) % 10)));
	return 0;
}

static int cmd_imu_cal_show(const struct shell *sh, size_t argc, char **argv)
{
	if (mag_cal[0] == 0 && mag_cal[1] == 0 && mag_cal[2] == 0) {
		shell_print(sh, "not calibrated");
		return 0;
	}
	shell_print(sh, "offsets  x: %d.%01d  y: %d.%01d  z: %d.%01d µT",
		(int32_t)(mag_cal[0] / 10000), (int32_t)abs((int32_t)((mag_cal[0] / 1000) % 10)),
		(int32_t)(mag_cal[1] / 10000), (int32_t)abs((int32_t)((mag_cal[1] / 1000) % 10)),
		(int32_t)(mag_cal[2] / 10000), (int32_t)abs((int32_t)((mag_cal[2] / 1000) % 10)));
	return 0;
}

static int cmd_imu_cal_reset(const struct shell *sh, size_t argc, char **argv)
{
	mag_cal[0] = mag_cal[1] = mag_cal[2] = 0;
	settings_delete("imu/mag_offsets");
	shell_print(sh, "calibration cleared");
	return 0;
}

static int cmd_imu_cal_gyro(const struct shell *sh, size_t argc, char **argv)
{
	struct sensor_value g[3];
	int64_t sum[3] = {0};
	int count = 0;

	shell_print(sh, "Keep robot stationary for %d seconds...",
		    CAL_GYRO_DURATION_MS / 1000);

	int64_t end = k_uptime_get() + CAL_GYRO_DURATION_MS;

	while (k_uptime_get() < end) {
		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, g) == 0) {
			for (int i = 0; i < 3; i++) {
				sum[i] += g[i].val1 * 1000 + g[i].val2 / 1000;
			}
			count++;
		}
		k_msleep(CAL_SAMPLE_MS);
	}

	if (count == 0) {
		shell_error(sh, "no samples collected");
		return -EIO;
	}

	for (int i = 0; i < 3; i++) {
		gyro_cal[i] = (int16_t)(sum[i] / count);
	}

	int ret = settings_save_one("imu/gyro_offsets", gyro_cal, sizeof(gyro_cal));

	if (ret) {
		shell_error(sh, "settings save failed: %d", ret);
		return ret;
	}

	shell_print(sh, "done.  bias  x: %s%d.%03d  y: %s%d.%03d  z: %s%d.%03d rad/s",
		gyro_cal[0] < 0 ? "-" : "", abs(gyro_cal[0]) / 1000, abs(gyro_cal[0]) % 1000,
		gyro_cal[1] < 0 ? "-" : "", abs(gyro_cal[1]) / 1000, abs(gyro_cal[1]) % 1000,
		gyro_cal[2] < 0 ? "-" : "", abs(gyro_cal[2]) / 1000, abs(gyro_cal[2]) % 1000);
	return 0;
}

static int cmd_imu_cal_gyro_show(const struct shell *sh, size_t argc, char **argv)
{
	if (gyro_cal[0] == 0 && gyro_cal[1] == 0 && gyro_cal[2] == 0) {
		shell_print(sh, "not calibrated");
		return 0;
	}
	shell_print(sh, "bias  x: %s%d.%03d  y: %s%d.%03d  z: %s%d.%03d rad/s",
		gyro_cal[0] < 0 ? "-" : "", abs(gyro_cal[0]) / 1000, abs(gyro_cal[0]) % 1000,
		gyro_cal[1] < 0 ? "-" : "", abs(gyro_cal[1]) / 1000, abs(gyro_cal[1]) % 1000,
		gyro_cal[2] < 0 ? "-" : "", abs(gyro_cal[2]) / 1000, abs(gyro_cal[2]) % 1000);
	return 0;
}

static int cmd_imu_cal_gyro_reset(const struct shell *sh, size_t argc, char **argv)
{
	gyro_cal[0] = gyro_cal[1] = gyro_cal[2] = 0;
	settings_delete("imu/gyro_offsets");
	shell_print(sh, "gyro calibration cleared");
	return 0;
}

static uint32_t isqrt(uint64_t n)
{
	if (n == 0) {
		return 0;
	}
	uint64_t x = n, y = (x + 1) / 2;

	while (y < x) {
		x = y;
		y = (x + n / x) / 2;
	}
	return (uint32_t)x;
}

static int cmd_imu_cal_accel(const struct shell *sh, size_t argc, char **argv)
{
	struct sensor_value a[3];
	int64_t sum[3] = {0};
	int count = 0;

	shell_print(sh, "Keep robot still on flat floor for %d seconds...",
		    CAL_ACCEL_DURATION_MS / 1000);

	int64_t end = k_uptime_get() + CAL_ACCEL_DURATION_MS;

	while (k_uptime_get() < end) {
		if (sensor_sample_fetch(imu) == 0 &&
		    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, a) == 0) {
			for (int i = 0; i < 3; i++) {
				sum[i] += a[i].val1 * 1000 + a[i].val2 / 1000;
			}
			count++;
		}
		k_msleep(CAL_SAMPLE_MS);
	}

	if (count == 0) {
		shell_error(sh, "no samples collected");
		return -EIO;
	}

	int64_t ax = sum[0] / count, ay = sum[1] / count, az = sum[2] / count;
	uint32_t mag = isqrt((uint64_t)(ax * ax + ay * ay + az * az));

	if (mag < 1000) {
		shell_error(sh, "magnitude too small (%u milli-m/s²)", mag);
		return -EINVAL;
	}

	accel_scale = (int16_t)(9807u * 1000u / mag);

	int ret = settings_save_one("imu/accel_scale", &accel_scale, sizeof(accel_scale));

	if (ret) {
		shell_error(sh, "settings save failed: %d", ret);
		return ret;
	}

	shell_print(sh, "done.  |a|=%u milli-m/s²  scale=%d.%03d",
		    mag, accel_scale / 1000, accel_scale % 1000);
	return 0;
}

static int cmd_imu_cal_accel_show(const struct shell *sh, size_t argc, char **argv)
{
	if (accel_scale == 0) {
		shell_print(sh, "not calibrated");
		return 0;
	}
	shell_print(sh, "scale  %d.%03d", accel_scale / 1000, accel_scale % 1000);
	return 0;
}

static int cmd_imu_cal_accel_reset(const struct shell *sh, size_t argc, char **argv)
{
	accel_scale = 0;
	settings_delete("imu/accel_scale");
	shell_print(sh, "accel calibration cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu_cal_accel,
	SHELL_CMD(show,  NULL, "Print accel scale factor", cmd_imu_cal_accel_show),
	SHELL_CMD(reset, NULL, "Clear accel calibration", cmd_imu_cal_accel_reset),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu_cal_mag,
	SHELL_CMD(show,  NULL, "Print magnetometer offsets", cmd_imu_cal_show),
	SHELL_CMD(reset, NULL, "Clear magnetometer calibration", cmd_imu_cal_reset),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu_cal_gyro,
	SHELL_CMD(show,  NULL, "Print gyro bias", cmd_imu_cal_gyro_show),
	SHELL_CMD(reset, NULL, "Clear gyro calibration", cmd_imu_cal_gyro_reset),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu_cal,
	SHELL_CMD(accel, &sub_imu_cal_accel, "Accel scale calibration",          cmd_imu_cal_accel),
	SHELL_CMD(mag,   &sub_imu_cal_mag,   "Magnetometer hard-iron calibration", cmd_imu_cal_start),
	SHELL_CMD(gyro,  &sub_imu_cal_gyro,  "Gyro zero-rate bias calibration",  cmd_imu_cal_gyro),
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
