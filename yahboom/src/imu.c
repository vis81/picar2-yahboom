/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>
#include "imu.h"

#define MPU9250_I2C_ADDR   0x69
#define MPU9250_REG_WHO_AM_I 0x75
#define MPU9250_REG_DATA   0x3B  /* burst: accel(6) temp(2) gyro(6) */

static const struct device *imu    = DEVICE_DT_GET(DT_NODELABEL(mpu9250));
static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c3));

int imu_init(void)
{
	if (!device_is_ready(imu)) {
		printk("MPU-9250 not ready\n");
		return -ENODEV;
	}
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
	struct sensor_value accel[3], gyro[3], temp;
	int ret;

	ret = sensor_sample_fetch(imu);
	if (ret) {
		shell_error(sh, "fetch error %d", ret);
		return ret;
	}

	sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ,  gyro);
	sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP,  &temp);

	shell_print(sh, "accel  x: %d.%06d  y: %d.%06d  z: %d.%06d m/s²",
		accel[0].val1, abs(accel[0].val2),
		accel[1].val1, abs(accel[1].val2),
		accel[2].val1, abs(accel[2].val2));
	shell_print(sh, "gyro   x: %d.%06d  y: %d.%06d  z: %d.%06d rad/s",
		gyro[0].val1, abs(gyro[0].val2),
		gyro[1].val1, abs(gyro[1].val2),
		gyro[2].val1, abs(gyro[2].val2));
	shell_print(sh, "temp   %d.%02d °C",
		temp.val1, abs(temp.val2) / 10000);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_imu,
	SHELL_CMD(read,   NULL, "Read accel, gyro and temperature", cmd_imu_read),
	SHELL_CMD(raw,    NULL, "Dump raw 14-byte data burst", cmd_imu_raw),
	SHELL_CMD(whoami, NULL, "Print WHO_AM_I register", cmd_imu_whoami),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(imu, &sub_imu, "IMU commands", NULL);

#endif /* CONFIG_SHELL */
