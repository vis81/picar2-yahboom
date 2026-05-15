/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _IMU_H_
#define _IMU_H_

struct imu_data {
	int16_t accel[3];  /* ×0.001 m/s²  */
	int16_t gyro[3];   /* ×0.001 rad/s */
	int16_t magn[3];   /* ×0.1 µT      */
	int16_t temp;      /* ×0.01 °C     */
};

int imu_init(void);
void imu_shutdown(void);
int imu_whoami(uint8_t *who);
int imu_sample(void);
int imu_get_data(struct imu_data *d);

#endif /* _IMU_H_ */
