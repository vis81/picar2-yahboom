/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _IMU_H_
#define _IMU_H_

int imu_init(void);
void imu_shutdown(void);
int imu_whoami(uint8_t *who);
int imu_sample(void);

#endif /* _IMU_H_ */
