/*
 * Copyright (c) 2022 Jonathan Hahn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
//#include <msgs/msgs.h>

#ifndef ZEPHYR_MOTOR_H
#define ZEPHYR_MOTOR_H

struct motor_stats {
	uint32_t rx_bytes;
	uint32_t rx_bytes_dropped;
    uint32_t rx_good;
    uint32_t rx_bad;
    uint32_t rx_discarded;
    uint64_t last_ts;
};

enum motor_dir {
	DIR_STOP,
	DIR_BREAK,
	DIR_FORWARD,
	DIR_BACKWARD,
	DIR_LAST = DIR_BACKWARD,
};

struct motor_driver_api {
	int (*write)(const struct device *dev, enum motor_dir dir, uint32_t throttle);
	int (*read)(const struct device *dev, enum motor_dir *dir, uint32_t *throttle);
	int (*get_velocity)(const struct device *dev, int32_t *speed);
	int (*get_pos)(const struct device *dev, int32_t *pos);
};

static inline int motor_write(const struct device *dev, enum motor_dir dir, uint32_t throttle) {
    const struct motor_driver_api *api = (struct motor_driver_api*)dev->api;
    if (api->write)
		return api->write(dev, dir, throttle);
	return -ENOTSUP;
};

static inline int motor_read(const struct device *dev, enum motor_dir *dir, uint32_t *throttle) {
    const struct motor_driver_api *api = (struct motor_driver_api*)dev->api;
    if (api->read)
		return api->read(dev, dir, throttle);
	return -ENOTSUP;
};

static inline int motor_get_velocity(const struct device *dev, int32_t *vel) {
    const struct motor_driver_api *api = (struct motor_driver_api*)dev->api;
    if (api->get_velocity)
		return api->get_velocity(dev, vel);
	return -ENOTSUP;
};

static inline int motor_get_position(const struct device *dev, int32_t *pos) {
    const struct motor_driver_api *api = (struct motor_driver_api*)dev->api;
    if (api->get_pos)
		return api->get_pos(dev, pos);
	return -ENOTSUP;
};


#endif // ZEPHYR_MOTOR_H
