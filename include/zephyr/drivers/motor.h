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
};

static inline int motor_write(const struct device *dev, enum motor_dir dir, uint32_t throttle) {
    const struct motor_driver_api *api = (struct motor_driver_api*)dev->api;
    if (api->write)
		return api->write(dev, dir, throttle);
	return -ENOTSUP;
}

#endif // ZEPHYR_MOTOR_H
