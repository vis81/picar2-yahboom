/*
 * Copyright (c) 2022 Jonathan Hahn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
//#include <msgs/msgs.h>

#ifndef ZEPHLY_RC_H
#define ZEPHLY_RC_H

#define SBUS_FLAGS_FRAME_LOST 0x04
#define SBUS_FLAGS_FAILSAFE 0x08

struct rc_stats {
	uint32_t rx_bytes;
	uint32_t rx_bytes_dropped;
    uint32_t rx_good;
    uint32_t rx_bad;
    uint32_t rx_discarded;
    uint64_t last_ts;
};

struct rc_api {
	int (*read_flags)(const struct device *dev, uint8_t* value, uint64_t* ts);
	int (*read_channel)(const struct device *dev, uint8_t channel, uint16_t* value, uint64_t* ts);
	int (*read_all)(const struct device *dev, uint8_t num_channels, uint16_t* values, uint8_t* flags, uint64_t* ts);
	int (*read_stats)(const struct device *dev, struct rc_stats *stats);
};

static inline int rc_read_stats(const struct device *dev, struct rc_stats *stats) {
    const struct rc_api *api = (struct rc_api*)dev->api;
    if (api->read_stats)
		return api->read_stats(dev, stats);
	return -ENOTSUP;
}

static inline int rc_read_channel(const struct device *dev, uint8_t channel, uint16_t* value, uint64_t* ts) {
    const struct rc_api *api = (struct rc_api*)dev->api;
    if (api->read_channel)
		return api->read_channel(dev, channel, value, ts);
	return -ENOTSUP;
}

static inline int rc_read_flags(const struct device *dev, uint8_t* value, uint64_t* ts) {
    const struct rc_api *api = (struct rc_api*)dev->api;
    if (api->read_flags)
		return api->read_flags(dev, value, ts);
	return -ENOTSUP;
}

static inline int rc_read_all(const struct device *dev, uint8_t num_channels, uint16_t* values, uint8_t* flags, uint64_t* ts) {
	const struct rc_api *api = (struct rc_api*)dev->api;
    if (api->read_all)
		return api->read_all(dev, num_channels, values, flags, ts);
	return -ENOTSUP;
}

#endif // ZEPHLY_RC_H
