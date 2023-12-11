/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
//#include <msgs/msgs.h>

#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

#define SBUS_FLAGS_FRAME_LOST 0x04
#define SBUS_FLAGS_FAILSAFE 0x08

struct sbus_stats {
	uint32_t rx_bytes;
	uint32_t rx_bytes_dropped;
    uint32_t rx_good;
    uint32_t rx_bad;
    uint32_t rx_discarded;
    uint64_t last_ts;
};

int sbus_read_stats(const struct device *dev, struct sbus_stats *stats);

#endif // ZEPHLY_RC_H
