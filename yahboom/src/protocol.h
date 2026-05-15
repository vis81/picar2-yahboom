/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdint.h>
#include <zephyr/device.h>

#define PROTO_START   0xAA
#define PROTO_MAX_LEN 32

typedef void (*proto_rx_cb_t)(uint8_t type, const uint8_t *payload, uint8_t len);

struct proto_stats {
	uint32_t rx_frames;   /* valid frames dispatched to callback */
	uint32_t rx_crc_err;  /* frames dropped due to bad CRC */
	uint32_t rx_len_err;  /* frames dropped due to length > PROTO_MAX_LEN */
};

void proto_init(const struct device *uart, proto_rx_cb_t cb);
int  proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);
void proto_get_stats(struct proto_stats *out);
void proto_clear_stats(void);

#endif /* _PROTOCOL_H_ */
