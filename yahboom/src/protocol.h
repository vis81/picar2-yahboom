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

void proto_init(const struct device *uart, proto_rx_cb_t cb);
int  proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);

#endif /* _PROTOCOL_H_ */
