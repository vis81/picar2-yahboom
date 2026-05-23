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

/* Error types for proto_set_err_cb */
#define PROTO_ERR_CRC  0u  /* CRC mismatch; detail unused */
#define PROTO_ERR_LEN  1u  /* length > PROTO_MAX_LEN; detail = bad len byte */
typedef void (*proto_err_cb_t)(uint8_t err, uint8_t msg_type, uint8_t detail);
void proto_set_err_cb(proto_err_cb_t cb);

struct proto_stats {
	uint32_t rx_frames;      /* valid frames dispatched to callback */
	uint32_t rx_crc_err;     /* frames dropped due to bad CRC */
	uint32_t rx_len_err;     /* frames dropped due to length > PROTO_MAX_LEN */
	uint32_t rx_drop;        /* bytes lost to ring buffer overflow */
	uint32_t rx_dma_restart; /* unplanned RX DMA restarts (data gap) */
	uint32_t tx_abort;       /* TX DMA aborts */
};

void proto_init(const struct device *uart, proto_rx_cb_t cb);
void proto_tx(const uint8_t *buf, size_t len);
int  proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);
void proto_get_stats(struct proto_stats *out);
void proto_clear_stats(void);
uint32_t proto_get_baud(void);
int      proto_set_baud(uint32_t baud);

#endif /* _PROTOCOL_H_ */
