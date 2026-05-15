/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include "protocol.h"

RING_BUF_DECLARE(rx_ring, 256);

static const struct device *proto_uart;
static proto_rx_cb_t proto_cb;
static struct proto_stats stats;

static void rx_work_fn(struct k_work *w);
static K_WORK_DEFINE(rx_work, rx_work_fn);

static uint8_t crc8(const uint8_t *buf, size_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; i++) {
			crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
		}
	}
	return crc;
}

int proto_encode(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
	out[0] = PROTO_START;
	out[1] = type;
	out[2] = len;
	if (len > 0) {
		memcpy(&out[3], payload, len);
	}
	out[3 + len] = crc8(&out[1], 2 + len);
	return 4 + len;
}

static void process_rx_byte(uint8_t b)
{
	static enum { S_START, S_TYPE, S_LEN, S_PAYLOAD, S_CRC } state = S_START;
	static uint8_t rx_type, rx_len, rx_pos;
	static uint8_t rx_buf[PROTO_MAX_LEN];

	switch (state) {
	case S_START:
		if (b == PROTO_START) {
			state = S_TYPE;
		}
		break;
	case S_TYPE:
		rx_type = b;
		state = S_LEN;
		break;
	case S_LEN:
		if (b > PROTO_MAX_LEN) {
			stats.rx_len_err++;
			state = S_START;
			break;
		}
		rx_len = b;
		rx_pos = 0;
		state = (b > 0) ? S_PAYLOAD : S_CRC;
		break;
	case S_PAYLOAD:
		rx_buf[rx_pos++] = b;
		if (rx_pos == rx_len) {
			state = S_CRC;
		}
		break;
	case S_CRC: {
		uint8_t crc_data[2 + PROTO_MAX_LEN];

		crc_data[0] = rx_type;
		crc_data[1] = rx_len;
		memcpy(&crc_data[2], rx_buf, rx_len);
		if (b == crc8(crc_data, 2 + rx_len)) {
			stats.rx_frames++;
			proto_cb(rx_type, rx_buf, rx_len);
		} else {
			stats.rx_crc_err++;
		}
		state = S_START;
		break;
	}
	}
}

static void rx_work_fn(struct k_work *w)
{
	uint8_t b;

	while (ring_buf_get(&rx_ring, &b, 1) == 1) {
		process_rx_byte(b);
	}
}

static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t buf[16];
	int n;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		n = uart_fifo_read(dev, buf, sizeof(buf));
		if (n > 0) {
			ring_buf_put(&rx_ring, buf, n);
			k_work_submit(&rx_work);
		}
	}
}

void proto_get_stats(struct proto_stats *out)
{
	*out = stats;
}

void proto_clear_stats(void)
{
	memset(&stats, 0, sizeof(stats));
}

void proto_init(const struct device *uart, proto_rx_cb_t cb)
{
	proto_uart = uart;
	proto_cb = cb;
	uart_irq_callback_user_data_set(uart, uart_isr, NULL);
	uart_irq_rx_enable(uart);
}
