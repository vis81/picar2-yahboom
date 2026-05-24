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
static proto_err_cb_t proto_err_cb;
static struct proto_stats stats;

void proto_set_err_cb(proto_err_cb_t cb)
{
	proto_err_cb = cb;
}

static void rx_work_fn(struct k_work *w);
static K_WORK_DEFINE(rx_work, rx_work_fn);

#define DMA_BUF_SIZE       64
#define RX_IDLE_TIMEOUT_US 200

static uint8_t dma_rx_buf[2][DMA_BUF_SIZE];
static uint8_t dma_buf_idx;
static bool rx_paused;

#define TX_BUF_SIZE (4 + PROTO_MAX_LEN)
static uint8_t tx_dma_buf[TX_BUF_SIZE];
static K_SEM_DEFINE(tx_sem, 1, 1);

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
			if (proto_err_cb) {
				proto_err_cb(PROTO_ERR_LEN, rx_type, b);
			}
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
			if (proto_err_cb) {
				proto_err_cb(PROTO_ERR_CRC, rx_type, 0);
			}
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

static void uart_async_cb(const struct device *dev, struct uart_event *evt,
			  void *user_data)
{
	switch (evt->type) {
	case UART_RX_RDY: {
		uint32_t put = ring_buf_put(&rx_ring,
					    evt->data.rx.buf + evt->data.rx.offset,
					    evt->data.rx.len);
		if (put < evt->data.rx.len) {
			stats.rx_drop += evt->data.rx.len - put;
		}
		k_work_submit(&rx_work);
		break;
	}
	case UART_RX_BUF_REQUEST:
		dma_buf_idx ^= 1;
		uart_rx_buf_rsp(dev, dma_rx_buf[dma_buf_idx], DMA_BUF_SIZE);
		break;
	case UART_RX_BUF_RELEASED:
		break;
	case UART_RX_DISABLED:
		if (!rx_paused) {
			stats.rx_dma_restart++;
			dma_buf_idx = 0;
			uart_rx_enable(dev, dma_rx_buf[0], DMA_BUF_SIZE,
				       RX_IDLE_TIMEOUT_US);
		}
		break;
	case UART_TX_DONE:
		k_sem_give(&tx_sem);
		break;
	case UART_TX_ABORTED:
		stats.tx_abort++;
		k_sem_give(&tx_sem);
		break;
	default:
		break;
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

uint32_t proto_get_baud(void)
{
	struct uart_config cfg;

	if (uart_config_get(proto_uart, &cfg) == 0) {
		return cfg.baudrate;
	}
	return 0;
}

int proto_set_baud(uint32_t baud)
{
	struct uart_config cfg;
	int ret;

	ret = uart_config_get(proto_uart, &cfg);
	if (ret) {
		return ret;
	}

	rx_paused = true;
	uart_rx_disable(proto_uart);

	cfg.baudrate = baud;
	ret = uart_configure(proto_uart, &cfg);
	rx_paused = false;

	if (ret) {
		return ret;
	}

	dma_buf_idx = 0;
	return uart_rx_enable(proto_uart, dma_rx_buf[0], DMA_BUF_SIZE,
			      RX_IDLE_TIMEOUT_US);
}

void proto_tx(const uint8_t *buf, size_t len)
{
	k_sem_take(&tx_sem, K_FOREVER);
	memcpy(tx_dma_buf, buf, len);
	if (uart_tx(proto_uart, tx_dma_buf, len, SYS_FOREVER_US) != 0) {
		k_sem_give(&tx_sem);
	}
}

void proto_init(const struct device *uart, proto_rx_cb_t cb)
{
	proto_uart = uart;
	proto_cb = cb;
	dma_buf_idx = 0;
	uart_callback_set(uart, uart_async_cb, NULL);
	uart_rx_enable(uart, dma_rx_buf[0], DMA_BUF_SIZE, RX_IDLE_TIMEOUT_US);
}
