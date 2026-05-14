/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/input/sbusreceiver.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT sbus_receiver

LOG_MODULE_REGISTER(sbus, CONFIG_INPUT_LOG_LEVEL);

#define SBUS_RECV_BUFFER_SIZE      25
#define SBUS_START_BYTE            0x0F
#define SBUS_END_BYTE              0x00
#define SBUS_SERVO_CHANNEL_BIT_LEN 11
#define SBUS_NUM_SERVO_CHANNELS    16
#define SBUS_FLAGS_OFFSET          (SBUS_RECV_BUFFER_SIZE - 2)

struct sbus_buffer {
	uint64_t ts;
	uint8_t data[SBUS_RECV_BUFFER_SIZE];
	uint8_t pointer;
	uint8_t locked;
};

struct sbus_data {
	const struct device *dev;
	struct k_work work;
	struct sbus_buffer buf[2];
	uint8_t cur_buf;
	struct sbus_stats stats;
};

struct sbus_chan_config {
	uint32_t channel;
	uint32_t axis;
};

struct sbus_config {
	const struct device *uart;
	const struct sbus_chan_config *channels;
	uint8_t num_channels;
	bool continue_on_frame_lost;
	bool continue_on_failsafe;
};

static void sbus_handle_rx(const struct device *dev)
{
	struct sbus_data *data = dev->data;
	const struct sbus_config *config = dev->config;
	struct sbus_stats *stats = &data->stats;
	struct sbus_buffer *buf = &data->buf[data->cur_buf];
	uint8_t tmp;

	while (uart_fifo_read(config->uart, &tmp, 1) == 1) {
		stats->rx_bytes++;
		if (buf->pointer == 0) {
			if (tmp == SBUS_START_BYTE && buf->data[0] == SBUS_END_BYTE) {
				LOG_DBG("sbus:start");
				buf->pointer++;
			} else {
				stats->rx_bytes_dropped++;
			}
			buf->data[0] = tmp;
		} else if (buf->pointer == SBUS_RECV_BUFFER_SIZE - 1) {
			if (tmp == SBUS_END_BYTE) {
				LOG_DBG("sbus:end");
				stats->rx_good++;
				struct sbus_buffer *buf_next = &data->buf[!data->cur_buf];
				if (!buf_next->locked) {
					buf->ts = k_uptime_get();
					stats->last_ts = buf->ts;
					data->cur_buf = !data->cur_buf;
					buf = buf_next;
					buf->pointer = 0;
					k_work_submit(&data->work);
				} else {
					stats->rx_discarded++;
					buf->pointer = 0;
				}
			} else {
				/* bad end byte: reset */
				LOG_WRN("sbus:reset at %d byte: 0x%02x", buf->pointer, tmp);
				LOG_HEXDUMP_DBG(buf->data, SBUS_RECV_BUFFER_SIZE, "data:");
				buf->pointer = 0;
				stats->rx_bad++;
				stats->rx_bytes_dropped += SBUS_RECV_BUFFER_SIZE;
			}
		} else {
			buf->data[buf->pointer++] = tmp;
		}
	}
}

static void sbus_uart_cb(const struct device *uart, void *dev_ptr)
{
	uart_irq_update(uart);

	while (uart_irq_rx_ready(uart)) {
		sbus_handle_rx((const struct device *)dev_ptr);
	}
}

static inline struct sbus_buffer *lock_buffer(struct sbus_data *data)
{
	uint32_t key = irq_lock();
	struct sbus_buffer *rbuf = &data->buf[!data->cur_buf];

	rbuf->locked = 1;
	irq_unlock(key);
	return rbuf;
}

static inline void unlock_buffer(struct sbus_buffer *buf)
{
	buf->locked = 0;
}

static uint16_t sbus_get_channel(uint8_t *data, uint8_t channel)
{
	uint8_t bit_offset, byte_offset;
	int8_t bits_remaining;
	uint16_t res = 0;

	bit_offset = 8 + SBUS_SERVO_CHANNEL_BIT_LEN * channel;
	byte_offset = bit_offset / 8;
	bit_offset %= 8;
	bits_remaining = SBUS_SERVO_CHANNEL_BIT_LEN;
	while (bits_remaining > 0) {
		uint8_t val = data[byte_offset] >> bit_offset;

		val &= ((1 << bits_remaining) - 1);
		res |= val << (SBUS_SERVO_CHANNEL_BIT_LEN - bits_remaining);
		bits_remaining -= (8 - bit_offset);
		byte_offset++;
		bit_offset = 0;
	}
	return res;
}

int sbus_read_stats(const struct device *dev, struct sbus_stats *stats)
{
	if (!dev)
		return -EINVAL;
	struct sbus_data *data = dev->data;

	*stats = data->stats;
	return 0;
}

static void sbus_work_handler(struct k_work *work)
{
	struct sbus_data *data = CONTAINER_OF(work, struct sbus_data, work);
	const struct sbus_config *config = data->dev->config;
	struct sbus_buffer *buf;
	uint16_t values[SBUS_NUM_SERVO_CHANNELS];
	uint16_t flags;

	buf = lock_buffer(data);
	flags = buf->data[SBUS_FLAGS_OFFSET];
	if (buf->ts == 0
		|| (flags & SBUS_FLAGS_FRAME_LOST && !config->continue_on_frame_lost)
		|| (flags & SBUS_FLAGS_FAILSAFE && !config->continue_on_failsafe)) {
		unlock_buffer(buf);
		return;
	}

	for (int i = 0; i < config->num_channels; i++)
		values[i] = sbus_get_channel(buf->data, config->channels[i].channel - 1);

	unlock_buffer(buf);

	for (int i = 0; i < config->num_channels; i++) {
		bool last = (i == config->num_channels - 1);
		int ret = input_report_abs(data->dev, config->channels[i].axis,
					   values[i], last, K_FOREVER);
		if (ret)
			LOG_WRN("input_report_abs err %d", ret);
	}
}

static int sbus_receiver_init(const struct device *dev)
{
	struct sbus_data *data = dev->data;
	const struct sbus_config *config = dev->config;

	if (!device_is_ready(config->uart)) {
		LOG_ERR("UART not ready.");
		return -EIO;
	}
	memset(&data->buf, 0, sizeof(data->buf));
	memset(&data->stats, 0, sizeof(data->stats));

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	struct uart_config cfg;
	int ret = uart_config_get(config->uart, &cfg);

	if (ret != 0)
		return ret;

	cfg.stop_bits = UART_CFG_STOP_BITS_2;
	cfg.data_bits = UART_CFG_DATA_BITS_8;
	cfg.parity    = UART_CFG_PARITY_EVEN;

	ret = uart_configure(config->uart, &cfg);
	if (ret != 0)
		return ret;
#endif
	data->dev = dev;
	k_work_init(&data->work, sbus_work_handler);

	uart_irq_rx_disable(config->uart);
	uart_irq_tx_disable(config->uart);
	uart_irq_callback_user_data_set(config->uart, sbus_uart_cb, (void *)dev);
	uart_irq_rx_enable(config->uart);

	return 0;
}

#define SBUS_CHAN_CFG_DEF(node_id)                           \
	{                                                    \
		.channel = DT_PROP(node_id, channel),        \
		.axis    = DT_PROP(node_id, zephyr_axis),    \
	}

#define SBUS_RC_INIT(i)                                                            \
	static struct sbus_data sbus_data_##i = {};                                \
	static const struct sbus_chan_config sbus_chan_config_##i[] = {             \
		DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(i, SBUS_CHAN_CFG_DEF, (,))}; \
	static struct sbus_config sbus_config_##i = {                              \
		.uart                  = DEVICE_DT_GET(DT_INST_BUS(i)),            \
		.num_channels          = ARRAY_SIZE(sbus_chan_config_##i),          \
		.channels              = sbus_chan_config_##i,                      \
		.continue_on_frame_lost = DT_INST_PROP(i, continue_on_frame_lost), \
		.continue_on_failsafe  = DT_INST_PROP(i, continue_on_failsafe),    \
	};                                                                         \
	DEVICE_DT_INST_DEFINE(i, &sbus_receiver_init, NULL,                        \
			      &sbus_data_##i, &sbus_config_##i,                    \
			      POST_KERNEL, CONFIG_RC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(SBUS_RC_INIT)
