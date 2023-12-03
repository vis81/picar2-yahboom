/*
* Copyright (c) 2022 Jonathan Hahn
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <string.h>

#include <zephyr/device.h>
#include "zephyr/devicetree.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/rc.h>
#include <zephyr/kernel.h>

#define DT_DRV_COMPAT futaba_sbusreceiver

#define SBUS_RECV_BUFFER_SIZE 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
#define SBUS_SERVO_CHANNEL_BIT_LEN 11
#define SBUS_NUM_SERVO_CHANNELS 16
#define SBUS_FLAGS_OFFSET SBUS_RECV_BUFFER_SIZE - 2

#define SBUS_RECV_BUFFER_CNT 2

//#define DEBUG(...) printk(__VA_ARGS__)
//#define DEBUG_DUMP(x,y) dump(x, y)

#define DEBUG(...)
#define DEBUG_DUMP(x,y)

struct sbus_buffer {
	uint64_t ts;
	uint8_t data[SBUS_RECV_BUFFER_SIZE];
	uint8_t pointer;
	uint8_t locked;
};

struct sbus_data {
	struct sbus_buffer buf[2];
	uint8_t cur_buf;
	struct rc_stats stats;
};

struct sbus_config {
    const struct device *uart;
};


void dump(uint8_t* buf, uint8_t len) {
	for (int i = 0; i < len; i++) {
		printk(" %02x", buf[i]);
	}
	printk("\n");
}

static void sbus_handle_rx(const struct device *dev) {
    struct sbus_data *data = dev->data;
    const struct sbus_config *config = dev->config;
    struct rc_stats *stats = &data->stats;
    struct sbus_buffer* buf = &data->buf[data->cur_buf];
	uint8_t tmp;

    while (uart_fifo_read(config->uart, &tmp, 1) == 1) {
		stats->rx_bytes++;
		if (buf->pointer == 0) {
			if (tmp == SBUS_START_BYTE) {
				DEBUG("sbus:start\n");
				buf->data[0] = tmp;
				buf->pointer++;
			} else {
				stats->rx_bytes_dropped++;
			}
		} else if (buf->pointer == SBUS_RECV_BUFFER_SIZE - 1) {
			if (tmp == SBUS_END_BYTE) {
				DEBUG("sbus:end\n");
				DEBUG_DUMP(buf, SBUS_RECV_BUFFER_SIZE);
				stats->rx_good++;
				struct sbus_buffer* buf_next = &data->buf[!data->cur_buf];
				if (!buf_next->locked) {
					data->cur_buf = !data->cur_buf;
					buf = buf_next;
					buf->pointer = 0;
					buf->ts = k_uptime_get();
					stats->last_ts = buf->ts;
				} else {
					stats->rx_discarded++;
					buf->pointer = 0;
				}
			} else {
				// reset buffer
				printk("sbus:reset\n");
				buf->pointer = 0;
				stats->rx_bad++;
				stats->rx_bytes_dropped += SBUS_RECV_BUFFER_SIZE;
			}
		} else {
			buf->data[buf->pointer++] = tmp;
		}
    }
}

static void sbus_uart_cb(const struct device *uart, void *dev_ptr) {
    uart_irq_update(uart);

    while (uart_irq_rx_ready(uart)) {
        sbus_handle_rx((const struct device*)dev_ptr);
    }
}

static int sbus_receiver_init(const struct device *dev) {
    struct sbus_data *data = dev->data;
    const struct sbus_config *config = dev->config;

    if (!device_is_ready(config->uart)) {
        printk("UART not ready.");
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
    cfg.parity = UART_CFG_PARITY_EVEN;
    
    ret = uart_configure(config->uart, &cfg);
    if (ret != 0)
        return ret;
#endif
    printk("Initializing SBUS Receiver!\n");

    uart_irq_rx_disable(config->uart);
	uart_irq_tx_disable(config->uart);
    uart_irq_callback_user_data_set(config->uart, sbus_uart_cb, (void*)dev);
    uart_irq_rx_enable(config->uart);

    return 0;
}

static inline struct sbus_buffer* lock_buffer(struct sbus_data *data) {
	uint32_t key = irq_lock();
	struct sbus_buffer* rbuf = &data->buf[!data->cur_buf];
	rbuf->locked = 1;
	irq_unlock(key);
	return rbuf;
}

static inline void unlock_buffer(struct sbus_buffer* buf){
	buf->locked = 0;
}

static uint16_t sbus_get_channel(uint8_t *data, uint8_t channel) {
	uint8_t bit_offset, byte_offset;
	int8_t bits_remaining;
	uint16_t res = 0;

	bit_offset = 8 + SBUS_SERVO_CHANNEL_BIT_LEN * channel;
	byte_offset = bit_offset / 8;
	bit_offset %= 8;
	bits_remaining = SBUS_SERVO_CHANNEL_BIT_LEN;
	while(bits_remaining > 0) {
		uint8_t val = data[byte_offset] >> bit_offset;
		val &= ((1 << bits_remaining) - 1);
		res |= val << (SBUS_SERVO_CHANNEL_BIT_LEN - bits_remaining);
		bits_remaining -= (8 - bit_offset);
		byte_offset++;
		bit_offset = 0;
	}
	return res;
}

static int sbus_read_channel(const struct device *dev, uint8_t channel, uint16_t* value, uint64_t* ts) {
	struct sbus_data *data = dev->data;
	struct sbus_buffer* buf;

	if (channel >= SBUS_NUM_SERVO_CHANNELS || !value)
		return -EINVAL;

	buf = lock_buffer(data);
	if (buf->ts == 0) {
		unlock_buffer(buf);
		return -ENODATA;
	}
	if(ts)
		*ts = buf->ts;
	*value = sbus_get_channel(buf->data, channel);
	unlock_buffer(buf);
	return 0;
}

static int sbus_read_flags(const struct device *dev, uint8_t* value, uint64_t* ts) {
	struct sbus_data *data = dev->data;
	struct sbus_buffer* buf;

	if (!value)
		return -EINVAL;
	buf = lock_buffer(data);
	if (buf->ts == 0) {
		unlock_buffer(buf);
		return -ENODATA;
	}
	if(ts)
		*ts = buf->ts;
	*value = buf->data[SBUS_FLAGS_OFFSET];
	unlock_buffer(buf);
	return 0;
}

static int sbus_read_all(const struct device *dev, uint8_t num_channels, uint16_t* values, uint8_t* flags, uint64_t* ts) {
	struct sbus_data *data = dev->data;
	struct sbus_buffer* buf;

	if (num_channels > SBUS_NUM_SERVO_CHANNELS)
		return -EINVAL;

	buf = lock_buffer(data);
	if (buf->ts == 0) {
		unlock_buffer(buf);
		return -ENODATA;
	}
	if (ts)
		*ts = buf->ts;
	for (int i = 0; i < num_channels; i++)
		values[i] = sbus_get_channel(buf->data, i);
	if (flags)
		*flags = buf->data[SBUS_FLAGS_OFFSET];
	unlock_buffer(buf);
	return 0;
}

int sbus_stats(const struct device *dev, struct rc_stats *stats)
{
	if (!dev)
		return -EINVAL;
	struct sbus_data *data = dev->data;
	*stats = data->stats;
	return 0;
}

struct rc_api sbus_api = {
    .read_stats = sbus_stats,
    .read_channel = sbus_read_channel,
    .read_all = sbus_read_all,
    .read_flags = sbus_read_flags,
};

#define SBUS_RC_INIT(i) \
    static struct sbus_data sbus_data_##i = {}; \
    static struct sbus_config sbus_config_##i = { \
        .uart = DEVICE_DT_GET(DT_INST_BUS(i)) \
    }; \
    DEVICE_DT_INST_DEFINE(i, &sbus_receiver_init, NULL, \
            &sbus_data_##i, &sbus_config_##i, \
            POST_KERNEL, RC_INIT_PRIORITY, \
            &sbus_api);

DT_INST_FOREACH_STATUS_OKAY(SBUS_RC_INIT)
