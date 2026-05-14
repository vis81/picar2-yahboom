/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/serial/uart_emul.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define SBUS_FRAME_LEN 25
#define SBUS_START     0x0F
#define SBUS_END       0x00

#define SBUS_FLAG_FRAME_LOST 0x04
#define SBUS_FLAG_FAILSAFE   0x08

static const struct device *emul_uart = DEVICE_DT_GET(DT_NODELABEL(euart0));

static K_SEM_DEFINE(event_sem, 0, 1);

static int last_axis;
static int last_value;

static void input_cb(struct input_event *evt)
{
	if (evt->type == INPUT_EV_ABS) {
		last_axis  = evt->code;
		last_value = evt->value;
		if (evt->sync) {
			k_sem_give(&event_sem);
		}
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);

static void build_frame(uint8_t *frame, const uint16_t *ch, uint8_t flags)
{
	memset(frame, 0, SBUS_FRAME_LEN);
	frame[0]  = SBUS_START;
	frame[24] = SBUS_END;
	frame[23] = flags;

	/* 16 channels × 11 bits packed into bytes 1..22, LSB first */
	for (int c = 0; c < 16; c++) {
		uint32_t val = ch[c] & 0x7FF;

		for (int b = 0; b < 11; b++) {
			if (val & (1u << b)) {
				uint32_t abs_bit = 8 + c * 11 + b;

				frame[abs_bit / 8] |= (1u << (abs_bit % 8));
			}
		}
	}
}

/*
 * The SBUS driver requires buf->data[0] == 0x00 (the SBUS end byte) before
 * it accepts a start byte.  Prepend one 0x00 byte so the start condition is
 * met regardless of what the previous test left in the buffer.
 */
static int push_frame_wait(const uint8_t *frame)
{
	uint8_t buf[1 + SBUS_FRAME_LEN];

	buf[0] = 0x00;
	memcpy(buf + 1, frame, SBUS_FRAME_LEN);

	k_sem_reset(&event_sem);
	uart_emul_put_rx_data(emul_uart, buf, sizeof(buf));
	return k_sem_take(&event_sem, K_MSEC(1000));
}

ZTEST_SUITE(sbus_receiver, NULL, NULL, NULL, NULL, NULL);

ZTEST(sbus_receiver, test_channel_min)
{
	uint16_t ch[16] = {0};
	uint8_t frame[SBUS_FRAME_LEN];

	build_frame(frame, ch, 0);

	zassert_ok(push_frame_wait(frame), "no input event for min values");
	zassert_equal(last_axis, 1, "wrong axis");
	zassert_equal(last_value, 0, "expected ch value 0");
}

ZTEST(sbus_receiver, test_channel_max)
{
	uint16_t ch[16] = {0};
	uint8_t frame[SBUS_FRAME_LEN];

	ch[0] = 0x7FF;
	ch[1] = 0x7FF;
	build_frame(frame, ch, 0);

	zassert_ok(push_frame_wait(frame), "no input event for max values");
	zassert_equal(last_axis, 1, "wrong axis");
	zassert_equal(last_value, 0x7FF, "expected ch value 0x7FF");
}

ZTEST(sbus_receiver, test_channel_mid)
{
	uint16_t ch[16] = {0};
	uint8_t frame[SBUS_FRAME_LEN];

	ch[0] = 992;
	ch[1] = 992;
	build_frame(frame, ch, 0);

	zassert_ok(push_frame_wait(frame), "no input event for mid value");
	zassert_equal(last_value, 992, "expected ch value 992");
}

ZTEST(sbus_receiver, test_bad_end_byte_no_event)
{
	uint16_t ch[16] = {0};
	uint8_t frame[SBUS_FRAME_LEN];

	ch[0] = 100;
	build_frame(frame, ch, 0);
	frame[24] = 0xFF; /* corrupt end byte */

	k_sem_reset(&event_sem);
	uint8_t buf[1 + SBUS_FRAME_LEN];

	buf[0] = 0x00;
	memcpy(buf + 1, frame, SBUS_FRAME_LEN);
	uart_emul_put_rx_data(emul_uart, buf, sizeof(buf));
	zassert_equal(k_sem_take(&event_sem, K_MSEC(200)), -EAGAIN,
		      "event fired despite bad end byte");
}

ZTEST(sbus_receiver, test_frame_lost_no_event)
{
	uint16_t ch[16] = {0};
	uint8_t frame[SBUS_FRAME_LEN];

	build_frame(frame, ch, SBUS_FLAG_FRAME_LOST);

	k_sem_reset(&event_sem);
	uint8_t buf[1 + SBUS_FRAME_LEN];

	buf[0] = 0x00;
	memcpy(buf + 1, frame, SBUS_FRAME_LEN);
	uart_emul_put_rx_data(emul_uart, buf, sizeof(buf));
	zassert_equal(k_sem_take(&event_sem, K_MSEC(200)), -EAGAIN,
		      "event fired despite frame_lost flag");
}

ZTEST(sbus_receiver, test_failsafe_no_event)
{
	uint16_t ch[16] = {0};
	uint8_t frame[SBUS_FRAME_LEN];

	build_frame(frame, ch, SBUS_FLAG_FAILSAFE);

	k_sem_reset(&event_sem);
	uint8_t buf[1 + SBUS_FRAME_LEN];

	buf[0] = 0x00;
	memcpy(buf + 1, frame, SBUS_FRAME_LEN);
	uart_emul_put_rx_data(emul_uart, buf, sizeof(buf));
	zassert_equal(k_sem_take(&event_sem, K_MSEC(200)), -EAGAIN,
		      "event fired despite failsafe flag");
}
