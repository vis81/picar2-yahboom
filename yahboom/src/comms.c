/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include "protocol.h"
#include "comms.h"
#include "motor.h"
#include "servo.h"
#include "imu.h"
#include "battery.h"
#include "rc.h"

/* STM32 → Pi stream IDs and message types */
#define STREAM_JOINT  0x01
#define STREAM_IMU    0x02
#define STREAM_BAT    0x03
#define STREAM_STATS  0x04
#define STREAM_MAX    4    /* valid IDs: 1..3; index 0 unused; STATS is one-shot */

/* Pi → STM32 message types */
#define MSG_CMD_VEL   0x80
#define MSG_REQ       0x81
#define MSG_SET_RATE  0x82
#define MSG_GET_STATS 0x83

static const struct device *pi_uart;
static bool connected;
static uint8_t js_seq;

static struct k_timer stream_tmr[STREAM_MAX];
static struct k_work  stream_wrk[STREAM_MAX];

static uint32_t tx_frames[STREAM_MAX];
static uint32_t rx_unknown;
static uint32_t rx_short;

static void on_timeout(struct k_work *w);
static K_WORK_DELAYABLE_DEFINE(watchdog_work, on_timeout);

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
	uint8_t frame[3 + PROTO_MAX_LEN + 1];
	int n = proto_encode(type, payload, len, frame);

	for (int i = 0; i < n; i++) {
		uart_poll_out(pi_uart, frame[i]);
	}
	if (type < STREAM_MAX) {
		tx_frames[type]++;
	}
}

static void send_joint_state(void)
{
	int32_t el = 0, er = 0, vl = 0, vr = 0;
	uint8_t steer = 0;
	uint8_t payload[14];

	motor_pos(MOTOR_L, &el);
	motor_pos(MOTOR_R, &er);
	motor_vel(MOTOR_L, &vl);
	motor_vel(MOTOR_R, &vr);
	servo_get(&steer);

	sys_put_le32((uint32_t)el, &payload[0]);
	sys_put_le32((uint32_t)er, &payload[4]);
	payload[8] = steer;
	payload[9] = js_seq++;
	sys_put_le16((uint16_t)(int16_t)vl, &payload[10]);
	sys_put_le16((uint16_t)(int16_t)vr, &payload[12]);

	send_frame(STREAM_JOINT, payload, sizeof(payload));
}

static void send_imu(void)
{
	struct imu_data d;
	uint8_t payload[20];
	if (imu_get_data(&d) != 0) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		sys_put_le16((uint16_t)d.accel[i], &payload[i * 2]);
		sys_put_le16((uint16_t)d.gyro[i],  &payload[6 + i * 2]);
		sys_put_le16((uint16_t)d.magn[i],  &payload[12 + i * 2]);
	}
	sys_put_le16((uint16_t)d.temp, &payload[18]);

	send_frame(STREAM_IMU, payload, sizeof(payload));
}

static void send_battery(void)
{
	int32_t mv = 0;
	uint8_t pct = 0;
	uint8_t payload[4];

	if (battery_read(&mv, &pct) != 0) {
		return;
	}

	sys_put_le16((uint16_t)mv, &payload[0]);
	payload[2] = pct;
	payload[3] = 0;

	send_frame(STREAM_BAT, payload, sizeof(payload));
}

static void send_stats(void)
{
	struct proto_stats ps;
	uint8_t payload[24];
	uint32_t tx_total = 0;

	proto_get_stats(&ps);
	for (int i = 1; i < STREAM_MAX; i++) {
		tx_total += tx_frames[i];
	}

	sys_put_le32(ps.rx_frames,  &payload[0]);
	sys_put_le32(ps.rx_crc_err, &payload[4]);
	sys_put_le32(ps.rx_len_err, &payload[8]);
	sys_put_le32(rx_short,      &payload[12]);
	sys_put_le32(rx_unknown,    &payload[16]);
	sys_put_le32(tx_total,      &payload[20]);

	send_frame(STREAM_STATS, payload, sizeof(payload));
}

static void send_stream(uint8_t id)
{
	switch (id) {
	case STREAM_JOINT: send_joint_state(); break;
	case STREAM_IMU:   send_imu();         break;
	case STREAM_BAT:   send_battery();     break;
	}
}

static void stream_work_fn(struct k_work *w)
{
	int id = (int)(w - stream_wrk);

	send_stream((uint8_t)id);
}

static void timer_expiry_fn(struct k_timer *t)
{
	int id = (int)(uintptr_t)k_timer_user_data_get(t);

	k_work_submit(&stream_wrk[id]);
}

static void on_timeout(struct k_work *w)
{
	connected = false;
	rc_set_enable(1);
}

#ifdef CONFIG_SHELL
static const struct shell *mon_shell;

static void comms_mon_print(uint8_t type, const uint8_t *payload, uint8_t len)
{
	const struct shell *sh = mon_shell;

	if (!sh) {
		return;
	}
	switch (type) {
	case MSG_CMD_VEL:
		if (len >= 5) {
			shell_print(sh, "CMD_VEL  l=%-6d r=%-6d steer=%u",
				(int)(int16_t)sys_get_le16(&payload[0]),
				(int)(int16_t)sys_get_le16(&payload[2]),
				payload[4]);
		}
		break;
	case MSG_REQ:
		if (len >= 1) {
			shell_print(sh, "REQ      stream=0x%02x", payload[0]);
		}
		break;
	case MSG_SET_RATE:
		if (len >= 3) {
			shell_print(sh, "SET_RATE stream=0x%02x hz=%u",
				payload[0], sys_get_le16(&payload[1]));
		}
		break;
	case MSG_GET_STATS:
		shell_print(sh, "GET_STATS reset=%u", len >= 1 ? payload[0] : 0u);
		break;
	default:
		shell_print(sh, "UNKNOWN  type=0x%02x len=%u", type, len);
		break;
	}
}
#endif /* CONFIG_SHELL */

static void comms_rx(uint8_t type, const uint8_t *payload, uint8_t len)
{
	k_work_reschedule(&watchdog_work, K_MSEC(500));
	if (!connected) {
		connected = true;
		rc_set_enable(0);
	}

#ifdef CONFIG_SHELL
	comms_mon_print(type, payload, len);
#endif

	switch (type) {
	case MSG_CMD_VEL:
		if (len < 5) {
			rx_short++;
			break;
		}
		motor_speed(MOTOR_L, (int32_t)(int16_t)sys_get_le16(&payload[0]));
		motor_speed(MOTOR_R, (int32_t)(int16_t)sys_get_le16(&payload[2]));
		servo_steer(payload[4]);
		break;

	case MSG_REQ:
		if (len < 1) {
			rx_short++;
			break;
		}
		send_stream(payload[0]);
		break;

	case MSG_SET_RATE: {
		if (len < 3) {
			rx_short++;
			break;
		}
		uint8_t id = payload[0];
		uint16_t hz = sys_get_le16(&payload[1]);

		if (id < 1 || id >= STREAM_MAX) {
			break;
		}
		if (hz == 0) {
			k_timer_stop(&stream_tmr[id]);
		} else {
			uint32_t period_ms = 1000u / hz;

			if (period_ms == 0) {
				period_ms = 1;
			}
			k_timer_start(&stream_tmr[id],
				      K_MSEC(period_ms), K_MSEC(period_ms));
		}
		break;
	}

	case MSG_GET_STATS:
		if (len >= 1 && payload[0]) {
			proto_clear_stats();
			rx_unknown = rx_short = 0;
			for (int i = 0; i < STREAM_MAX; i++) {
				tx_frames[i] = 0;
			}
		}
		send_stats();
		break;

	default:
		rx_unknown++;
		break;
	}
}

void comms_init(void)
{
	pi_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));

	for (int i = 1; i < STREAM_MAX; i++) {
		k_work_init(&stream_wrk[i], stream_work_fn);
		k_timer_init(&stream_tmr[i], timer_expiry_fn, NULL);
		k_timer_user_data_set(&stream_tmr[i], (void *)(uintptr_t)i);
	}

	proto_init(pi_uart, comms_rx);
}

#ifdef CONFIG_SHELL

static int cmd_comms_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct proto_stats ps;
	uint32_t tx_total = 0;

	proto_get_stats(&ps);
	for (int i = 1; i < STREAM_MAX; i++) {
		tx_total += tx_frames[i];
	}

	shell_print(sh, "rx frames   %u", ps.rx_frames);
	shell_print(sh, "rx crc_err  %u", ps.rx_crc_err);
	shell_print(sh, "rx len_err  %u", ps.rx_len_err);
	shell_print(sh, "rx short    %u", rx_short);
	shell_print(sh, "rx unknown  %u", rx_unknown);
	shell_print(sh, "tx frames   %u", tx_total);
	shell_print(sh, "connected   %s", connected ? "yes" : "no");
	return 0;
}

static int cmd_comms_stats_clear(const struct shell *sh, size_t argc, char **argv)
{
	proto_clear_stats();
	rx_unknown = rx_short = 0;
	for (int i = 0; i < STREAM_MAX; i++) {
		tx_frames[i] = 0;
	}
	shell_print(sh, "cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_comms_stats,
	SHELL_CMD(clear, NULL, "Clear protocol statistics", cmd_comms_stats_clear),
	SHELL_SUBCMD_SET_END
);

static int cmd_comms_baud(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "%u", proto_get_baud());
		return 0;
	}

	char *end;
	uint32_t baud = strtoul(argv[1], &end, 10);
	int ret;

	if (*end != '\0' || baud == 0) {
		shell_error(sh, "invalid baud rate");
		return -EINVAL;
	}

	ret = proto_set_baud(baud);
	if (ret) {
		shell_error(sh, "failed (%d)", ret);
		return ret;
	}

	shell_print(sh, "%u", baud);
	return 0;
}

static int cmd_comms_mon(const struct shell *sh, size_t argc, char **argv)
{
	if (mon_shell) {
		mon_shell = NULL;
		shell_print(sh, "monitor off");
	} else {
		mon_shell = sh;
		shell_print(sh, "monitor on");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_comms,
	SHELL_CMD(stats, &sub_comms_stats, "Print protocol statistics", cmd_comms_stats),
	SHELL_CMD_ARG(baud, NULL, "Get/set USART1 baud rate [rate]", cmd_comms_baud, 1, 1),
	SHELL_CMD(mon, NULL, "Toggle Pi→STM32 frame monitor", cmd_comms_mon),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(comms, &sub_comms, "Comms commands", NULL);

#endif /* CONFIG_SHELL */
