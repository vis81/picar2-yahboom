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
#define MSG_CMD_VEL       0x80
#define MSG_REQ           0x81
#define MSG_SET_RATE      0x82
#define MSG_GET_STATS     0x83
#define MSG_TIMESYNC      0x84

/* STM32 → Pi timesync response */
#define MSG_TIMESYNC_RESP 0x05

static inline int64_t decode_le64(const uint8_t *p)
{
	return (int64_t)((uint64_t)sys_get_le32(p) |
			 ((uint64_t)sys_get_le32(p + 4) << 32));
}

static int64_t get_uptime_us(void)
{
	return (int64_t)(k_cycle_get_64() / 72u);
}

static const struct device *pi_uart;
static bool connected;
static uint8_t js_seq;

static struct k_timer stream_tmr[STREAM_MAX];
static struct k_work  stream_wrk[STREAM_MAX];

static uint32_t tx_frames[STREAM_MAX];
static uint32_t rx_unknown;
static uint32_t rx_short;

/* Timesync state */
static uint32_t ts_count;
static bool     ts_offset_valid;
static int64_t  ts_offset_us;
static int64_t  ts_t1_prev;
static int64_t  ts_t2_prev;
static int64_t  ts_last_rx_ms;
static int64_t  ts_offsets[8];
static int      ts_hist_idx;
static bool     ts_last_raw_valid;
static int64_t  ts_last_raw;
static int64_t  ts_diffs[8];
static int      ts_diff_idx;
static int      ts_diff_count;

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

static int64_t ts_median(void)
{
	int64_t tmp[8];

	for (int i = 0; i < 8; i++) {
		tmp[i] = ts_offsets[i];
	}
	for (int i = 1; i < 8; i++) {
		int64_t key = tmp[i];
		int j = i - 1;

		while (j >= 0 && tmp[j] > key) {
			tmp[j + 1] = tmp[j];
			j--;
		}
		tmp[j + 1] = key;
	}
	return tmp[4];
}

static void send_timesync_resp(int64_t t2)
{
	uint8_t payload[8];

	sys_put_le32((uint32_t)((uint64_t)t2 & 0xFFFFFFFFu), &payload[0]);
	sys_put_le32((uint32_t)((uint64_t)t2 >> 32),         &payload[4]);
	send_frame(MSG_TIMESYNC_RESP, payload, 8);
}

static void send_joint_state(void)
{
	int32_t el = 0, er = 0, vl = 0, vr = 0;
	uint8_t steer = 0;
	uint8_t payload[22];

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

	int64_t pi_time = ts_offset_valid ? get_uptime_us() + ts_offset_us : 0LL;

	sys_put_le32((uint32_t)((uint64_t)pi_time & 0xFFFFFFFFu), &payload[14]);
	sys_put_le32((uint32_t)((uint64_t)pi_time >> 32),         &payload[18]);

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
	case MSG_TIMESYNC:
		if (len >= 16) {
			shell_print(sh, "TIMESYNC  t1=%lld t4_prev=%lld",
				(long long)decode_le64(&payload[0]),
				(long long)decode_le64(&payload[8]));
		}
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

	case MSG_TIMESYNC: {
		if (len < 16) {
			rx_short++;
			break;
		}
		int64_t t2      = get_uptime_us();
		int64_t t1      = decode_le64(&payload[0]);
		int64_t t4_prev = decode_le64(&payload[8]);

		ts_last_rx_ms = k_uptime_get();
		ts_count++;

		if (t4_prev != 0 && ts_t1_prev != 0) {
			/* offset = Pi_midtime - STM32_receive, so pi_time = stm32_us + offset */
			int64_t raw = (ts_t1_prev + t4_prev) / 2 - ts_t2_prev;

			if (!ts_offset_valid) {
				/* First valid sample: fill all slots to prevent zero contamination */
				for (int i = 0; i < 8; i++) {
					ts_offsets[i] = raw;
				}
				ts_hist_idx = 1;
			} else {
				ts_offsets[ts_hist_idx++ % 8] = raw;
			}
			ts_offset_us    = ts_median();
			ts_offset_valid = true;

			if (ts_last_raw_valid) {
				int64_t diff = raw - ts_last_raw;

				if (diff < 0) diff = -diff;
				ts_diffs[ts_diff_idx++ % 8] = diff;
				if (ts_diff_count < 8) ts_diff_count++;
			}
			ts_last_raw       = raw;
			ts_last_raw_valid = true;
		}
		ts_t1_prev = t1;
		ts_t2_prev = t2;
		send_timesync_resp(t2);
		break;
	}

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

static int cmd_ts_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (ts_offset_valid) {
		int64_t mn = ts_offsets[0], mx = ts_offsets[0];

		for (int i = 1; i < 8; i++) {
			if (ts_offsets[i] < mn) mn = ts_offsets[i];
			if (ts_offsets[i] > mx) mx = ts_offsets[i];
		}
		int64_t avg_diff = 0;

		if (ts_diff_count > 0) {
			for (int i = 0; i < ts_diff_count; i++) avg_diff += ts_diffs[i];
			avg_diff /= ts_diff_count;
		}
		shell_print(sh,
			"ts: count=%u  valid=yes  offset=%lld us"
			"  jitter=%lld us  avg_diff=%lld us  age=%lld ms",
			ts_count,
			(long long)ts_offset_us,
			(long long)(mx - mn),
			(long long)avg_diff,
			(long long)(k_uptime_get() - ts_last_rx_ms));
	} else {
		shell_print(sh, "ts: count=%u  valid=no  offset=\xe2\x80\x94", ts_count);
	}
	return 0;
}

static int cmd_ts_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ts_count          = 0;
	ts_offset_valid   = false;
	ts_offset_us      = 0;
	ts_t1_prev        = 0;
	ts_t2_prev        = 0;
	ts_last_rx_ms     = 0;
	ts_hist_idx       = 0;
	ts_last_raw_valid = false;
	ts_last_raw       = 0;
	ts_diff_idx       = 0;
	ts_diff_count     = 0;
	for (int i = 0; i < 8; i++) {
		ts_offsets[i] = 0LL;
		ts_diffs[i]   = 0LL;
	}
	shell_print(sh, "cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ts,
	SHELL_CMD(status, NULL, "Show timesync stats", cmd_ts_status),
	SHELL_CMD(clear,  NULL, "Clear timesync state", cmd_ts_clear),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ts, &sub_ts, "Timesync commands", NULL);

#endif /* CONFIG_SHELL */
