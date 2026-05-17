/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include "protocol.h"
#include "comms.h"
#include "comms_mon.h"

static inline int64_t decode_le64(const uint8_t *p)
{
	return (int64_t)((uint64_t)sys_get_le32(p) |
			 ((uint64_t)sys_get_le32(p + 4) << 32));
}

static const struct shell *mon_shell;

/* Filter bits — default: show everything */
#define MON_FILT_CMD_VEL   BIT(0)
#define MON_FILT_REQ       BIT(1)
#define MON_FILT_SET_RATE  BIT(2)
#define MON_FILT_GET_STATS BIT(3)
#define MON_FILT_TIMESYNC  BIT(4)
#define MON_FILT_UNKNOWN   BIT(5)
#define MON_FILT_ERR       BIT(6)
#define MON_FILT_ALL       0x7Fu
static uint32_t mon_filter = MON_FILT_ALL;

static void comms_mon_err(uint8_t err, uint8_t msg_type, uint8_t detail)
{
	const struct shell *sh = mon_shell;

	if (!sh || !(mon_filter & MON_FILT_ERR)) {
		return;
	}
	int64_t ms = k_uptime_get();

	if (err == PROTO_ERR_CRC) {
		shell_print(sh, "[%8lld] !CRC_ERR  type=0x%02x",
			(long long)ms, msg_type);
	} else {
		shell_print(sh, "[%8lld] !LEN_ERR  type=0x%02x bad_len=%u",
			(long long)ms, msg_type, detail);
	}
}

void comms_mon_rx(uint8_t type, const uint8_t *payload, uint8_t len)
{
	const struct shell *sh = mon_shell;

	if (!sh) {
		return;
	}

	uint32_t fbit;

	switch (type) {
	case MSG_CMD_VEL:   fbit = MON_FILT_CMD_VEL;   break;
	case MSG_REQ:       fbit = MON_FILT_REQ;        break;
	case MSG_SET_RATE:  fbit = MON_FILT_SET_RATE;   break;
	case MSG_GET_STATS: fbit = MON_FILT_GET_STATS;  break;
	case MSG_TIMESYNC:  fbit = MON_FILT_TIMESYNC;   break;
	default:            fbit = MON_FILT_UNKNOWN;     break;
	}
	if (!(mon_filter & fbit)) {
		return;
	}

	int64_t ms = k_uptime_get();

	switch (type) {
	case MSG_CMD_VEL:
		if (len >= 5) {
			shell_print(sh, "[%8lld] CMD_VEL  l=%-6d r=%-6d steer=%u",
				(long long)ms,
				(int)(int16_t)sys_get_le16(&payload[0]),
				(int)(int16_t)sys_get_le16(&payload[2]),
				payload[4]);
		}
		break;
	case MSG_REQ:
		if (len >= 1) {
			shell_print(sh, "[%8lld] REQ      stream=0x%02x",
				(long long)ms, payload[0]);
		}
		break;
	case MSG_SET_RATE:
		if (len >= 3) {
			shell_print(sh, "[%8lld] SET_RATE stream=0x%02x hz=%u",
				(long long)ms, payload[0], sys_get_le16(&payload[1]));
		}
		break;
	case MSG_GET_STATS:
		shell_print(sh, "[%8lld] GET_STATS reset=%u",
			(long long)ms, len >= 1 ? payload[0] : 0u);
		break;
	case MSG_TIMESYNC:
		if (len >= 16) {
			shell_print(sh, "[%8lld] TIMESYNC  t1=%lld t4_prev=%lld",
				(long long)ms,
				(long long)decode_le64(&payload[0]),
				(long long)decode_le64(&payload[8]));
		}
		break;
	default:
		shell_print(sh, "[%8lld] UNKNOWN  type=0x%02x len=%u",
			(long long)ms, type, len);
		break;
	}
}

static const struct { const char *name; uint32_t bit; } filter_map[] = {
	{ "cmd_vel",   MON_FILT_CMD_VEL   },
	{ "req",       MON_FILT_REQ       },
	{ "set_rate",  MON_FILT_SET_RATE  },
	{ "get_stats", MON_FILT_GET_STATS },
	{ "timesync",  MON_FILT_TIMESYNC  },
	{ "unknown",   MON_FILT_UNKNOWN   },
	{ "err",       MON_FILT_ERR       },
};

static int cmd_mon_filter(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		char buf[64] = "";
		const char *sep = "";

		for (size_t i = 0; i < ARRAY_SIZE(filter_map); i++) {
			if (mon_filter & filter_map[i].bit) {
				strncat(buf, sep, sizeof(buf) - strlen(buf) - 1);
				strncat(buf, filter_map[i].name,
					sizeof(buf) - strlen(buf) - 1);
				sep = ",";
			}
		}
		shell_print(sh, "filter=0x%02x (%s)", mon_filter,
			buf[0] ? buf : "none");
		return 0;
	}
	if (argc == 2 && strcmp(argv[1], "all") == 0) {
		mon_filter = MON_FILT_ALL;
		shell_print(sh, "filter=all");
		return 0;
	}

	uint32_t new_filter = 0;

	for (int a = 1; a < (int)argc; a++) {
		bool found = false;

		for (size_t i = 0; i < ARRAY_SIZE(filter_map); i++) {
			if (strcmp(argv[a], filter_map[i].name) == 0) {
				new_filter |= filter_map[i].bit;
				found = true;
				break;
			}
		}
		if (!found) {
			shell_error(sh,
				"unknown: %s  valid: cmd_vel req set_rate get_stats timesync unknown err all",
				argv[a]);
			return -EINVAL;
		}
	}
	mon_filter = new_filter;
	shell_print(sh, "filter=0x%02x", mon_filter);
	return 0;
}

static int cmd_comms_mon(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (mon_shell) {
		mon_shell = NULL;
		proto_set_err_cb(NULL);
		shell_print(sh, "monitor off");
	} else {
		mon_shell = sh;
		proto_set_err_cb(comms_mon_err);
		shell_print(sh, "monitor on");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mon,
	SHELL_CMD_ARG(filter, NULL,
		"Show/set filter [all|cmd_vel|req|set_rate|get_stats|timesync|unknown|err]",
		cmd_mon_filter, 1, 7),
	SHELL_SUBCMD_SET_END
);

SHELL_SUBCMD_ADD((comms), mon, &sub_mon,
		 "Toggle Pi\xe2\x86\x92STM32 frame monitor", cmd_comms_mon, 0, 0);
