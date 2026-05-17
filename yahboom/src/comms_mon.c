/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include "comms.h"
#include "comms_mon.h"

static inline int64_t decode_le64(const uint8_t *p)
{
	return (int64_t)((uint64_t)sys_get_le32(p) |
			 ((uint64_t)sys_get_le32(p + 4) << 32));
}

static const struct shell *mon_shell;

void comms_mon_rx(uint8_t type, const uint8_t *payload, uint8_t len)
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

static int cmd_comms_mon(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	if (mon_shell) {
		mon_shell = NULL;
		shell_print(sh, "monitor off");
	} else {
		mon_shell = sh;
		shell_print(sh, "monitor on");
	}
	return 0;
}

SHELL_SUBCMD_ADD((comms), mon, NULL, "Toggle Pi\xe2\x86\x92STM32 frame monitor",
		 cmd_comms_mon, 0, 0);
