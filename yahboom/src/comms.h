/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _COMMS_H_
#define _COMMS_H_

/* Pi → STM32 message types */
#define MSG_CMD_VEL   0x80
#define MSG_REQ       0x81
#define MSG_SET_RATE  0x82
#define MSG_GET_STATS 0x83
#define MSG_TIMESYNC  0x84

void comms_init(void);

#endif /* _COMMS_H_ */
