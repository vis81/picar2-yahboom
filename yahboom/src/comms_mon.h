/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _COMMS_MON_H_
#define _COMMS_MON_H_

#include <stdint.h>

#ifdef CONFIG_COMMS_MON
void comms_mon_rx(uint8_t type, const uint8_t *payload, uint8_t len);
#else
static inline void comms_mon_rx(uint8_t type, const uint8_t *payload, uint8_t len)
{
	ARG_UNUSED(type);
	ARG_UNUSED(payload);
	ARG_UNUSED(len);
}
#endif

#endif /* _COMMS_MON_H_ */
