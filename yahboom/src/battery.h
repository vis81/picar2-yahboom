/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _battery_h_
#define _battery_h_

int battery_init();
int battery_read(int32_t *mv, uint8_t *pct);

#endif
