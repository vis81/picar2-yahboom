/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _servo_h_
#define _servo_h_

int servo_init();
int servo_steer(uint8_t val);
int servo_get(uint8_t *pval);

#endif
