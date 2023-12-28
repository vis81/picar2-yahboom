/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _motor_h_
#define _motor_h_

enum motor_id {
	MOTOR_L,
	MOTOR_R,
	MOTOR_LAST = MOTOR_R
};

int motor_init();
int motor_throttle(enum motor_id id, uint32_t dir, uint32_t throttle);
int motor_speed(enum motor_id id, int32_t speed);

#endif
