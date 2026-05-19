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
	MOTOR_LAST = MOTOR_R,
	MOTOR_CNT
};

int motor_init();
int motor_throttle(enum motor_id id, uint32_t dir, uint32_t throttle);
int motor_speed(enum motor_id id, int32_t speed);
int motor_pos(enum motor_id id, int32_t *pos);
int motor_vel(enum motor_id id, int32_t *vel);
int motor_set_pid_gains(enum motor_id id, int32_t kp, int32_t kd, int32_t ki);
void motor_stop_all(void);

#endif
