/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _servo_h_
#define _servo_h_

#include <stdint.h>

int  servo_init(void);
/* steer as signed µs delta from center: 0 = neutral, positive = CW */
int  servo_steer(int16_t delta_us);
int  servo_get(int16_t *delta_us);
/* Returns the valid delta range for servo id: neg_limit ≤ 0, pos_limit ≥ 0 */
int  servo_steer_delta_range(int id, int16_t *neg_limit, int16_t *pos_limit);
void servo_neutral_all(void);
/* center calibration — saved to settings, survives reboot */
int  servo_set_center(int id, uint16_t us);
int  servo_get_center(int id, uint16_t *us);
/* low-level µs access used by the shell */
int  servo_write_id(int id, uint16_t us);
int  servo_read_id(int id, uint16_t *us);

#endif
