/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _servo_h_
#define _servo_h_

#include <stdint.h>

int  servo_init(void);
/* steer in tenths of degrees: 0 = center, +900 = 90° CW, -900 = 90° CCW */
int  servo_steer(int16_t tenths_deg);
int  servo_get(int16_t *tenths_deg);
void servo_neutral_all(void);
/* center calibration — saved to settings, survives reboot */
int  servo_set_center(int id, uint16_t us);
int  servo_get_center(int id, uint16_t *us);
/* low-level µs access used by the shell */
int  servo_write_id(int id, uint16_t us);
int  servo_read_id(int id, uint16_t *us);

#endif
