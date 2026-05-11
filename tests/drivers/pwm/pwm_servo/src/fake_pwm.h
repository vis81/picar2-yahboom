/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef FAKE_PWM_H_
#define FAKE_PWM_H_

#include <zephyr/device.h>

/*
 * Returns the pulse width (in nanoseconds) from the last successful
 * pwm_set_pulse_dt() call.  Valid because get_cycles_per_sec == NSEC_PER_SEC,
 * so stored cycles == nanoseconds numerically.
 */
uint32_t fake_pwm_last_pulse_ns(const struct device *dev);

/* Arm the fake to return err on the next set_cycles call (one-shot). */
void fake_pwm_inject_err(const struct device *dev, int err);

#endif /* FAKE_PWM_H_ */
