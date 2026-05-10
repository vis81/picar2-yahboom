/* SPDX-License-Identifier: Apache-2.0 */
/* pwm_is_ready_dt() added in Zephyr 3.5.0 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_COMPAT_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM_COMPAT_H_

#include <version.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>

#if ZEPHYR_VERSION_CODE < ZEPHYR_VERSION(3, 5, 0)
static inline bool pwm_is_ready_dt(const struct pwm_dt_spec *spec)
{
	return device_is_ready(spec->dev);
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PWM_COMPAT_H_ */
