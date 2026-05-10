/* SPDX-License-Identifier: Apache-2.0 */
/* adc_is_ready_dt() and adc_read_dt() added in Zephyr 3.5.0 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_COMPAT_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_COMPAT_H_

#include <version.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>

#if ZEPHYR_VERSION_CODE < ZEPHYR_VERSION(3, 5, 0)
static inline bool adc_is_ready_dt(const struct adc_dt_spec *spec)
{
	return device_is_ready(spec->dev);
}

static inline int adc_read_dt(const struct adc_dt_spec *spec,
			      struct adc_sequence *sequence)
{
	return adc_read(spec->dev, sequence);
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_COMPAT_H_ */
