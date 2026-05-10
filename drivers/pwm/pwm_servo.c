/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#define DT_DRV_COMPAT pwm_servo

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm_servo.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pwm_compat.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_servo, CONFIG_PWM_LOG_LEVEL);

// Simple functions for mapping a value betwen [0, 100] to the
// range [min, max] and vice versa.
#define MAP(value, min, max) ((value) * ((max) - (min))/100 + (min))
#define PAM(value, min, max) (((value) - (min)) * 100 / ((max) - (min)))

// Structs of this type need to kept in the global portion (static) of RAM
// (not const) because they are accessed by EasyDMA.
typedef struct
{
    bool ready;
} servo_group_t;

struct servo_data {
	uint8_t current_value;
	bool ready;
};

struct servo_cfg {
    uint32_t pin;
    struct pwm_dt_spec pwm_spec;
    uint8_t  pwm_index;
    uint8_t  pwm_channel;
    uint8_t  init_value;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
};

static inline int channel_get(const struct device *dev,
	                     uint8_t *p_value)
{
	struct servo_data *p_data = dev->data;
    *p_value = p_data->current_value;
    return 0;
}

static inline int channel_set(const struct device *dev,
	                     uint8_t value)
{
    const struct servo_cfg *p_cfg  = dev->config;
    struct servo_data      *p_data = dev->data;

	if (value > SERVO_MAX_VALUE)
		return -EINVAL;
	uint32_t val = MAP(value, PWM_USEC(p_cfg->min_pulse_us), PWM_USEC(p_cfg->max_pulse_us));
	p_data->current_value = value;
	LOG_DBG("Set pulse %u", val);
    return pwm_set_pulse_dt(&p_cfg->pwm_spec, val);
}


static int pwm_servo_init(const struct device *dev)
{
    int err;
    const struct servo_cfg *p_cfg  = dev->config;
    struct servo_data      *p_data = dev->data;

    if (unlikely(p_data->ready)) {
        /* Already initialized */
        return 0;
    }
    
    if (!pwm_is_ready_dt(&p_cfg->pwm_spec)) {
		printk("Error: PWM device %s is not ready\n", p_cfg->pwm_spec.dev->name);
		return 0;
	}

    err = channel_set(dev, p_cfg->init_value);
    if (0 != err) {
		printk("channel_set error%d\n", err);
    	goto ERR_EXIT;
    }

    p_data->ready = true;
	printk("INIT done\n");
    return 0;

ERR_EXIT:
    return -ENXIO;
}

static int pwm_servo_write(const struct device *dev, uint8_t value)
{
	struct servo_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = channel_set(dev, value);
    if (0 != err) {
    	return err;
    }
    return 0;
}

static int pwm_servo_read(const struct device *dev, uint8_t *value)
{
	struct servo_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = channel_get(dev, value);
    if (0 != err) {
    	return err;
    }
    return 0;
}

static const struct servo_driver_api servo_driver_api = {
    .write = pwm_servo_write,
    .read  = pwm_servo_read,
};

#define INST(num) DT_INST(num, pwm_servo)

#define SERVO_DEVICE(n) \
    static const struct servo_cfg servo_cfg_##n = { \
        .init_value   = DT_PROP(INST(n), init_value), \
        .min_pulse_us = DT_PROP(INST(n), min_pulse_us), \
        .max_pulse_us = DT_PROP(INST(n), max_pulse_us), \
        .pwm_spec     = PWM_DT_SPEC_GET(INST(n)) \
    }; \
    static struct servo_data servo_data_##n; \
    DEVICE_DT_DEFINE(INST(n), \
            pwm_servo_init, \
            NULL, \
            &servo_data_##n, \
            &servo_cfg_##n, \
            POST_KERNEL, \
            CONFIG_PWM_SERVO_INIT_PRIORITY, \
            &servo_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SERVO_DEVICE)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "PWM-Servo driver enabled without any devices"
#endif
