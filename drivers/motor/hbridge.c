/*
 * Copyright (c) 2021 Daniel Veilleux
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#define DT_DRV_COMPAT hbridge

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hbridge, CONFIG_MOTOR_LOG_LEVEL);

#define MAX_THROTTLE 100
// Simple functions for mapping a value betwen [0, 100] to the
// range [min, max] and vice versa.
#define MAP(value, min, max) ((value) * ((max) - (min))/MAX_THROTTLE + (min))
#define PAM(value, min, max) (((value) - (min)) * MAX_THROTTLE / ((max) - (min)))

#define PWM_A 0
#define PWM_B 1
#define PWM_CNT 2

struct hbridge_data {
	uint8_t current_throttle;
	enum motor_dir current_dir;
	bool ready;
};

struct hbridge_cfg {
    struct pwm_dt_spec pwm_spec[PWM_CNT];
};

static inline int channel_get(const struct device *dev,
	                     uint8_t *p_value)
{
	struct hbridge_data *p_data = dev->data;
    *p_value = p_data->current_throttle;
    return 0;
}

static inline int hbridge_set(const struct device *dev, enum motor_dir dir,
	                     uint32_t throttle)
{
    const struct hbridge_cfg *p_cfg  = dev->config;
    struct hbridge_data      *p_data = dev->data;
    uint32_t pwm_a, pwm_b;
    uint32_t pwm_a_period = p_cfg->pwm_spec[PWM_A].period;
    uint32_t pwm_b_period = p_cfg->pwm_spec[PWM_B].period;
    int ret;

	//printk("pwm set dir %u thr %u\n", dir, throttle);

	if (throttle > MAX_THROTTLE || dir > DIR_LAST)
		return -EINVAL;

	p_data->current_dir = dir;

	switch (dir) {
		case DIR_FORWARD:
			pwm_a = MAP(throttle, 0, pwm_a_period);
			pwm_b = 0;
			break;
		case DIR_BACKWARD:
			pwm_a = 0;
			pwm_b = MAP(throttle, 0, pwm_b_period);
			break;
		case DIR_BREAK:
			pwm_a = MAP(throttle, pwm_a_period, pwm_a_period);
			pwm_b = MAP(throttle, pwm_b_period, pwm_b_period);
			throttle = 0;
			break;
		case DIR_STOP:
			pwm_a = pwm_b = 0;
			throttle = 0;
			break;
	};
	p_data->current_throttle = throttle;

	ret = pwm_set_pulse_dt(&p_cfg->pwm_spec[PWM_A], pwm_a);
	if (unlikely(ret)) {
		printk("PWM_A set error %d\n", ret);
        return ret;
    }
	ret = pwm_set_pulse_dt(&p_cfg->pwm_spec[PWM_B], pwm_b);
	if (unlikely(ret)) {
		printk("PWM_A set error %d\n", ret);
        return ret;
    }

    return 0;
}


static int hbridge_write(const struct device *dev, enum motor_dir dir, uint32_t throttle)
{
	struct hbridge_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        //LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = hbridge_set(dev, dir, throttle);
    if (err)
		printk("hbridge_write error %d\n", err);
    return err;
}

static int hbridge_read(const struct device *dev, uint8_t *value)
{
	struct hbridge_data      *p_data = dev->data;

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

static int hbridge_init(const struct device *dev)
{
    int err;
    const struct hbridge_cfg *p_cfg  = dev->config;
    struct hbridge_data      *p_data = dev->data;

	printk("hbridge_init\n");
    if (unlikely(p_data->ready)) {
        /* Already initialized */
        return 0;
    }
    
    if (!pwm_is_ready_dt(&p_cfg->pwm_spec[0])) {
		printk("Error: PWM device %s/%d is not ready\n", p_cfg->pwm_spec[0].dev->name, p_cfg->pwm_spec[0].channel);
		return 0;
	}
    if (!pwm_is_ready_dt(&p_cfg->pwm_spec[1])) {
		printk("Error: PWM device %s/%d is not ready\n", p_cfg->pwm_spec[1].dev->name, p_cfg->pwm_spec[1].channel);
		return 0;
	}

    err = hbridge_set(dev, DIR_STOP, 0);
    if (unlikely(err)) {
		printk("channel_set error%d\n", err);
    	goto ERR_EXIT;
    }

    p_data->ready = true;
	LOG_INF("hbridge_init done\n");
    return 0;

ERR_EXIT:
    return -ENXIO;
}

static const struct motor_driver_api motor_driver_api = {
    .write = hbridge_write,
};

#define INST(num) DT_INST(num, hbridge)

#define HBRIDGE_DEVICE(n) \
    static const struct hbridge_cfg hbridge_cfg_##n = { \
        .pwm_spec     = { PWM_DT_SPEC_GET_BY_IDX(INST(n), 0), \
						  PWM_DT_SPEC_GET_BY_IDX(INST(n), 1) } \
    }; \
    static struct hbridge_data hbridge_data_##n; \
    DEVICE_DT_DEFINE(INST(n), \
            hbridge_init, \
            NULL, \
            &hbridge_data_##n, \
            &hbridge_cfg_##n, \
            POST_KERNEL, \
            CONFIG_MOTOR_HBRIDGE_INIT_PRIORITY, \
            &motor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HBRIDGE_DEVICE)

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "Hbridge driver enabled without any devices"
#endif
