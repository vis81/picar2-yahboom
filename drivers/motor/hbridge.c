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
#include <zephyr/drivers/pwm_compat.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_compat.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(hbridge, CONFIG_MOTOR_LOG_LEVEL);

#define MAX_THROTTLE 100
// Simple functions for mapping a value betwen [0, 100] to the
// range [min, max] and vice versa.
#define MAP(value, min, max) ((value) * ((max) - (min))/MAX_THROTTLE + (min))
#define PAM(value, min, max) (((value) - (min)) * MAX_THROTTLE / ((max) - (min)))

#define PWM_A 0
#define PWM_B 1
#define PWM_CNT 2

struct hbridge_pid {
	int32_t KP;
	int32_t KI;
	int32_t KD;
	int32_t KO;
	int32_t integral;
	int32_t prev_error;
	int32_t setpoint;
};

struct hbridge_data {
	const struct device *dev;
	struct motor_config motor_cfg;
	int32_t last_pos;
    int32_t last_vel;
	uint8_t current_throttle;
	enum motor_dir current_dir;
	bool ready;

	struct hbridge_pid pid;
	bool pid_enabled;
	bool pid_set;
	struct k_mutex pid_mutex;
};

struct hbridge_cfg {
    struct pwm_dt_spec pwm_spec[PWM_CNT];
    const struct device *const qdec_dev;
    uint32_t period_us;
    uint32_t counts_per_rev;
    uint32_t pid_gains[4];
};

static inline int hbridge_set(const struct device *dev, enum motor_dir dir,
	                     uint32_t throttle)
{
    const struct hbridge_cfg *p_cfg  = dev->config;
    struct hbridge_data      *p_data = dev->data;
    uint32_t pwm_a, pwm_b;
    uint32_t pwm_a_period = p_cfg->pwm_spec[PWM_A].period;
    uint32_t pwm_b_period = p_cfg->pwm_spec[PWM_B].period;
    int ret;

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
			pwm_a = MAP(MAX_THROTTLE, 0, pwm_a_period);
			pwm_b = MAP(MAX_THROTTLE, 0, pwm_b_period);
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
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = hbridge_set(dev, dir, throttle);
    if (err)
		LOG_WRN("hbridge_set error %d", err);
    return err;
}

static int hbridge_read(const struct device *dev, enum motor_dir *dir, uint32_t *throttle)
{
	struct hbridge_data      *p_data = dev->data;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }
	*throttle = p_data->current_throttle;
	*dir = p_data->current_dir;
    return 0;
}

static int hbridge_get_pos(const struct device *dev, int32_t *pos)
{
	struct hbridge_data      *p_data = dev->data;
	const struct hbridge_cfg *p_cfg  = dev->config;
	//int rc;
	
	if (!p_cfg->qdec_dev)
		return -ENOTSUP;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }
    *pos = p_data->last_pos * 360 / (int32_t) p_cfg->counts_per_rev;
    return 0;
}

static int hbridge_get_velocity(const struct device *dev, int32_t *vel)
{
	struct hbridge_data      *p_data = dev->data;
	const struct hbridge_cfg *p_cfg  = dev->config;
	
	if (!p_cfg->qdec_dev)
		return -ENOTSUP;

    if (unlikely(!p_data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }
	*vel = p_data->last_vel * 360 / (int32_t) p_cfg->counts_per_rev;
    return 0;
}

#if 0
// floating point impl
static float hbridge_do_pid(float current, float dt, struct hbridge_pid  *pid)
{
    float out;
    float proportional;
	float derivative;
	float error = pid->setpoint - current;
    float integral_tmp;

    integral_tmp = pid->integral + error * dt;
    derivative = (error - pid->prev_error) / dt;
    proportional = error;
    out = pid->KP * proportional + pid->KI * integral_tmp + pid->KD * derivative;
    //printk("PID1 P: %.3f I: %.3f D: %.3f\n", pid->KP * proportional, pid->KI * integral_tmp, pid->KD * derivative);
    if (out < 0)
        out = 0;
    else if (out > 100)
        out = 100;
    else
        pid->integral = integral_tmp;
    pid->prev_error = error;
    return out;
}
#endif
static int32_t hbridge_do_pid(int32_t current, int32_t dt, struct hbridge_pid  *pid)
{
    int32_t out;
    int32_t proportional;
	int32_t derivative;
	int32_t error = pid->setpoint - current;
    int32_t integral_tmp;

    integral_tmp = pid->integral + error * dt;
    derivative = (error - pid->prev_error);
    proportional = error;
    out = (pid->KP * proportional + pid->KI * integral_tmp / pid->KO + pid->KD * derivative * pid->KO / dt) / pid->KO;
    if (out < 0)
        out = 0;
    else if (out > MAX_THROTTLE)
        out = MAX_THROTTLE;
    else
        pid->integral = integral_tmp;
    pid->prev_error = error;
    return out;
}

static int hbridge_set_velocity(const struct device *dev, int32_t vel)
{
	struct hbridge_data      *p_data = dev->data;
	const struct hbridge_cfg *p_cfg  = dev->config;
	
	if (!p_cfg->qdec_dev)
		return -ENOTSUP;
	
	enum motor_dir dir = (vel == 0 ? DIR_STOP : (vel > 0 ? DIR_FORWARD : DIR_BACKWARD));

    k_mutex_lock(&p_data->pid_mutex, K_FOREVER);
	vel = abs(vel);
	p_data->pid.setpoint = vel * (float)p_cfg->counts_per_rev / 360.0
							* p_cfg->period_us / 1000000.0;
	p_data->pid_enabled = vel && p_data->pid_set;
	p_data->current_dir = dir;
	if (!vel) {
		hbridge_set(dev, dir, 0);
		// reset pid to avoid spikes on start
		p_data->pid.integral = 0;
		p_data->pid.prev_error = 0;
	}
    k_mutex_unlock(&p_data->pid_mutex);
    return 0;
}

static int hbridge_set_pid(const struct device *dev, int32_t kp, int32_t kd, int32_t ki, int32_t ko) {
    struct hbridge_data      *p_data = dev->data;
    k_mutex_lock(&p_data->pid_mutex, K_FOREVER);
    p_data->pid.KP = kp;
    p_data->pid.KI = ki;
    p_data->pid.KD = kd;
    p_data->pid.KO = ko;
    p_data->pid.integral = 0;
    p_data->pid.prev_error = 0;
    p_data->pid_set = true;
    k_mutex_unlock(&p_data->pid_mutex);
    LOG_DBG("set pid kp=%d kd=%d ki=%d ko=%d", kp, kd, ki, ko);
    return 0;
}

static void hbridge_input_cb(const struct device *dev, struct input_event *evt)
{
	struct hbridge_data      *p_data = dev->data;
	const struct hbridge_cfg *p_cfg  = dev->config;
	p_data->last_vel = evt->value * 1000000 / (int32_t)(p_cfg->period_us);
	p_data->last_pos += evt->value;

	k_mutex_lock(&p_data->pid_mutex, K_FOREVER);
	if (p_data->pid_enabled) {
		int32_t dt = (int32_t) ((int64_t)p_cfg->period_us * p_data->pid.KO / 1000000);
		int32_t val = abs(evt->value);
		int32_t pwm_val = hbridge_do_pid(val, dt, &p_data->pid);
		if (p_data->motor_cfg.flags.pid_debug)
			printk("PID %u %d %d %d\n", k_uptime_get_32(), p_data->pid.setpoint, val , pwm_val);
		hbridge_set(dev, p_data->current_dir, pwm_val);
	}
	k_mutex_unlock(&p_data->pid_mutex);
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

	k_mutex_init(&p_data->pid_mutex);
	p_data->dev = dev;
    p_data->ready = true;
    if (p_cfg->pid_gains[3])
		hbridge_set_pid(dev, p_cfg->pid_gains[0], p_cfg->pid_gains[1], p_cfg->pid_gains[2], p_cfg->pid_gains[3]);
	LOG_INF("hbridge_init done\n");
    return 0;

ERR_EXIT:
    return -ENXIO;
}

static int hbridge_configure(const struct device *dev, int32_t cfg_mask, struct motor_config *cfg) {
	struct hbridge_data      *p_data = dev->data;
	if (cfg_mask & MOTOR_CFG_PID_DEBUG)
		p_data->motor_cfg.flags.pid_debug = cfg->flags.pid_debug;
	return 0;
}

static const struct motor_driver_api motor_driver_api = {
    .write = hbridge_write,
    .read = hbridge_read,
    .get_pos = hbridge_get_pos,
    .get_velocity = hbridge_get_velocity,
    .set_velocity = hbridge_set_velocity,
    .set_pid = hbridge_set_pid,
    .configure = hbridge_configure,
};

#define INST(num) DT_INST(num, hbridge)
#define DT_QDEC_DEV(n)  DEVICE_DT_GET(DT_INST_PROP(n, qdec_dev))

#define INPUT_CALLBACK_FUNC(index) \
static void hbridge_input_cb_##index(struct input_event *evt)	\
{										\
	hbridge_input_cb(DEVICE_DT_GET(INST(index)), evt); \
}

#define ARR_ELEM(idx, inst, attr) \
	GPIO_DT_SPEC_INST_GET_BY_IDX(inst, attr, idx)

#define HBRIDGE_DEVICE(n) \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, qdec_dev), (INPUT_CALLBACK_FUNC(n)) ); \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(n, qdec_dev), (INPUT_CALLBACK_DEFINE(DT_QDEC_DEV(n), hbridge_input_cb_##n))); \
    static const struct hbridge_cfg hbridge_cfg_##n = { \
        .pwm_spec     = { PWM_DT_SPEC_GET_BY_IDX(INST(n), 0), \
						  PWM_DT_SPEC_GET_BY_IDX(INST(n), 1) }, \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, qdec_dev), (.qdec_dev = DT_QDEC_DEV(n),)) \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, qdec_dev), (.period_us = DT_PROP(DT_INST_PROP(n, qdec_dev), sample_time_us),)) \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, counts_per_rev), (.counts_per_rev = DT_INST_PROP(n, counts_per_rev),)) \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, pid_gains), ( .pid_gains = DT_INST_PROP(n, pid_gains), )) \
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
