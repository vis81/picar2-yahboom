/*
 * Copyright (c) 2022, Valerio Setti <vsetti@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_input_qdec

/** @file
 * @brief STM32 family Quadrature Decoder (QDEC) driver.
 */

#include <errno.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/logging/log.h>

#include <stm32_ll_tim.h>

LOG_MODULE_REGISTER(input_stm32_qdec, CONFIG_INPUT_LOG_LEVEL);

/* Device constant configuration parameters */
struct qdec_stm32_dev_cfg {
	const struct pinctrl_dev_config *pin_config;
	struct stm32_pclken pclken;
	TIM_TypeDef *timer_inst;
	bool is_input_polarity_inverted;
	uint8_t input_filtering_level;
	uint32_t sample_time_us;
	uint16_t axis;
};

/* Device run time data */
struct qdec_stm32_dev_data {
	const struct device *dev;
	struct k_timer poll_timer;
	struct k_work work;
	int16_t last_position;
	int32_t delta;
};

static void qdec_stm32_polling_work(struct k_work *work) {
	struct qdec_stm32_dev_data *data = CONTAINER_OF(work, struct qdec_stm32_dev_data, work);
	const struct qdec_stm32_dev_cfg *const cfg = data->dev->config;
	//LOG_ERR("timer %d %d", data->delta, data->last_position);
	input_report_rel(data->dev, cfg->axis, data->delta, true, K_FOREVER);
}

int16_t difference_int16(int16_t i, int16_t j) {
    return (int16_t)((int32_t)i - (int32_t)j);
}

static void qdec_stm32_timer_func(struct k_timer *timer_id) {
	struct qdec_stm32_dev_data *data = CONTAINER_OF(timer_id, struct qdec_stm32_dev_data, poll_timer);
	const struct qdec_stm32_dev_cfg *const dev_cfg = data->dev->config;
	int16_t t = LL_TIM_GetCounter(dev_cfg->timer_inst);
	
	data->delta =  difference_int16(t, data->last_position);
	data->last_position = t;
	k_work_submit(&data->work);
}

static int qdec_stm32_initialize(const struct device *dev)
{
	const struct qdec_stm32_dev_cfg *const dev_cfg = dev->config;
	struct qdec_stm32_dev_data *const p_data = dev->data;
	int retval;
	LL_TIM_ENCODER_InitTypeDef init_props;
	uint32_t max_counter_value;

	retval = pinctrl_apply_state(dev_cfg->pin_config, PINCTRL_STATE_DEFAULT);
	if (retval < 0) {
		return retval;
	}

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	retval = clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t)&dev_cfg->pclken);
	if (retval < 0) {
		LOG_ERR("Could not initialize clock");
		return retval;
	}

	LL_TIM_ENCODER_StructInit(&init_props);

	if (dev_cfg->is_input_polarity_inverted) {
		init_props.IC1ActiveInput = LL_TIM_IC_POLARITY_FALLING;
		init_props.IC2ActiveInput = LL_TIM_IC_POLARITY_FALLING;
	}

	init_props.IC1Filter = dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2;
	init_props.IC2Filter = dev_cfg->input_filtering_level * LL_TIM_IC_FILTER_FDIV1_N2;

	init_props.EncoderMode = LL_TIM_ENCODERMODE_X4_TI12;

	if (IS_TIM_32B_COUNTER_INSTANCE(dev_cfg->timer_inst)) {
		max_counter_value = UINT32_MAX;
	} else {
		max_counter_value = UINT16_MAX;
	}
	LL_TIM_SetAutoReload(dev_cfg->timer_inst, max_counter_value);

	if (LL_TIM_ENCODER_Init(dev_cfg->timer_inst, &init_props) != SUCCESS) {
		LOG_ERR("Initalization failed");
		return -EIO;
	}

	LL_TIM_EnableCounter(dev_cfg->timer_inst);
	p_data->last_position = 0;
	p_data->dev = dev;

	k_timer_init(&p_data->poll_timer, qdec_stm32_timer_func, NULL);
	k_work_init(&p_data->work, qdec_stm32_polling_work);
	k_timer_start(&p_data->poll_timer, K_USEC(dev_cfg->sample_time_us), K_USEC(dev_cfg->sample_time_us));
	return 0;
}

#define QDEC_STM32_INIT(n)								\
	PINCTRL_DT_INST_DEFINE(n);							\
	static const struct qdec_stm32_dev_cfg qdec##n##_stm32_config = {		\
		.pin_config = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.timer_inst = ((TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n))),		\
		.pclken = {								\
			.bus = DT_CLOCKS_CELL(DT_INST_PARENT(n), bus),			\
			.enr = DT_CLOCKS_CELL(DT_INST_PARENT(n), bits)			\
		},									\
		.is_input_polarity_inverted = DT_INST_PROP(n, st_input_polarity_inverted),	\
		.input_filtering_level = DT_INST_PROP(n, st_input_filter_level),		\
		.sample_time_us = DT_INST_PROP(n, sample_time_us),		\
		.axis = DT_INST_PROP(n, zephyr_axis), \
	};										\
											\
	static struct qdec_stm32_dev_data qdec##n##_stm32_data;				\
											\
	DEVICE_DT_INST_DEFINE(n, qdec_stm32_initialize, NULL,			\
				&qdec##n##_stm32_data, &qdec##n##_stm32_config,		\
				POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,		\
				NULL);

DT_INST_FOREACH_STATUS_OKAY(QDEC_STM32_INIT)
