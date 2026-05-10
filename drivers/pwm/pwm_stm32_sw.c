/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2024 Valentyn Shevchenko
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Software PWM driver for STM32 — uses a hardware timer for timing but drives
 * arbitrary GPIO pins rather than the timer's dedicated output pins.
 *
 * Mechanism: the timer runs freely and generates two interrupt types:
 *   UPDATE (UEV) — fires at the start of each period → assert all active GPIOs HIGH
 *   CCx           — fires when the counter reaches CCR  → drive the matching GPIO LOW
 *
 * This decouples PWM output from pin-mux constraints: any GPIO can carry PWM,
 * not only the pins wired to the timer's AF channels.  The trade-off is that
 * timing accuracy depends on ISR latency.
 *
 * All channels on one driver instance share a single timer and therefore a
 * single period.  Changing the period through one channel affects all channels.
 */

#define DT_DRV_COMPAT st_stm32_pwm_sw
#include <errno.h>
#include <stdlib.h>

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/dt-bindings/pwm/stm32_pwm.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(pwm_stm32_sw, CONFIG_PWM_LOG_LEVEL);

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

/* Helper macros for GPIO array initialization from DT */
#define ARR_LEN(inst, attr) \
	DT_INST_PROP_LEN(inst, attr)

#define ARR_ELEM(idx, inst, attr) \
	GPIO_DT_SPEC_INST_GET_BY_IDX(inst, attr, idx)

#define ARR_INIT(n, attr) \
	LISTIFY(ARR_LEN(n, attr), ARR_ELEM, (,), n, attr)

/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#if defined(LL_TIM_CHANNEL_CH6)
#define TIMER_HAS_6CH 1
#define TIMER_MAX_CH 6u
#else
#define TIMER_HAS_6CH 0
#define TIMER_MAX_CH 4u
#endif

struct pwm_stm32_data {
	uint32_t tim_clk;
	/* Last pulse width written per channel (adjusted timer ticks).
	 * Zero means the channel is idle; the ISR skips asserting HIGH for it. */
	uint32_t pulse_cycles[TIMER_MAX_CH];
};

struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct reset_dt_spec reset;
	struct gpio_dt_spec sw_gpio[TIMER_MAX_CH];
	uint32_t channels[TIMER_MAX_CH];
	uint32_t num_gpios;
	void (*irq_config_func)(const struct device *dev);
};

static const uint32_t ch2ll[TIMER_MAX_CH] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if TIMER_HAS_6CH
	LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
};

static void (*const set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *, uint32_t) = {
	LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
	LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
#if TIMER_HAS_6CH
	LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
};

/*
 * CC interrupt control for CH1-CH4 only.  CH5/CH6 on some parts share a
 * combined CC interrupt and are not supported by this driver.
 */
static void (*const enable_cc_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_EnableIT_CC1, LL_TIM_EnableIT_CC2,
	LL_TIM_EnableIT_CC3, LL_TIM_EnableIT_CC4
};

static void (*const disable_cc_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_DisableIT_CC1, LL_TIM_DisableIT_CC2,
	LL_TIM_DisableIT_CC3, LL_TIM_DisableIT_CC4
};

static uint32_t (*const is_cc_enabled[])(const TIM_TypeDef *) = {
	LL_TIM_IsEnabledIT_CC1, LL_TIM_IsEnabledIT_CC2,
	LL_TIM_IsEnabledIT_CC3, LL_TIM_IsEnabledIT_CC4
};

/* SR flag for each CC channel (1-based index; [0] unused). */
static const uint32_t chan2srbit[] = {
	0,
	TIM_SR_CC1IF, TIM_SR_CC2IF, TIM_SR_CC3IF, TIM_SR_CC4IF,
};

static inline bool is_center_aligned(const uint32_t ll_countermode)
{
	return ((ll_countermode == LL_TIM_COUNTERMODE_CENTER_DOWN) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP_DOWN));
}

static int get_tim_clk(const struct stm32_pclken *pclken, uint32_t *tim_clk)
{
	int r;
	const struct device *clk;
	uint32_t bus_clk, apb_psc;

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	r = clock_control_get_rate(clk, (clock_control_subsys_t)pclken,
				   &bus_clk);
	if (r < 0) {
		return r;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	if (pclken->bus == STM32_CLOCK_BUS_APB1) {
		apb_psc = STM32_D2PPRE1;
	} else {
		apb_psc = STM32_D2PPRE2;
	}
#else
	if (pclken->bus == STM32_CLOCK_BUS_APB1) {
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB1DIVR, RCC_APB1DIVR_APB1DIV));
#else
		apb_psc = STM32_APB1_PRESCALER;
#endif
	}
#if !defined(CONFIG_SOC_SERIES_STM32C0X) && !defined(CONFIG_SOC_SERIES_STM32F0X) && \
	!defined(CONFIG_SOC_SERIES_STM32G0X)
	else {
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB2DIVR, RCC_APB2DIVR_APB2DIV));
#else
		apb_psc = STM32_APB2_PRESCALER;
#endif
	}
#endif
#endif

#if defined(RCC_DCKCFGR_TIMPRE) || defined(RCC_DCKCFGR1_TIMPRE) || \
	defined(RCC_CFGR_TIMPRE)
	if (LL_RCC_GetTIMPrescaler() == LL_RCC_TIM_PRESCALER_TWICE) {
		/* TIMPRE = 0 */
		if (apb_psc <= 2u) {
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		} else {
			*tim_clk = bus_clk * 2u;
		}
	} else {
		/* TIMPRE = 1 */
		if (apb_psc <= 4u) {
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		} else {
			*tim_clk = bus_clk * 4u;
		}
	}
#else
	if (apb_psc == 1u) {
		*tim_clk = bus_clk;
	} else {
		*tim_clk = bus_clk * 2u;
	}
#endif

	return 0;
}

static int pwm_stm32_set_cycles(const struct device *dev, uint32_t channel,
				uint32_t period_cycles, uint32_t pulse_cycles,
				pwm_flags_t flags)
{
	const struct pwm_stm32_config *cfg = dev->config;
	struct pwm_stm32_data *data = dev->data;
	int gpio_idx;

	/* Polarity flags are not implemented: the GPIO is always asserted HIGH
	 * on UPDATE (period start) and LOW on CC (compare match). */
	ARG_UNUSED(flags);

	for (gpio_idx = 0; gpio_idx < cfg->num_gpios; gpio_idx++) {
		if (cfg->channels[gpio_idx] == channel)
			break;
	}

	if (channel < 1u || channel > TIMER_MAX_CH || gpio_idx == cfg->num_gpios) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}

	/* CH5/CH6 on some parts do not have per-channel CC interrupts. */
	if (channel <= ARRAY_SIZE(disable_cc_interrupt)) {
		disable_cc_interrupt[channel - 1](cfg->timer);
	}

	if (!IS_TIM_32B_COUNTER_INSTANCE(cfg->timer) &&
	    (period_cycles > UINT16_MAX + 1)) {
		return -ENOTSUP;
	}

	const uint32_t ll_channel = ch2ll[channel - 1u];

	if (period_cycles == 0u) {
		LL_TIM_CC_DisableChannel(cfg->timer, ll_channel);
		data->pulse_cycles[gpio_idx] = 0;
		gpio_pin_set_dt(&cfg->sw_gpio[gpio_idx], 0);
		/* Disable UPDATE interrupt when no channels remain active. */
		bool any_active = false;

		for (int i = 0; i < cfg->num_gpios; i++) {
			if (data->pulse_cycles[i]) {
				any_active = true;
				break;
			}
		}
		if (!any_active) {
			LL_TIM_DisableIT_UPDATE(cfg->timer);
		}
		return 0;
	}

	if (cfg->countermode == LL_TIM_COUNTERMODE_UP) {
		/* remove 1 period cycle, accounts for 1 extra low cycle */
		period_cycles -= 1U;
	} else if (cfg->countermode == LL_TIM_COUNTERMODE_DOWN) {
		/* remove 1 pulse cycle, accounts for 1 extra high cycle */
		pulse_cycles -= 1U;
		/* remove 1 period cycle, accounts for 1 extra low cycle */
		period_cycles -= 1U;
	} else if (is_center_aligned(cfg->countermode)) {
		pulse_cycles /= 2U;
		period_cycles /= 2U;
	} else {
		return -ENOTSUP;
	}

	if (!LL_TIM_CC_IsEnabledChannel(cfg->timer, ll_channel)) {
		LL_TIM_OC_InitTypeDef oc_init;

		LL_TIM_OC_StructInit(&oc_init);
		/* Configure the OC channel in PWM1 mode to arm the CC interrupt.
		 * The timer's physical output pin is not used — we drive GPIOs
		 * directly from the ISR. */
		oc_init.OCMode = LL_TIM_OCMODE_PWM1;
		oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
		oc_init.CompareValue = pulse_cycles;

		if (LL_TIM_OC_Init(cfg->timer, ll_channel, &oc_init) != SUCCESS) {
			LOG_ERR("Could not initialize timer channel output");
			return -EIO;
		}

		LL_TIM_EnableARRPreload(cfg->timer);
		LL_TIM_OC_EnablePreload(cfg->timer, ll_channel);
		LL_TIM_SetAutoReload(cfg->timer, period_cycles);
		/* Force an immediate update event to transfer CCR/ARR from their
		 * preload registers so the very first period has correct timing. */
		LL_TIM_GenerateEvent_UPDATE(cfg->timer);
	} else {
		set_timer_compare[channel - 1u](cfg->timer, pulse_cycles);
		LL_TIM_SetAutoReload(cfg->timer, period_cycles);
	}

	data->pulse_cycles[gpio_idx] = pulse_cycles;

	if (pulse_cycles == 0u) {
		gpio_pin_set_dt(&cfg->sw_gpio[gpio_idx], 0);
		/* Disable UPDATE interrupt when no channels remain active. */
		bool any_active = false;

		for (int i = 0; i < cfg->num_gpios; i++) {
			if (data->pulse_cycles[i]) {
				any_active = true;
				break;
			}
		}
		if (!any_active) {
			LL_TIM_DisableIT_UPDATE(cfg->timer);
		}
	} else if (pulse_cycles > period_cycles) {
		/* 100% duty: CCR > ARR, CC event unreachable.  Assert HIGH now
		 * instead of arming CC, which would race against the stale
		 * CCR_active=0 value and drive GPIO LOW on the first UEV. */
		gpio_pin_set_dt(&cfg->sw_gpio[gpio_idx], 1);
		LL_TIM_EnableIT_UPDATE(cfg->timer);
	} else {
		if (channel <= ARRAY_SIZE(enable_cc_interrupt)) {
			enable_cc_interrupt[channel - 1](cfg->timer);
		}
		LL_TIM_EnableIT_UPDATE(cfg->timer);
	}

	return 0;
}

static void pwm_stm32_isr(const struct device *dev)
{
	const struct pwm_stm32_config *cfg = dev->config;
	struct pwm_stm32_data *data = dev->data;
	uint32_t sr = READ_REG(cfg->timer->SR);

	/* Process UPDATE first: when both UIF and CC are pending (ISR delayed
	 * past pulse end), the final GPIO state will be LOW (CC wins below). */
	if (sr & TIM_SR_UIF) {
		for (int i = 0; i < cfg->num_gpios; i++) {
			/* pulse_cycles == 0 means idle; GPIO should stay LOW. */
			if (!data->pulse_cycles[i])
				continue;
			gpio_pin_set_dt(&cfg->sw_gpio[i], 1);
		}
		LL_TIM_ClearFlag_UPDATE(cfg->timer);
	}

	for (int i = 0; i < cfg->num_gpios; i++) {
		uint32_t ch = cfg->channels[i];

		if (ch > ARRAY_SIZE(is_cc_enabled)) {
			continue;
		}
		if (!is_cc_enabled[ch - 1](cfg->timer)) {
			continue;
		}
		uint32_t sr_bit = chan2srbit[ch];

		if (sr & sr_bit) {
			gpio_pin_set_dt(&cfg->sw_gpio[i], 0);
			/* STM32 SR flags are RC_W0: write 0 to clear, write 1
			 * has no effect.  ~sr_bit clears only this flag. */
			WRITE_REG(cfg->timer->SR, ~sr_bit);
		}
	}
}

static int pwm_stm32_get_cycles_per_sec(const struct device *dev,
					uint32_t channel, uint64_t *cycles)
{
	struct pwm_stm32_data *data = dev->data;
	const struct pwm_stm32_config *cfg = dev->config;

	*cycles = (uint64_t)(data->tim_clk / (cfg->prescaler + 1));

	return 0;
}

static const struct pwm_driver_api pwm_stm32_driver_api = {
	.set_cycles = pwm_stm32_set_cycles,
	.get_cycles_per_sec = pwm_stm32_get_cycles_per_sec,
};

static int pwm_stm32_init(const struct device *dev)
{
	struct pwm_stm32_data *data = dev->data;
	const struct pwm_stm32_config *cfg = dev->config;

	int r;
	const struct device *clk;
	LL_TIM_InitTypeDef init;

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	r = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
	if (r < 0) {
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}

	r = get_tim_clk(&cfg->pclken, &data->tim_clk);
	if (r < 0) {
		LOG_ERR("Could not obtain timer clock (%d)", r);
		return r;
	}

	(void)reset_line_toggle_dt(&cfg->reset);

	LL_TIM_StructInit(&init);

	init.Prescaler = cfg->prescaler;
	init.CounterMode = cfg->countermode;
	init.Autoreload = 0u;
	init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

	if (LL_TIM_Init(cfg->timer, &init) != SUCCESS) {
		LOG_ERR("Could not initialize timer");
		return -EIO;
	}

#if !defined(CONFIG_SOC_SERIES_STM32L0X) && !defined(CONFIG_SOC_SERIES_STM32L1X)
	/* Enable main output for advanced timers that have a break circuit
	 * (e.g. TIM1, TIM8); a no-op for general-purpose timers. */
	if (IS_TIM_BREAK_INSTANCE(cfg->timer)) {
		LL_TIM_EnableAllOutputs(cfg->timer);
	}
#endif

	LL_TIM_EnableCounter(cfg->timer);

	for (int i = 0; i < cfg->num_gpios; i++) {
		if (gpio_is_ready_dt(&cfg->sw_gpio[i])) {
			if (gpio_pin_configure_dt(&cfg->sw_gpio[i], GPIO_OUTPUT_ACTIVE) < 0) {
				LOG_ERR("gpio_pin_configure_dt fail");
			}
		} else {
			LOG_ERR("gpio_is_ready_dt fail");
		}
	}

	cfg->irq_config_func(dev);
	return 0;
}

#define PWM(index) DT_INST_PARENT(index)

#define IRQ_CONNECT_AND_ENABLE_BY_NAME(index, name)				\
{										\
	IRQ_CONNECT(DT_IRQ_BY_NAME(PWM(index), name, irq),			\
		    DT_IRQ_BY_NAME(PWM(index), name, priority),			\
		    pwm_stm32_isr, DEVICE_DT_INST_GET(index), IRQ_ZERO_LATENCY);\
	irq_enable(DT_IRQ_BY_NAME(PWM(index), name, irq));			\
}

#define IRQ_CONNECT_AND_ENABLE_DEFAULT(index)					\
{										\
	IRQ_CONNECT(DT_IRQN(PWM(index)),					\
		    DT_IRQ(PWM(index), priority),				\
		    pwm_stm32_isr, DEVICE_DT_INST_GET(index), IRQ_ZERO_LATENCY);\
	irq_enable(DT_IRQN(PWM(index)));					\
}

#define IRQ_CONFIG_FUNC(index)							\
static void pwm_stm32_irq_config_func_##index(const struct device *dev)	\
{										\
	COND_CODE_1(DT_IRQ_HAS_NAME(PWM(index), cc),				\
		(IRQ_CONNECT_AND_ENABLE_BY_NAME(index, cc)			\
		 IRQ_CONNECT_AND_ENABLE_BY_NAME(index, up)),			\
		(IRQ_CONNECT_AND_ENABLE_DEFAULT(index))				\
	);									\
}

#define DT_INST_CLK(index, inst)						\
	{									\
		.bus = DT_CLOCKS_CELL(PWM(index), bus),				\
		.enr = DT_CLOCKS_CELL(PWM(index), bits)				\
	}

#define PWM_DEVICE_INIT(index)							\
	BUILD_ASSERT(ARR_LEN(index, gpios) <= TIMER_MAX_CH,			\
		"st,stm32-pwm-sw: gpios count exceeds TIMER_MAX_CH ("		\
		STRINGIFY(TIMER_MAX_CH) ")");					\
	BUILD_ASSERT(ARR_LEN(index, gpios) ==					\
		     DT_INST_PROP_LEN(index, channels),				\
		"st,stm32-pwm-sw: gpios and channels must be the same length");\
										\
	static struct pwm_stm32_data pwm_stm32_data_##index;			\
										\
	IRQ_CONFIG_FUNC(index)							\
										\
	static const struct pwm_stm32_config pwm_stm32_config_##index = {	\
		.timer = (TIM_TypeDef *)DT_REG_ADDR(PWM(index)),		\
		.prescaler = DT_PROP(PWM(index), st_prescaler),			\
		.countermode = DT_PROP(PWM(index), st_countermode),		\
		.pclken = DT_INST_CLK(index, timer),				\
		.reset = RESET_DT_SPEC_GET(PWM(index)),				\
		.sw_gpio = { ARR_INIT(index, gpios) },				\
		.channels = DT_PROP(DT_INST(index, st_stm32_pwm_sw), channels),	\
		.num_gpios = ARR_LEN(index, gpios),				\
		.irq_config_func = pwm_stm32_irq_config_func_##index,		\
	};									\
										\
	DEVICE_DT_INST_DEFINE(index, &pwm_stm32_init, NULL,			\
			      &pwm_stm32_data_##index,				\
			      &pwm_stm32_config_##index, POST_KERNEL,		\
			      CONFIG_PWM_INIT_PRIORITY,				\
			      &pwm_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
