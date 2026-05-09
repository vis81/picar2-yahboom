/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2020 Teslabs Engineering S.L.
 * Copyright (c) 2023 Nobleo Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_pwm_sw
#include <errno.h>
#include <stdlib.h>

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
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


/* Helper macro that UTIL_LISTIFY can use and produces an element with comma */
#define ARR_LEN(inst, attr) \
	DT_INST_PROP_LEN(inst, attr)

#define ARR_ELEM(idx, inst, attr) \
	GPIO_DT_SPEC_INST_GET_BY_IDX(inst, attr, idx)

#define ARR_INIT(n, attr) \
	LISTIFY(ARR_LEN(n, attr), ARR_ELEM, (,), n, attr)

#define MAX_GPIOS 4

/** PWM data. */
struct pwm_stm32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
    uint32_t pulse_cycles[MAX_GPIOS];
};

/** PWM configuration. */
struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
    struct gpio_dt_spec sw_gpio[MAX_GPIOS];
    uint32_t channels[MAX_GPIOS];
    uint32_t num_gpios;
	void (*irq_config_func)(const struct device *dev);
};

/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#if defined(LL_TIM_CHANNEL_CH6)
#define TIMER_HAS_6CH 1
#define TIMER_MAX_CH 6u
#else
#define TIMER_HAS_6CH 0
#define TIMER_MAX_CH 4u
#endif

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if TIMER_HAS_6CH
	LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
};

/** Some stm32 mcus have complementary channels : 3 or 4 */
static const uint32_t ch2ll_n[] = {
#if defined(LL_TIM_CHANNEL_CH1N)
	LL_TIM_CHANNEL_CH1N,
	LL_TIM_CHANNEL_CH2N,
	LL_TIM_CHANNEL_CH3N,
#if defined(LL_TIM_CHANNEL_CH4N)
/** stm32g4x and stm32u5x have 4 complementary channels */
	LL_TIM_CHANNEL_CH4N,
#endif /* LL_TIM_CHANNEL_CH4N */
#endif /* LL_TIM_CHANNEL_CH1N */
};
/** Maximum number of complemented timer channels is ARRAY_SIZE(ch2ll_n)*/

/** Channel to compare set function mapping. */
static void (*const set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *,
						     uint32_t) = {
	LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
	LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
#if TIMER_HAS_6CH
	LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
};

/** Channel to capture get function mapping. */
#if !defined(CONFIG_SOC_SERIES_STM32F4X) && \
	!defined(CONFIG_SOC_SERIES_STM32G4X) && \
	!defined(CONFIG_SOC_SERIES_STM32MP1X)
static uint32_t __maybe_unused (*const get_channel_capture[])(const TIM_TypeDef *) = {
#else
static uint32_t __maybe_unused (*const get_channel_capture[])(TIM_TypeDef *) = {
#endif
	LL_TIM_IC_GetCaptureCH1, LL_TIM_IC_GetCaptureCH2,
	LL_TIM_IC_GetCaptureCH3, LL_TIM_IC_GetCaptureCH4
};


/** Channel to enable capture interrupt mapping. */
static void __maybe_unused (*const enable_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_EnableIT_CC1, LL_TIM_EnableIT_CC2,
	LL_TIM_EnableIT_CC3, LL_TIM_EnableIT_CC4
};

/** Channel to disable capture interrupt mapping. */
static void __maybe_unused (*const disable_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_DisableIT_CC1, LL_TIM_DisableIT_CC2,
	LL_TIM_DisableIT_CC3, LL_TIM_DisableIT_CC4
};

/** Channel to is capture active flag mapping. */
#if !defined(CONFIG_SOC_SERIES_STM32F4X) && \
	!defined(CONFIG_SOC_SERIES_STM32G4X) && \
	!defined(CONFIG_SOC_SERIES_STM32MP1X)
static uint32_t __maybe_unused (*const is_capture_active[])(const TIM_TypeDef *) = {
#else
static uint32_t __maybe_unused (*const is_capture_active[])(TIM_TypeDef *) = {
#endif
	LL_TIM_IsActiveFlag_CC1, LL_TIM_IsActiveFlag_CC2,
	LL_TIM_IsActiveFlag_CC3, LL_TIM_IsActiveFlag_CC4
};

/** Channel to clearing capture flag mapping. */
static void __maybe_unused (*const clear_capture_interrupt[])(TIM_TypeDef *) = {
	LL_TIM_ClearFlag_CC1, LL_TIM_ClearFlag_CC2,
	LL_TIM_ClearFlag_CC3, LL_TIM_ClearFlag_CC4
};

static uint32_t __maybe_unused (*const is_it_enabled[])(const TIM_TypeDef *) = {
	LL_TIM_IsEnabledIT_CC1, LL_TIM_IsEnabledIT_CC2,
	LL_TIM_IsEnabledIT_CC3, LL_TIM_IsEnabledIT_CC4
};

/**
 * Obtain LL polarity from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL polarity.
 */
static uint32_t get_polarity(pwm_flags_t flags)
{
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		return LL_TIM_OCPOLARITY_HIGH;
	}

	return LL_TIM_OCPOLARITY_LOW;
}

/**
 * @brief  Check if LL counter mode is center-aligned.
 *
 * @param  ll_countermode LL counter mode.
 *
 * @return `true` when center-aligned, otherwise `false`.
 */
static inline bool is_center_aligned(const uint32_t ll_countermode)
{
	return ((ll_countermode == LL_TIM_COUNTERMODE_CENTER_DOWN) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP_DOWN));
}

/**
 * Obtain timer clock speed.
 *
 * @param pclken  Timer clock control subsystem.
 * @param tim_clk Where computed timer clock will be stored.
 *
 * @return 0 on success, error code otherwise.
 */
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
#if !defined(CONFIG_SOC_SERIES_STM32C0X) && !defined(CONFIG_SOC_SERIES_STM32F0X) &&                \
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
	/*
	 * There are certain series (some F4, F7 and H7) that have the TIMPRE
	 * bit to control the clock frequency of all the timers connected to
	 * APB1 and APB2 domains.
	 *
	 * Up to a certain threshold value of APB{1,2} prescaler, timer clock
	 * equals to HCLK. This threshold value depends on TIMPRE setting
	 * (2 if TIMPRE=0, 4 if TIMPRE=1). Above threshold, timer clock is set
	 * to a multiple of the APB domain clock PCLK{1,2} (2 if TIMPRE=0, 4 if
	 * TIMPRE=1).
	 */

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
	/*
	 * If the APB prescaler equals 1, the timer clock frequencies
	 * are set to the same frequency as that of the APB domain.
	 * Otherwise, they are set to twice (×2) the frequency of the
	 * APB domain.
	 */
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

	uint32_t ll_channel;
	uint32_t current_ll_channel; /* complementary output if used */
	uint32_t negative_ll_channel;
    int gpio_idx;
    
    for (gpio_idx = 0; gpio_idx < cfg->num_gpios; gpio_idx++) {
        if (cfg->channels[gpio_idx] == channel)
            break;
    }

	if (channel < 1u || channel > TIMER_MAX_CH || gpio_idx == cfg->num_gpios) {
		LOG_ERR("Invalid channel (%d)", channel);
		return -EINVAL;
	}
	disable_capture_interrupt[channel - 1](cfg->timer);

	/*
	 * Non 32-bit timers count from 0 up to the value in the ARR register
	 * (16-bit). Thus period_cycles cannot be greater than UINT16_MAX + 1.
	 */
	if (!IS_TIM_32B_COUNTER_INSTANCE(cfg->timer) &&
	    (period_cycles > UINT16_MAX + 1)) {
		return -ENOTSUP;
	}

	ll_channel = ch2ll[channel - 1u];

	if (channel <= ARRAY_SIZE(ch2ll_n)) {
		negative_ll_channel = ch2ll_n[channel - 1u];
	} else {
		negative_ll_channel = 0;
	}

	/* in LL_TIM_CC_DisableChannel and LL_TIM_CC_IsEnabledChannel,
	 * the channel param could be the complementary one
	 */
	if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
		if (!negative_ll_channel) {
			/* setting a flag on a channel that has not this capability */
			LOG_ERR("Channel %d has NO complementary output", channel);
			return -EINVAL;
		}
		current_ll_channel = negative_ll_channel;
	} else {
		current_ll_channel = ll_channel;
	}

	if (period_cycles == 0u) {
		LL_TIM_CC_DisableChannel(cfg->timer, current_ll_channel);
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

	if (!LL_TIM_CC_IsEnabledChannel(cfg->timer, current_ll_channel)) {
		LL_TIM_OC_InitTypeDef oc_init;

		LL_TIM_OC_StructInit(&oc_init);

		oc_init.OCMode = LL_TIM_OCMODE_PWM1;

#if defined(LL_TIM_CHANNEL_CH1N)
		/* the flags holds the STM32_PWM_COMPLEMENTARY information */
		if ((flags & STM32_PWM_COMPLEMENTARY_MASK) == STM32_PWM_COMPLEMENTARY) {
			oc_init.OCNState = LL_TIM_OCSTATE_ENABLE;
			oc_init.OCNPolarity = get_polarity(flags);

			/* inherit the polarity of the positive output */
			oc_init.OCState = LL_TIM_CC_IsEnabledChannel(cfg->timer, ll_channel)
						  ? LL_TIM_OCSTATE_ENABLE
						  : LL_TIM_OCSTATE_DISABLE;
			oc_init.OCPolarity = LL_TIM_OC_GetPolarity(cfg->timer, ll_channel);
		} else {
			oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
			oc_init.OCPolarity = get_polarity(flags);

			/* inherit the polarity of the negative output */
			if (negative_ll_channel) {
				oc_init.OCNState =
					LL_TIM_CC_IsEnabledChannel(cfg->timer, negative_ll_channel)
						? LL_TIM_OCSTATE_ENABLE
						: LL_TIM_OCSTATE_DISABLE;
				oc_init.OCNPolarity =
					LL_TIM_OC_GetPolarity(cfg->timer, negative_ll_channel);
			}
		}
#else /* LL_TIM_CHANNEL_CH1N */

		oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
		oc_init.OCPolarity = get_polarity(flags);
#endif /* LL_TIM_CHANNEL_CH1N */
		oc_init.CompareValue = pulse_cycles;

		/* in LL_TIM_OC_Init, the channel is always the non-complementary */
		if (LL_TIM_OC_Init(cfg->timer, ll_channel, &oc_init) != SUCCESS) {
			LOG_ERR("Could not initialize timer channel output");
			return -EIO;
		}

		LL_TIM_EnableARRPreload(cfg->timer);
		/* in LL_TIM_OC_EnablePreload, the channel is always the non-complementary */
		LL_TIM_OC_EnablePreload(cfg->timer, ll_channel);
		LL_TIM_SetAutoReload(cfg->timer, period_cycles);
		LL_TIM_GenerateEvent_UPDATE(cfg->timer);
	} else {
		/* in LL_TIM_OC_SetPolarity, the channel could be the complementary one */
		LL_TIM_OC_SetPolarity(cfg->timer, current_ll_channel, get_polarity(flags));
		set_timer_compare[channel - 1u](cfg->timer, pulse_cycles);
		LL_TIM_SetAutoReload(cfg->timer, period_cycles);
	}
    data->pulse_cycles[gpio_idx] = pulse_cycles;
    if (pulse_cycles) {
		enable_capture_interrupt[channel - 1](cfg->timer);
		LL_TIM_EnableIT_UPDATE(cfg->timer);
	} else {
		gpio_pin_set_dt(&cfg->sw_gpio[gpio_idx], 0);
	}
	return 0;
}

int chan2srbit[] = {
    0,
    TIM_SR_CC1IF,
    TIM_SR_CC2IF,
    TIM_SR_CC3IF,
    TIM_SR_CC4IF,
};

#define CLOCKS_PER_USEC (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000)
static void pwm_stm32_isr(const struct device *dev)
{
	const struct pwm_stm32_config *cfg = dev->config;
    int sr = READ_REG(cfg->timer->SR);

	struct pwm_stm32_data *data = dev->data;
#if 0
    int dier = READ_REG(cfg->timer->DIER);
    printk("%10u:isr 0x%x 0x%x 0x%x pin %d\n", k_uptime_get_32(), sr, dier, READ_REG(cfg->timer->CNT), cfg->sw_gpio[0].pin);
#endif
    /* Process UPDATE first so that when both UIF and CC are pending
     * (ISR delayed past pulse end), the final GPIO state is LOW (CC wins). */
    if (sr & TIM_SR_UIF) {
        for (int i = 0; i < cfg->num_gpios; i++) {
            int channel = cfg->channels[i];
            if (!is_it_enabled[channel - 1](cfg->timer))
                continue;
            gpio_pin_set_dt(&cfg->sw_gpio[i], 1);
        }
        LL_TIM_ClearFlag_UPDATE(cfg->timer);
    }

    for (int i = 0; i < cfg->num_gpios; i++) {
        int channel = cfg->channels[i];
        if (!is_it_enabled[channel - 1](cfg->timer))
            continue;
        int sr_bit = chan2srbit[channel];
        if (sr & sr_bit) {
            gpio_pin_set_dt(&cfg->sw_gpio[i], 0);
            WRITE_REG(cfg->timer->SR, ~(sr_bit));
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

	/* enable clock and store its speed */
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

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&data->reset);

	/* initialize timer */
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
	/* enable outputs and counter */
	if (IS_TIM_BREAK_INSTANCE(cfg->timer)) {
		LL_TIM_EnableAllOutputs(cfg->timer);
	}
#endif

	LL_TIM_EnableCounter(cfg->timer);

    for (int i = 0; i < cfg->num_gpios; i++) {
        if (gpio_is_ready_dt(&cfg->sw_gpio[i])) {
            if (gpio_pin_configure_dt(&cfg->sw_gpio[i], GPIO_OUTPUT_ACTIVE) < 0)
                LOG_ERR("gpio_pin_configure_dt fail");
        } else
            LOG_ERR("gpio_is_ready_dt fail");
    }

	cfg->irq_config_func(dev);
	return 0;
}

#define PWM(index) DT_INST_PARENT(index)

#define IRQ_CONNECT_AND_ENABLE_BY_NAME(index, name)				\
{										\
	IRQ_CONNECT(DT_IRQ_BY_NAME(PWM(index), name, irq),			\
			DT_IRQ_BY_NAME(PWM(index), name, priority),		\
			pwm_stm32_isr, DEVICE_DT_INST_GET(index), IRQ_ZERO_LATENCY);		\
	irq_enable(DT_IRQ_BY_NAME(PWM(index), name, irq));			\
}

#define IRQ_CONNECT_AND_ENABLE_DEFAULT(index)					\
{										\
	IRQ_CONNECT(DT_IRQN(PWM(index)),					\
			DT_IRQ(PWM(index), priority),				\
			pwm_stm32_isr, DEVICE_DT_INST_GET(index), IRQ_ZERO_LATENCY);		\
	irq_enable(DT_IRQN(PWM(index)));					\
}

#define IRQ_CONFIG_FUNC(index)                                                  \
static void pwm_stm32_irq_config_func_##index(const struct device *dev)		\
{										\
	COND_CODE_1(DT_IRQ_HAS_NAME(PWM(index), cc),				\
		(IRQ_CONNECT_AND_ENABLE_BY_NAME(index, cc)			\
		 IRQ_CONNECT_AND_ENABLE_BY_NAME(index, up)),			\
		(IRQ_CONNECT_AND_ENABLE_DEFAULT(index))				\
	);									\
}

#define DT_INST_CLK(index, inst)                                               \
	{                                                                      \
		.bus = DT_CLOCKS_CELL(PWM(index), bus),				\
		.enr = DT_CLOCKS_CELL(PWM(index), bits)				\
	}

#define PWM_DEVICE_INIT(index)                                                 \
	static struct pwm_stm32_data pwm_stm32_data_##index = {		       \
		.reset = RESET_DT_SPEC_GET(PWM(index)),			       \
	};								       \
									       \
	IRQ_CONFIG_FUNC(index)						       \
									       \
									       \
	static const struct pwm_stm32_config pwm_stm32_config_##index = {      \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(PWM(index)),	       \
		.prescaler = DT_PROP(PWM(index), st_prescaler),		       \
		.countermode = DT_PROP(PWM(index), st_countermode),	       \
		.pclken = DT_INST_CLK(index, timer),                           \
        .sw_gpio = { ARR_INIT(index, gpios) },  \
        .channels = DT_PROP(DT_INST(index, st_stm32_pwm_sw), channels),  \
        .num_gpios = ARR_LEN(index, gpios), \
		.irq_config_func = pwm_stm32_irq_config_func_##index, \
	};                                                                     \
									       \
	DEVICE_DT_INST_DEFINE(index, &pwm_stm32_init, NULL,                    \
			    &pwm_stm32_data_##index,                           \
			    &pwm_stm32_config_##index, POST_KERNEL,            \
			    CONFIG_PWM_INIT_PRIORITY,                          \
			    &pwm_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT)
