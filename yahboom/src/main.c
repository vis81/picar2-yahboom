/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/version.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/rc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/uart_pipe.h>
#include "zephyr/drivers/motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/input/input.h>
//#include <unistd.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

#include "battery.h"
#include "buzzer.h"
#include "comms.h"
#include "imu.h"
#include "motor.h"
#include "pi.h"
#include "power.h"
#include "rc.h"
#include "servo.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* STM32F103 system memory (ROM bootloader) address */
#define STM32_SYSTEM_MEMORY 0x1FFFF000U

/* User push button, PD2 — the wake source for STOP mode. */
static const struct gpio_dt_spec wake_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

void system_peripherals_down(void)
{
	rc_set_enable(0);
	motor_stop_all();
	servo_neutral_all();
	buzzer_stop();
	imu_shutdown();
}

void system_shutdown(void)
{
	system_peripherals_down();
	power_standby();
}

/* Returns only on failure — a successful STOP is exited by rebooting. */
int system_sleep(void)
{
	system_peripherals_down();
	return power_stop();
}

void power_standby(void)
{
	__disable_irq();
	SysTick->CTRL = 0;

	/* STM32F103 STANDBY: all I/O tri-stated, ~2 µA */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_CWUF;
	PWR->CR |= PWR_CR_PDDS;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
}

int power_stop(void)
{
	int rc;

	if (!gpio_is_ready_dt(&wake_button)) {
		return -ENODEV;
	}

	/* Pull-up because the button shorts to ground and the board has no
	 * external one. Retained through STOP, unlike STANDBY, so it only
	 * draws while the button is actually held. */
	rc = gpio_pin_configure_dt(&wake_button, GPIO_INPUT | GPIO_PULL_UP);
	if (rc) {
		return rc;
	}

	/* EXTI2. Configured while the kernel is still up; no callback is
	 * needed since the wake path reboots rather than resuming. */
	rc = gpio_pin_interrupt_configure_dt(&wake_button, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc) {
		return rc;
	}

	__disable_irq();
	SysTick->CTRL = 0;

	/* STM32F103 STOP with the regulator in low-power mode, ~20 µA. Unlike
	 * STANDBY this keeps I/O driving, so GLOBAL_EN stays asserted and the
	 * Pi remains gated for the whole sleep. PRIMASK is set, so the button
	 * wakes the core from WFI without running the ISR. */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_CWUF;
	PWR->CR &= ~PWR_CR_PDDS;
	PWR->CR |= PWR_CR_LPDS;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();

	/* Woken. The F1 resumes on HSI at 8 MHz with HSE and PLL off, so every
	 * clocked peripheral — UART baud, PWM, timers — is now wrong. Zephyr
	 * has no PM support for STM32F1 to restore the clock tree, so reboot
	 * instead of resuming. RAM state is lost, which is what STANDBY would
	 * have done anyway. */
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	sys_reboot(SYS_REBOOT_COLD);

	CODE_UNREACHABLE;
}

static void jump_to_system_bootloader(void)
{
	uint32_t msp = *(volatile uint32_t *)STM32_SYSTEM_MEMORY;
	uint32_t pc  = *(volatile uint32_t *)(STM32_SYSTEM_MEMORY + 4U);

	/* Disable SysTick */
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;

	/* Disable all IRQs and clear pending */
	for (int i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFFU;
		NVIC->ICPR[i] = 0xFFFFFFFFU;
	}

	/* Reset all APB peripherals so the ROM bootloader gets a clean USART1 */
	RCC->APB1RSTR = 0xFFFFFFFFU;
	RCC->APB2RSTR = 0xFFFFFFFFU;
	RCC->APB1RSTR = 0x00000000U;
	RCC->APB2RSTR = 0x00000000U;

	/* Point vector table at system memory */
	SCB->VTOR = STM32_SYSTEM_MEMORY;

	__DSB();
	__ISB();

	__set_MSP(msp);
	((void (*)(void))pc)();
}

#ifdef CONFIG_SHELL
static int cmd_sys_bootloader(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "entering STM32 bootloader");
	k_msleep(50);
	jump_to_system_bootloader();
	return 0;
}

static int cmd_sys_reboot(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "rebooting");
	k_msleep(50);
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

static int cmd_sys_halt(const struct shell *sh, size_t argc, char **argv)
{
	/* Kept for bench comparison against sys sleep; STANDBY's 2 µA is
	 * meaningless next to the ~49 mA board floor, so this is not the
	 * command to reach for in normal use. */
	shell_warn(sh, "STANDBY tri-states all I/O — GLOBAL_EN releases and the Pi will boot");
	shell_print(sh, "entering standby — reset pin or power cycle to wake");
	k_msleep(50);
	system_shutdown();
	return 0;
}

static int cmd_sys_sleep(const struct shell *sh, size_t argc, char **argv)
{
	if (pi_is_on()) {
		shell_error(sh, "Pi is running — shut it down first "
			        "(no SHUTDOWN_REQ wire yet; 'pi off' hard-cuts it)");
		return -EBUSY;
	}

	shell_print(sh, "entering STOP — press the user button to wake");
	k_msleep(50);

	int rc = system_sleep();

	/* Only reached if STOP was never entered. */
	shell_error(sh, "STOP not entered (%d)", rc);
	return rc;
}

static int cmd_sys_uptime(const struct shell *sh, size_t argc, char **argv)
{
	int64_t ms = k_uptime_get();
	shell_print(sh, "%lld d %02lld:%02lld:%02lld.%03lld",
		ms / 86400000,
		ms / 3600000 % 24,
		ms / 60000   % 60,
		ms / 1000    % 60,
		ms           % 1000);
	return 0;
}

static int cmd_sys_version(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Zephyr  %s", KERNEL_VERSION_STRING);
	shell_print(sh, "app     %s", APP_GIT_SHA);
	shell_print(sh, "built   %s %s", __DATE__, __TIME__);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sys,
	SHELL_CMD(bootloader, NULL, "reboot into STM32 ROM bootloader", cmd_sys_bootloader),
	SHELL_CMD(halt,       NULL, "enter STANDBY (~2 µA); releases the Pi gate — prefer sleep", cmd_sys_halt),
	SHELL_CMD(sleep,      NULL, "enter STOP (~20 µA); user button (PD2) wakes it", cmd_sys_sleep),
	SHELL_CMD(reboot,     NULL, "reboot the system", cmd_sys_reboot),
	SHELL_CMD(uptime,     NULL, "print time since boot", cmd_sys_uptime),
	SHELL_CMD(version,    NULL, "print kernel version", cmd_sys_version),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sys, &sub_sys, "system commands", NULL);
#endif



int main(void)
{
	printk("Yahboom demo\n");
	buzzer_init();
	battery_init();
	imu_init();
	motor_init();
	servo_init();
	rc_init();
	pi_init();
	comms_init();

	//buzzer_play(BUZZER_FUNKYTOWN, 50);

	return 0;
}
