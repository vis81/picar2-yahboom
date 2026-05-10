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
#include <zephyr/drivers/input/sbusreceiver.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/rc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/uart_pipe.h>
#include "zephyr/drivers/pwm_servo.h"
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
#include "control.h"
#include "imu.h"
#include "motor.h"
#include "rc.h"
#include "servo.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* STM32F103 system memory (ROM bootloader) address */
#define STM32_SYSTEM_MEMORY 0x1FFFF000U

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
	shell_print(sh, "entering standby — reset pin or power cycle to wake");
	k_msleep(50);

	__disable_irq();
	SysTick->CTRL = 0;

	/* STM32F103 STANDBY: all I/O tri-stated, ~2 µA */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_CWUF;            /* clear wake-up flag */
	PWR->CR |= PWR_CR_PDDS;            /* standby (not stop) on deep sleep */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	return 0;
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
	SHELL_CMD(halt,       NULL, "enter STANDBY (~2 µA); reset pin or power cycle to wake", cmd_sys_halt),
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
	control_init();

	//buzzer_play(BUZZER_FUNKYTOWN, 50);

	return 0;
}
