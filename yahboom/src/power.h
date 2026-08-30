/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _power_h_
#define _power_h_

/* Enter STM32F103 STANDBY mode (~2 µA); only a reset or power cycle wakes it. */
void power_standby(void);

/* Enter STM32F103 STOP mode (~20 µA), woken by the user button on PD2. Costs
 * 18 µA more than STANDBY — nothing against the ~49 mA board floor — and buys
 * an EXTI wake source plus retained I/O state, so GLOBAL_EN keeps the Pi gated
 * while asleep. Waking reboots rather than resuming, because the F1 comes back
 * on HSI with HSE and PLL off and Zephyr has no PM support for this SoC to
 * restore the clock tree. Returns only on failure to enter STOP. */
int power_stop(void);

/* Orderly shutdown: RC → motors → servos → buzzer → IMU → STANDBY. */
void system_shutdown(void);

/* As system_shutdown(), but ends in STOP so the button can wake it. */
int system_sleep(void);

#endif
