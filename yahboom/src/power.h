/*
 * Copyright (c) 2024 Valentyn Shevchenko
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _power_h_
#define _power_h_

/* Enter STM32F103 STANDBY mode (~2 µA); only a reset or power cycle wakes it. */
void power_standby(void);

#endif
