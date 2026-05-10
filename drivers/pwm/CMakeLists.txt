# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PWM_STM32_SW	pwm_stm32_sw.c)
zephyr_library_sources_ifdef(CONFIG_PWM_SERVO pwm_servo.c)

