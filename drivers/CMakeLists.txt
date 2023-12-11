# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_PWM pwm)
add_subdirectory_ifdef(CONFIG_RC rc)
add_subdirectory_ifdef(CONFIG_SENSOR sensor)
add_subdirectory_ifdef(CONFIG_MOTOR motor)
add_subdirectory_ifdef(CONFIG_INPUT input)
