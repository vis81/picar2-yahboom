# Copyright (c) 2022 Jonathan Hahn
# SPDX-License-Identifier: Apache-2.0

# zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/pwm.h)

zephyr_include_directories(.)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RC_SBUS 
    sbus_receiver.c sbus_parser.c)
