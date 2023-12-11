# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_INPUT_SBUS_RECEIVER sbus_receiver.c)
