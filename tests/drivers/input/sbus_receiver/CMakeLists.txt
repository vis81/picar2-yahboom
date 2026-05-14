# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sbus_receiver_test)

# input_sbus.c includes pinctrl.h but doesn't use pinctrl; provide a stub for native_sim
zephyr_include_directories(include)

target_sources(app PRIVATE
	src/main.c
)
