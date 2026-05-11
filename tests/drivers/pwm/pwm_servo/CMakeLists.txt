# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Make Zephyr pick up the fake-pwm binding from this test's dts/bindings/.
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pwm_servo_test)

target_sources(app PRIVATE
    src/main.c
    src/fake_pwm.c
)
