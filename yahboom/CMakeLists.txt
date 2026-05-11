# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(servo_motor)

execute_process(
    COMMAND git -C ${CMAKE_SOURCE_DIR} rev-parse --short HEAD
    OUTPUT_VARIABLE APP_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT APP_GIT_SHA)
    set(APP_GIT_SHA "unknown")
endif()

target_sources(app PRIVATE src/main.c src/buzzer.c src/battery.c src/motor.c src/rc.c src/servo.c src/control.c src/imu.c src/selftest.c)
target_compile_definitions(app PRIVATE APP_GIT_SHA="${APP_GIT_SHA}")
