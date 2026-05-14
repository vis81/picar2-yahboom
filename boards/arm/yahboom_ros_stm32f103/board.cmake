# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F103RC" "--speed=4000")
board_runner_args(stm32flash "--device=/dev/ttyYahboom0" "--baud-rate=115200" "--start-addr=0x08000000" "--execution-addr=0x08000000")

include(${ZEPHYR_BASE}/boards/common/stm32flash.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
