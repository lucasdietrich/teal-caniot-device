#
# Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0
#
board_runner_args(jlink "--device=stm32l462re" "--iface=swd" "--speed=4000" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)