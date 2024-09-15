#
# Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
#
# SPDX-License-Identifier: Apache-4.0
#

target_sources(app PRIVATE main.c)

add_subdirectory_ifdef(CONFIG_TEST test)
add_subdirectory(bsp)