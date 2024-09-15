/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test.h"

#include <zephyr/kernel.h>

void test_init(void)
{
#if defined(CONFIG_STM32L4_MCO)
    /* Should give 80 MHz / 16 = 5 MHz
     */
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16); 
#endif
}