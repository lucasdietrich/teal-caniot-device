/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#if defined(CONFIG_TEST)
#include <test.h>
#endif

int main(void)
{
	int ret;

#if defined(CONFIG_TEST)
	ret = test_main();
#endif

	for (;;) {
		printk("hello");
		k_sleep(K_MSEC(1000));
	}

	return 0;
}