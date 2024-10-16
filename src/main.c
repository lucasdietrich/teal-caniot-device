/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_TEST)
#include <test.h>
#endif

int main(void)
{
	int ret = 0;

#if defined(CONFIG_TEST)
	ret = test_main();
#endif

	for (;;) {
	}

	return ret;
}