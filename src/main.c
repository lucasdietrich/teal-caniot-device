/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <test.h>

int main(void)
{
	int ret;

#if defined(CONFIG_TEST)
	ret = test_main();
#endif

	for (;;) {
	}

	return 0;
}