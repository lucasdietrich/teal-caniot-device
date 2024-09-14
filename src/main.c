/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <caniot/device.h>

#include <test.h>

static struct caniot_device device;

void myfunc(uint32_t counter)
{
	printf("Counter: %d\n", counter);
	k_sleep(K_MSEC(1000));
}

int main(void)
{
	printk("Hello from teal device!\n");

	// caniot_app_init(&device);

#if defined(CONFIG_TEST)
	test_init();
#endif

	uint32_t counter = 0;

	for (;;) {
		counter++;
		myfunc(counter);
	}
}