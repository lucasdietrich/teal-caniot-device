/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

#include <caniot/device.h>
#include <test.h>

#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

static struct gpio_dt_spec led_red	 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});
static struct gpio_dt_spec led_blue	 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0});

static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

static int configure_led(struct gpio_dt_spec *led)
{
	int ret = 0;

	if (led->port && !gpio_is_ready_dt(led)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n", ret,
			   led->port->name);
		led->port = NULL;
	}

	if (led->port) {
		ret = gpio_pin_configure_dt(led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n", ret,
				   led->port->name, led->pin);
			led->port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led->port->name, led->pin);
		}
	}

	return ret;
}

static int button_init(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name,
			   button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
			   button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	ret = configure_led(&led_red);
	if (ret != 0) {
		return ret;
	}

	ret = configure_led(&led_green);
	if (ret != 0) {
		return ret;
	}

	ret = configure_led(&led_blue);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param)
{
	printk("USB status: %d\n", cb_status);
}

static int usb_init(void)
{
	int ret;

	ret = usb_enable(usb_status_cb);
	if (ret != 0) {
		printk("Failed to enable USB");
		goto exit;
	}

exit:
	return ret;
}

const struct device *const dev_tcn75 = DEVICE_DT_GET_ONE(microchip_tcn75a);

static void sensor_thread(void *a1, void *a2, void *a3)
{
	ARG_UNUSED(a1);
	ARG_UNUSED(a2);
	ARG_UNUSED(a3);

	if (!device_is_ready(dev_tcn75)) {
		printk("sensor: device not ready.\n");
		return;
	}

	printk("device is %p, name is %s\n", dev_tcn75, dev_tcn75->name);

	for (;;) {
		int ret;
		struct sensor_value temp_val;

		/* Read TCN75 temperature value */
		ret = sensor_sample_fetch(dev_tcn75);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
		}

		ret = sensor_channel_get(dev_tcn75, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
		if (ret) {
			printk("sensor_channel_get failed ret %d\n", ret);
		}

		printk("temperature %.6f\n", sensor_value_to_double(&temp_val));

		k_sleep(K_MSEC(1000));
	}
}

K_THREAD_DEFINE(sensor_thread_id, 1024, sensor_thread, NULL, NULL, NULL, 10, 0, SYS_FOREVER_MS);

const static struct device *dev_can = DEVICE_DT_GET(DT_NODELABEL(can1));

CAN_MSGQ_DEFINE(can_msgq, 2);

static void can_thread(void *a1, void *a2, void *a3)
{
	ARG_UNUSED(a1);
	ARG_UNUSED(a2);
	ARG_UNUSED(a3);

	while (1) {
		int ret;
		struct can_frame rx_frame;

		ret = k_msgq_get(&can_msgq, &rx_frame, K_FOREVER);

		if (ret == 0) {
			printk("CAN frame received\n");
			printk("ID: %d\n", rx_frame.id);
			printk("DLC: %d\n", rx_frame.dlc);
			printk("Data: ");
			for (int i = 0; i < rx_frame.dlc; i++) {
				printk("%d ", rx_frame.data[i]);
			}
			printk("\n");
		}
	}
}

K_THREAD_DEFINE(can_thread_id, 1024, can_thread, NULL, NULL, NULL, 10, 0, 0);

const struct can_filter can_filter = {0};

static int can_init(void)
{
	int ret = 0;

	if (!device_is_ready(dev_can)) {
		printk("CAN device not ready");
		ret = -ENODEV;
		return ret;
	}

	int filter_id = can_add_rx_filter_msgq(dev_can, &can_msgq, &can_filter);
	if (filter_id < 0) {
		LOG_ERR("Unable to add rx msgq [%d]", filter_id);
		return filter_id;
	}

	ret = can_start(dev_can);
	if (ret) {
		LOG_ERR("CAN: Failed to start ret=%d", ret);
	}

	return ret;
}

int main(void)
{
#if defined(CONFIG_USB_DEVICE_STACK)
	usb_init();
#endif

	can_init();

#if defined(CONFIG_TEST)
	test_init();
#endif

	button_init();

	k_thread_start(sensor_thread_id);

	printk("Press the button\n");

	while (1) {
		/* If we have an LED, reflect the button state. */
		if (led_red.port) {
			int val = gpio_pin_get_dt(&button);

			if (val >= 0) {
				gpio_pin_set_dt(&led_red, val);
				gpio_pin_set_dt(&led_green, val);
				gpio_pin_set_dt(&led_blue, !val);
			}
			k_msleep(1);
		}
	}

	return 0;
}
