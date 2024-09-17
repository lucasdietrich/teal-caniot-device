/*
 * Copyright (c) 2024 Lucas Dietrich <ld.adecy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>


#include <zephyr/logging/log.h>
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

#define USB_CDC_ACM_ALT_NODE DT_NODELABEL(usb_cdc_acm_alt)
#define USB_CDC_ACM_ALT_ENABLED DT_NODE_HAS_STATUS(USB_CDC_ACM_ALT_NODE, okay)

#if USB_CDC_ACM_ALT_ENABLED
#define USB_ALT_BUFFER_SIZE 128u
static const struct device *const dev_cdc_acm_alt = DEVICE_DT_GET(USB_CDC_ACM_ALT_NODE);
uint8_t usb_alt_buffer[USB_ALT_BUFFER_SIZE];
static struct ring_buf usb_alt_ring;
#endif

static void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param)
{
	printk("USB status: %d\n", cb_status);
}

#if USB_CDC_ACM_ALT_ENABLED
void usb_alt_cb(const struct device *dev, void *user_data)
{
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		LOG_DBG("dev %p", dev);

		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int read;
			size_t wrote;
			struct ring_buf *rb = &usb_alt_ring;

			read = uart_fifo_read(dev, buf, sizeof(buf));
			if (read < 0) {
				LOG_ERR("Failed to read UART FIFO");
				read = 0;
			};

			wrote = ring_buf_put(rb, buf, read);
			if (wrote < read) {
				LOG_ERR("Drop %zu bytes", read - wrote);
			}

			LOG_DBG("dev %p send %zu bytes",
				dev, wrote);
			if (wrote) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buf[64];
			size_t wrote, len;

			len = ring_buf_get(&usb_alt_ring, buf, sizeof(buf));
			if (!len) {
				LOG_DBG("dev %p TX buffer empty", dev);
				uart_irq_tx_disable(dev);
			} else {
				wrote = uart_fifo_fill(dev, buf, len);
				LOG_DBG("dev %p wrote len %zu", dev, wrote);
			}
		}
	}
}
#endif

static int usb_init(void)
{
	int ret;

#if USB_CDC_ACM_ALT_ENABLED
	if (!device_is_ready(dev_cdc_acm_alt)) {
		printk("alt CDC ACM device not ready");
		ret = -ENODEV;
		goto exit;
	}
#endif

	ret = usb_enable(usb_status_cb);
	if (ret != 0) {
		printk("Failed to enable USB");
		goto exit;
	}

#if USB_CDC_ACM_ALT_ENABLED
	uint32_t dtr;
	while (1) {
		uart_line_ctrl_get(dev_cdc_acm_alt, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		}

		k_sleep(K_MSEC(100));
	}

	uint32_t baudrate;

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev_cdc_acm_alt, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_DBG("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(dev_cdc_acm_alt, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_DBG("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 1 sec for the host to do all settings */
	k_busy_wait(1000000);

	ret = uart_line_ctrl_get(dev_cdc_acm_alt, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_DBG("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_DBG("Baudrate detected: %d", baudrate);
	}

	ring_buf_init(&usb_alt_ring, sizeof(usb_alt_buffer), usb_alt_buffer);

	uart_irq_callback_set(dev_cdc_acm_alt, usb_alt_cb);
	uart_irq_rx_enable(dev_cdc_acm_alt);
#endif

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

	uint32_t last_tx = k_uptime_get_32();
	uint32_t tx_counter = 0;

	while (1) {
		int ret;
		struct can_frame rx_frame, tx_frame;

		ret = k_msgq_get(&can_msgq, &rx_frame, K_MSEC(500));
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

		uint32_t now = k_uptime_get_32();
		if (now - last_tx > 5000) {
			last_tx = now;

			// Send a CAN frame
			memset(&tx_frame, 0, sizeof(tx_frame));
			tx_frame.id = 0x123; // (1<<3);
			tx_frame.dlc = 8;
			tx_frame.data_32[0] = tx_counter;
			tx_frame.data_32[1] = 0x55667788;
			tx_frame.flags = 0u;

			ret = can_send(dev_can, &tx_frame, K_FOREVER, NULL, NULL);
			if (ret) {
				printk("CAN send failed\n");
			} else {
				printk("CAN frame sent\n");
				tx_counter++;
			}
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

static void mco_init(void)
{
#if defined(CONFIG_STM32L4_MCO)
    /* Should give 80 MHz / 16 = 5 MHz
     */
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16); 
#endif
}

int test_main(void)
{
#if defined(CONFIG_USB_DEVICE_STACK)
	usb_init();
#endif

	can_init();

#if defined(CONFIG_TEST)
	mco_init();
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