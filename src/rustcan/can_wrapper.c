#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);


#define CAN_NODE DT_NODELABEL(can1)

#if DT_NODE_HAS_STATUS(CAN_NODE, okay)
const static struct device *dev_can = DEVICE_DT_GET(CAN_NODE);

CAN_MSGQ_DEFINE(can_msgq, 2);

const struct can_filter can_filter = {0};

int can_init(void)
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

	// k_thread_start(can_thread_id);

	return ret;
}

// static void can_thread(void *a1, void *a2, void *a3)
// {
// 	ARG_UNUSED(a1);
// 	ARG_UNUSED(a2);
// 	ARG_UNUSED(a3);

// 	uint32_t last_tx = k_uptime_get_32();
// 	uint32_t tx_counter = 0;

// 	while (1) {
// 		int ret;
// 		struct can_frame rx_frame, tx_frame;

// 		ret = k_msgq_get(&can_msgq, &rx_frame, K_MSEC(500));
// 		if (ret == 0) {
// 			printk("CAN frame received\n");
// 			printk("ID: %d\n", rx_frame.id);
// 			printk("DLC: %d\n", rx_frame.dlc);
// 			printk("Data: ");
// 			for (int i = 0; i < rx_frame.dlc; i++) {
// 				printk("%d ", rx_frame.data[i]);
// 			}
// 			printk("\n");
// 		}

// 		uint32_t now = k_uptime_get_32();
// 		if (now - last_tx > 5000) {
// 			last_tx = now;

// 			// Send a CAN frame
// 			memset(&tx_frame, 0, sizeof(tx_frame));
// 			tx_frame.id = 0x123; // (1<<3);
// 			tx_frame.dlc = 8;
// 			tx_frame.data_32[0] = tx_counter;
// 			tx_frame.data_32[1] = 0x55667788;
// 			tx_frame.flags = 0u;

// 			ret = can_send(dev_can, &tx_frame, K_FOREVER, NULL, NULL);
// 			if (ret) {
// 				printk("CAN send failed\n");
// 			} else {
// 				printk("CAN frame sent\n");
// 				tx_counter++;
// 			}
// 		}
// 	}
// }

// K_THREAD_DEFINE(can_thread_id, 1024, can_thread, NULL, NULL, NULL, 10, 0, SYS_FOREVER_MS);

#endif /* DT_NODE_HAS_STATUS(CAN_NODE, okay) */