# CAN Issue

Its not possible to get rid of the `__bindgen_anon_1` for the `union` field of the `can_frame` 
with `payload` to have a nice name is rust because it would break the zephyr code:

E.g. having

```
struct can_frame {
    // ...
	union {
		/** Payload data accessed as unsigned 8 bit values. */
		uint8_t data[CAN_MAX_DLEN];
		/** Payload data accessed as unsigned 32 bit values. */
		uint32_t data_32[DIV_ROUND_UP(CAN_MAX_DLEN, sizeof(uint32_t))];
	} payload;
}
```

breaks `can_stm32_bxcan.c` and more:

```
/home/lucas/zephyrproject/zephyr/drivers/can/can_stm32_bxcan.c: In function 'can_stm32_rx_fifo_pop':
/home/lucas/zephyrproject/zephyr/drivers/can/can_stm32_bxcan.c:104:22: error: 'struct can_frame' has no member named 'data_32'
  104 |                 frame->data_32[0] = mbox->RDLR;
      |                      ^~
/home/lucas/zephyrproject/zephyr/drivers/can/can_stm32_bxcan.c:105:22: error: 'struct can_frame' has no member named 'data_32'
  105 |                 frame->data_32[1] = mbox->RDHR;
      |                      ^~
```