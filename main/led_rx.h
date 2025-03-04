#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct led_rx_msg {
	uint8_t colors[32 * 3];
	unsigned count;
} led_rx_msg_t;

typedef void (*led_rx_receive_fn)(led_rx_msg_t const *msg);

typedef struct led_rx_config {
	gpio_num_t gpio_num;
    rmt_clock_source_t clk_src; /*!< RMT clock source */
    uint32_t resolution_hz;     /*!< RMT tick resolution, if set to zero, a default resolution (10MHz) will be applied */
    size_t mem_block_symbols;   /*!< How many RMT symbols can one RMT channel hold at one time. Set to 0 will fallback to use the default size. */
	led_rx_receive_fn on_receive;
} led_rx_config_t;

void led_rx_start(struct led_rx_config const *config);
