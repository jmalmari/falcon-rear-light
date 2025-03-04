#pragma once

typedef struct led_rx_config {
	gpio_num_t gpio_num;
    rmt_clock_source_t clk_src; /*!< RMT clock source */
    uint32_t resolution_hz;     /*!< RMT tick resolution, if set to zero, a default resolution (10MHz) will be applied */
    size_t mem_block_symbols;   /*!< How many RMT symbols can one RMT channel hold at one time. Set to 0 will fallback to use the default size. */
} led_rx_config_t;

void led_rx_start(struct led_rx_config const *config);
