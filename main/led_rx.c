#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_rx.h"
#include "led_rx.h"
#include "esp_log.h"
#include "ledgen.h"

#define LED_STRIP_RMT_DEFAULT_RESOLUTION 10000000 // 10MHz resolution

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS 64
#else
#define LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS 48
#endif

led_rx_receive_fn on_receive = 0;

static bool on_rmt_rx_done(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_ctx;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}

static void receive_task(void *arg)
{
	rmt_channel_handle_t rx_chan = (rmt_channel_handle_t)arg;

	rmt_receive_config_t rmt_rx_config = {
		.signal_range_min_ns = 75,
		.signal_range_max_ns = 1500,
	};

	QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
	rmt_rx_event_callbacks_t cbs = {
		.on_recv_done = on_rmt_rx_done,
	};
	ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, receive_queue));

	ESP_ERROR_CHECK(rmt_enable(rx_chan));

	while(1)
 	{
		static rmt_symbol_word_t raw_symbols[32 * 24 + 2]; // 64 symbols should be sufficient for a standard NEC frame
		// ready to receive
		ESP_ERROR_CHECK(rmt_receive(rx_chan, raw_symbols, sizeof(raw_symbols), &rmt_rx_config));
		// wait for the RX-done signal
		rmt_rx_done_event_data_t rx_data;
		xQueueReceive(receive_queue, &rx_data, portMAX_DELAY);

		static led_rx_msg_t result;

		unsigned const num_leds = rx_data.num_symbols / 24;
		result.count = num_leds * 3;

		for (unsigned color3 = 0; color3 < num_leds; ++color3)
		{
			rmt_symbol_word_t const *symbol = rx_data.received_symbols + color3 * 24;

			// order: green red blue
			uint8_t grb[3] = {0};
			for (unsigned component = 0; component < 3; ++component)
			{
				for (unsigned bit_no = 0; bit_no < 8; ++bit_no)
				{
					grb[component] |= (symbol->duration0 > 2 * symbol->duration1 ? (1 << (7 - bit_no)) : 0);
					++symbol;
				}
			}

			if (color3 < sizeof(result.colors) / 3)
			{
				result.colors[color3 * 3 + 0] = grb[1];
				result.colors[color3 * 3 + 1] = grb[0];
				result.colors[color3 * 3 + 2] = grb[2];
			}
		}

		if (on_receive)
		{
			on_receive(&result);
		}

#undef LED_TRACE
#ifdef LED_TRACE
		{
			unsigned rgb_total[3] = {0};
			for (unsigned color3 = 0; color3 < num_leds; ++color3)
			{
				rgb_total[0] += result.colors[color3 * 3 + 0];
				rgb_total[1] += result.colors[color3 * 3 + 1];
				rgb_total[2] += result.colors[color3 * 3 + 2];
			}
			ESP_LOGI(TAG, "Got %u: %u, %u, %u",
					 num_leds,
					 (unsigned)rgb_total[0] / num_leds,
					 (unsigned)rgb_total[1] / num_leds,
					 (unsigned)rgb_total[2] / num_leds);
		}
#endif
	}
}

void led_rx_start(led_rx_config_t const *config)
{
	rmt_rx_channel_config_t rx_chan_config = {
		.gpio_num = config->gpio_num,
		/*!< Clock source of RMT RX channel, channels in the same group must use the same clock source */
		.clk_src = config->clk_src ? config->clk_src : RMT_CLK_SRC_DEFAULT,
	    .resolution_hz = config->resolution_hz ? config->resolution_hz : LED_STRIP_RMT_DEFAULT_RESOLUTION,
		.mem_block_symbols = config->mem_block_symbols ? config->mem_block_symbols : LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS,
		.intr_priority = 0,
		.flags = {
			.invert_in = false,
			.with_dma = 0, /*!< If set, the driver will allocate an RMT channel with DMA capability */
			.io_loop_back = 0, /*!< For debug/test, the signal output from the GPIO will be fed to the input path as well */
			.backup_before_sleep = 0,
		},
	};

	static rmt_channel_handle_t rx_chan = NULL;

	on_receive = config->on_receive;

	ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &rx_chan));

	xTaskCreate(receive_task, "led_rx", 2048, rx_chan, 10, NULL);
}
