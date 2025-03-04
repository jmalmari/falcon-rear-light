#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "led_rx.h"
#include "ledgen.h"

char const *const TAG = "falcon_ledgen";

#define LED_PANEL_COLUMNS 9
#define LED_PANEL_ROWS 3

#define LED_IN_GPIO 9
#define LED_STRIP_BLINK_GPIO  2
#define LED_STRIP_LED_NUMBERS 32
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

static led_strip_handle_t configure_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
    };

    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

static uint32_t point_index(unsigned row, unsigned column)
{
	if (LED_PANEL_ROWS <= row || LED_PANEL_COLUMNS <= column)
	{
		return 1;
	}
	return row * LED_PANEL_COLUMNS + column;
}

static void fill_row(led_strip_handle_t led_strip, unsigned row, uint32_t r, uint32_t g, uint32_t b)
{
	for (unsigned column = 0; column < LED_PANEL_COLUMNS; ++column)
	{
		ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(row, column), r, g, b));
	}
}

void app_main(void)
{
    led_strip_handle_t led_strip = configure_led();

    ESP_LOGI(TAG, "Start Falcon LED strip");

	led_rx_config_t const rx_config = {
		.gpio_num = LED_IN_GPIO,
		.clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
	};

	led_rx_start(&rx_config);

	float const speed = 2 * M_PI * 1;
	unsigned const gain_min = 20;
	unsigned const gain_max = 150;

    while (1)
	{
		float t = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0;
		int gain = gain_min + ((gain_max - gain_min) / 2.0f) * (1.0f + sinf(t * speed));
		if (gain < gain_min)
		{
			gain = gain_min;
		}
		else if (gain_max < gain)
		{
			gain = gain_max;
		}


		fill_row(led_strip, 0, gain, 3, 6);
		fill_row(led_strip, 1, gain, 0, 0);
		fill_row(led_strip, 2, gain, 0, 0);
		ESP_ERROR_CHECK(led_strip_refresh(led_strip));
		vTaskDelay(pdMS_TO_TICKS(30));
    }
}
