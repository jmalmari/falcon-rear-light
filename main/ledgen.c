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

#ifdef LEDGEN_DBG_GPIO
static void ledgen_dbg_gpio_init()
{
	gpio_num_t const pins[] = {
		LEDGEN_GPIO_DBG_ISR,
		LEDGEN_GPIO_DBG_IN,
		LEDGEN_GPIO_DBG_OUT,
	};

	for (unsigned ii = 0; ii < sizeof(pins) / sizeof(pins[0]); ++ii)
	{
		gpio_reset_pin(pins[ii]);
		gpio_set_level(pins[ii], 0);
		gpio_set_direction(pins[ii], GPIO_MODE_OUTPUT);
	}
}
#endif

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

struct ledgen
{
	QueueHandle_t led_in_queue;
};

static struct ledgen ctx = {0};

static void ledgen_init()
{
	ctx.led_in_queue = xQueueCreate(1, sizeof(led_rx_msg_t));

#ifdef LEDGEN_DBG_GPIO
	ledgen_dbg_gpio_init();
#endif
}

static void on_led_in(led_rx_msg_t const *msg)
{
	if (ctx.led_in_queue)
	{
		if (!xQueueSend(ctx.led_in_queue, msg, 0))
		{
			ESP_LOGI(TAG, "LED in overflow");
		}
	}
}

void app_main(void)
{
	ledgen_init();

    led_strip_handle_t led_strip = configure_led();

    ESP_LOGI(TAG, "Start Falcon LED strip");

	led_rx_config_t const rx_config = {
		.gpio_num = LED_IN_GPIO,
		.clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
		.on_receive = on_led_in,
	};

	led_rx_start(&rx_config);

	float const speed = 2 * M_PI * 1;
	unsigned const gain_min = 20;
	unsigned const gain_max = 150;

    while (1)
	{
		LEDGEN_GPIO_DBG_OUT_OFF();
		led_rx_msg_t led_in;
		if (xQueueReceive(ctx.led_in_queue, &led_in, pdMS_TO_TICKS(35)))
		{
			LEDGEN_GPIO_DBG_OUT_ON();
			uint8_t const *rgb = led_in.colors;
			for (unsigned ii = 0; ii < led_in.count / 3; ++ii)
			{
				uint8_t gray = ((unsigned)rgb[0] + (unsigned)rgb[1] + (unsigned)rgb[2]) / 3u;

				ESP_ERROR_CHECK(led_strip_set_pixel(
									led_strip, ii,
									gray,
									gray,
									gray));
				rgb += 3;
			}
		}
		else
		{
			LEDGEN_GPIO_DBG_OUT_ON();
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

			fill_row(led_strip, 0, 0, gain, 0);
			fill_row(led_strip, 1, 0, gain, 0);
			fill_row(led_strip, 2, 0, gain, 0);
		}
		ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    }
}
