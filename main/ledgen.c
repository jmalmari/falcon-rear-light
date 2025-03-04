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
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define FALCON_REAR_RED_RUN 30
#define FALCON_REAR_RED_BRAKE 255

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

static void fill(led_strip_handle_t led_strip, uint32_t r, uint32_t g, uint32_t b)
{
    for (unsigned ii = 0; ii < LED_STRIP_LED_NUMBERS; ++ii)
    {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, ii, r, g, b));
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

enum panel_state
{
    mostly_black,
    mostly_red,
    other,
};

static enum panel_state get_panel_state(led_rx_msg_t const *panel)
{
    unsigned rgb_avg[3] = {0};
    uint8_t const *rgb = panel->colors;
    unsigned num_leds = panel->count / 3;
    for (unsigned ii = 0; ii < num_leds; ++ii)
    {
        rgb_avg[0] += rgb[0];
        rgb_avg[1] += rgb[1];
        rgb_avg[2] += rgb[2];
        rgb += 3;
    }
    rgb_avg[0] /= num_leds;
    rgb_avg[1] /= num_leds;
    rgb_avg[2] /= num_leds;

    if (rgb_avg[0] <= 1 &&
        rgb_avg[1] <= 1 &&
        rgb_avg[2] <= 1)
    {
        return mostly_black;
    }
    else if (rgb_avg[0] > 1 &&
             rgb_avg[1] <= 1 &&
             rgb_avg[2] <= 1)
    {
        return mostly_red;
    }
    else
    {
        return other;
    }
}

static void apply_effect(led_strip_handle_t led_strip, led_rx_msg_t const *const led_in)
{
    struct panel_snapshot
    {
        float time;
        enum panel_state state;
    };

    static struct panel_snapshot snapshot[3] = {
        {0, mostly_black},
        {0, mostly_black},
        {0, mostly_black},
    };

    enum panel_state new_state = get_panel_state(led_in);

    static float target = FALCON_REAR_RED_RUN;
    static float current = FALCON_REAR_RED_RUN;
    static float t_prev = -1.0f;

    float const t = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0f;

    if (t_prev < 0.0f)
    {
        t_prev = t;
    }

    if (new_state != snapshot[2].state)
    {
        snapshot[0] = snapshot[1];
        snapshot[1] = snapshot[2];
        snapshot[2].time = t;
        snapshot[2].state = new_state;
    }

    // Three constituents of red blinking must be found within 1.5s to
    // maintain solid braking status. This corresponds to stable
    // frequencies higher than 0.667Hz.
    if ((t - snapshot[0].time) < 1.5f &&
        ((snapshot[0].state == mostly_red &&
          snapshot[1].state == mostly_black &&
          snapshot[2].state == mostly_red) ||
         (snapshot[0].state == mostly_black &&
          snapshot[1].state == mostly_red &&
          snapshot[2].state == mostly_black)))
    {
        LEDGEN_GPIO_DBG_ISR_ON();
        target = FALCON_REAR_RED_BRAKE;
    }
    else
    {
        LEDGEN_GPIO_DBG_ISR_OFF();
        target = FALCON_REAR_RED_RUN;
    }

    float const time_constant = 0.3f;
    float const time_constant_slow = 2.0f;
    current += (t - t_prev) / (target < current ? time_constant_slow : time_constant) * (target - current);
    t_prev = t;

    fill(led_strip, current, 0, 0);

    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

static void falcon_idle(led_strip_handle_t led_strip)
{
    float const speed[3] = {
        2 * M_PI * 0.5,
        2 * M_PI * 1.0,
        2 * M_PI * 3.0,
    };
    unsigned const gain_min[3] = {20, 10, 1};
    unsigned const gain_max[3] = {150, 100, 50};

    float t = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0;
    int gain[3];
    for (unsigned ii = 0; ii < 3; ++ii)
    {
        gain[ii] = gain_min[ii] + ((gain_max[ii] - gain_min[ii]) / 2.0f) * (1.0f + sinf(t * speed[ii]));

        if (gain[ii] < gain_min[ii])
        {
            gain[ii] = gain_min[ii];
        }
        else if (gain_max[ii] < gain[ii])
        {
            gain[ii] = gain_max[ii];
        }
    }

    fill_row(led_strip, 0, gain[0], 0, 0);
    fill_row(led_strip, 1, 0, gain[1], 0);
    fill_row(led_strip, 2, 0, 0, gain[2]);

	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(0, 0), gain_max[0], 0, 0));
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(0, LED_PANEL_COLUMNS - 1), gain_min[0], 0, 0));
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(1, 0), 0, gain_max[1], 0));
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(1, LED_PANEL_COLUMNS - 1), 0, gain_min[1], 0));
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(2, 0), 0, 0, gain_max[2]));
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, point_index(2, LED_PANEL_COLUMNS - 1), 0, 0, gain_min[2]));
}

static void falcon_run(led_strip_handle_t led_strip)
{
    fill(led_strip, FALCON_REAR_RED_RUN, 0, 0);
}

static void falcon_brake(led_strip_handle_t led_strip)
{
    unsigned const ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    fill(led_strip, ms % 1000 < 500 ? FALCON_REAR_RED_BRAKE : 0, 0, 0);
}

static void show_idle(led_strip_handle_t led_strip)
{
    enum State
    {
        idle,
        run,
        brake,
        state_count,
    };

    static TickType_t state_entered = 0;
    static enum State state = idle;

    TickType_t const now = xTaskGetTickCount();
    if (!state_entered)
    {
        state_entered = now;
    }

    if (pdMS_TO_TICKS(3000) <= now - state_entered)
    {
        state_entered = now;
        state = (state + 1) % state_count;
    }

    switch (state)
    {
        case idle:
            falcon_idle(led_strip);
            break;
        case run:
            falcon_run(led_strip);
            break;
        case brake:
            falcon_brake(led_strip);
            break;
        case state_count:
            return;
    }

    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
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

    bool idle = false;
    TickType_t const idle_timeout = pdMS_TO_TICKS(35);
    TickType_t last_receive_time = xTaskGetTickCount();

    while (1)
    {
        LEDGEN_GPIO_DBG_OUT_OFF();

        led_rx_msg_t led_in;
        if (xQueueReceive(ctx.led_in_queue, &led_in, pdMS_TO_TICKS(0)))
        {
            LEDGEN_GPIO_DBG_OUT_ON();
            idle = false;
            last_receive_time = xTaskGetTickCount();
            apply_effect(led_strip, &led_in);
        }
        else if (idle || idle_timeout < xTaskGetTickCount() - last_receive_time)
        {
            LEDGEN_GPIO_DBG_OUT_ON();
            idle = true;
            show_idle(led_strip);

            // Slow down a bit. Otherwise the receiver will
            // occasionally lose sync when using this for test input.
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
