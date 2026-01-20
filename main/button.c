/**
 * @file button.c
 * @brief Button input handler implementation
 *
 * M5Stamp Pico button is active LOW (pressed = 0)
 */

#include "button.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "BUTTON";

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static bool s_initialized = false;
static button_callback_t s_callback = NULL;
static TimerHandle_t s_poll_timer = NULL;
static bool s_last_pressed = false;
static int64_t s_press_start_time = 0;
static bool s_long_press_fired = false;

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static bool read_button(void)
{
    // M5Stamp Pico button is active LOW
    return (gpio_get_level(BUTTON_GPIO) == 0);
}

static void poll_timer_callback(TimerHandle_t timer)
{
    bool pressed = read_button();
    int64_t now = esp_timer_get_time();

    if (pressed && !s_last_pressed) {
        // Button just pressed
        s_press_start_time = now;
        s_long_press_fired = false;
    }
    else if (pressed && s_last_pressed) {
        // Button held
        int64_t hold_time_ms = (now - s_press_start_time) / 1000;

        if (!s_long_press_fired && hold_time_ms >= BUTTON_LONG_PRESS_MS) {
            // Long press threshold reached
            s_long_press_fired = true;
            ESP_LOGI(TAG, "Long press detected");

            if (s_callback) {
                s_callback(BUTTON_EVENT_LONG_PRESS);
            }
        }
    }
    else if (!pressed && s_last_pressed) {
        // Button just released
        if (!s_long_press_fired) {
            // Short press (released before long press threshold)
            ESP_LOGD(TAG, "Short press detected");

            if (s_callback) {
                s_callback(BUTTON_EVENT_SHORT_PRESS);
            }
        }
        s_press_start_time = 0;
        s_long_press_fired = false;
    }

    s_last_pressed = pressed;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t button_init(button_callback_t callback)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Configure GPIO
    // Note: GPIO 34-39 are input-only and have no internal pull-up/pull-down.
    // M5Stamp Pico has external pull-up on button GPIO.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // GPIO 39 has no internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,     // We use polling
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    s_callback = callback;
    s_last_pressed = read_button();
    s_press_start_time = 0;
    s_long_press_fired = false;

    // Create poll timer (check every 20ms)
    s_poll_timer = xTimerCreate(
        "btn_poll",
        pdMS_TO_TICKS(20),
        pdTRUE,  // Auto-reload
        NULL,
        poll_timer_callback
    );

    if (s_poll_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timer");
        return ESP_FAIL;
    }

    if (xTimerStart(s_poll_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start timer");
        xTimerDelete(s_poll_timer, 0);
        s_poll_timer = NULL;
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Button handler initialized (GPIO %d)", BUTTON_GPIO);

    return ESP_OK;
}

void button_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    if (s_poll_timer) {
        xTimerStop(s_poll_timer, portMAX_DELAY);
        xTimerDelete(s_poll_timer, portMAX_DELAY);
        s_poll_timer = NULL;
    }

    s_callback = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "Button handler de-initialized");
}

bool button_is_pressed(void)
{
    return read_button();
}

void button_set_callback(button_callback_t callback)
{
    s_callback = callback;
}
