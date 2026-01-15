/**
 * @file led_status.c
 * @brief RGB LED status indicator implementation
 *
 * Uses RMT peripheral to drive SK6812/WS2812 addressable LED.
 *
 * Status patterns:
 * - IDLE:       Slow red blink (1s period)
 * - SCANNING:   Fast blue blink (200ms period)
 * - PAIRING:    Blue/green alternating (500ms each)
 * - CONNECTING: Slow green blink (1s period)
 * - CONNECTED:  Solid green
 * - ERROR:      Fast red blink (100ms period)
 */

#include "led_status.h"
#include "config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

static const char *TAG = "LED";

/* ============================================================================
 * SK6812/WS2812 Timing Constants (in RMT ticks @ 10MHz)
 * ============================================================================ */

#define LED_RMT_RESOLUTION_HZ   10000000  // 10MHz, 100ns per tick

// SK6812 timing (compatible with WS2812B)
#define T0H_TICKS   3   // 0.3us
#define T0L_TICKS   9   // 0.9us
#define T1H_TICKS   6   // 0.6us
#define T1L_TICKS   6   // 0.6us
#define RESET_TICKS 800 // 80us reset

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static bool s_initialized = false;
static led_status_mode_t s_current_mode = LED_STATUS_OFF;
static rmt_channel_handle_t s_rmt_channel = NULL;
static rmt_encoder_handle_t s_led_encoder = NULL;
static TimerHandle_t s_blink_timer = NULL;
static bool s_blink_state = false;
static uint8_t s_current_r = 0, s_current_g = 0, s_current_b = 0;

/* ============================================================================
 * LED Encoder for RMT
 * ============================================================================ */

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_strip_encoder_t;

static size_t led_encoder_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                  const void *primary_data, size_t data_size,
                                  rmt_encode_state_t *ret_state)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    int state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (led_encoder->state) {
    case 0:  // Encode RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // Fall through
    case 1:  // Send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                 sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }

out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t led_encoder_reset(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t led_encoder_del(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t create_led_encoder(rmt_encoder_handle_t *ret_encoder)
{
    led_strip_encoder_t *led_encoder = calloc(1, sizeof(led_strip_encoder_t));
    if (!led_encoder) {
        return ESP_ERR_NO_MEM;
    }

    led_encoder->base.encode = led_encoder_encode;
    led_encoder->base.reset = led_encoder_reset;
    led_encoder->base.del = led_encoder_del;

    // Bytes encoder for RGB data
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = T0H_TICKS,
            .level1 = 0,
            .duration1 = T0L_TICKS,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = T1H_TICKS,
            .level1 = 0,
            .duration1 = T1L_TICKS,
        },
        .flags.msb_first = 1,
    };

    esp_err_t ret = rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(led_encoder);
        return ret;
    }

    // Copy encoder for reset code
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(led_encoder->bytes_encoder);
        free(led_encoder);
        return ret;
    }

    // Reset code
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = RESET_TICKS,
        .level1 = 0,
        .duration1 = RESET_TICKS,
    };

    *ret_encoder = &led_encoder->base;
    return ESP_OK;
}

/* ============================================================================
 * Private Functions
 * ============================================================================ */

static void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_initialized || !s_rmt_channel || !s_led_encoder) {
        return;
    }

    // SK6812 uses GRB order
    uint8_t grb[3] = {g, r, b};

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    rmt_transmit(s_rmt_channel, s_led_encoder, grb, sizeof(grb), &tx_config);

    // Wait for transmission with timeout (100ms is more than enough for ~110us transmission)
    esp_err_t ret = rmt_tx_wait_all_done(s_rmt_channel, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED transmission timeout or error: %s", esp_err_to_name(ret));
        return;
    }

    s_current_r = r;
    s_current_g = g;
    s_current_b = b;
}

static void blink_timer_callback(TimerHandle_t timer)
{
    s_blink_state = !s_blink_state;

    switch (s_current_mode) {
    case LED_STATUS_IDLE:
        // Slow red blink
        if (s_blink_state) {
            set_led_color(64, 0, 0);  // Dim red
        } else {
            set_led_color(0, 0, 0);
        }
        break;

    case LED_STATUS_SCANNING:
        // Fast blue blink
        if (s_blink_state) {
            set_led_color(0, 0, 128);  // Blue
        } else {
            set_led_color(0, 0, 0);
        }
        break;

    case LED_STATUS_PAIRING:
        // Blue/green alternating
        if (s_blink_state) {
            set_led_color(0, 0, 128);  // Blue
        } else {
            set_led_color(0, 128, 0);  // Green
        }
        break;

    case LED_STATUS_CONNECTING:
        // Slow green blink
        if (s_blink_state) {
            set_led_color(0, 64, 0);  // Dim green
        } else {
            set_led_color(0, 0, 0);
        }
        break;

    case LED_STATUS_ERROR:
        // Fast red blink
        if (s_blink_state) {
            set_led_color(255, 0, 0);  // Bright red
        } else {
            set_led_color(0, 0, 0);
        }
        break;

    default:
        break;
    }
}

static void update_blink_timer(void)
{
    if (!s_blink_timer) {
        return;
    }

    // Stop timer first
    xTimerStop(s_blink_timer, 0);

    uint32_t period_ms = 0;

    switch (s_current_mode) {
    case LED_STATUS_IDLE:
        period_ms = 500;  // 1s total period (500ms on, 500ms off)
        break;
    case LED_STATUS_SCANNING:
        period_ms = 100;  // 200ms total period
        break;
    case LED_STATUS_PAIRING:
        period_ms = 500;  // 500ms each color
        break;
    case LED_STATUS_CONNECTING:
        period_ms = 500;  // 1s total period
        break;
    case LED_STATUS_ERROR:
        period_ms = 50;   // 100ms total period
        break;
    case LED_STATUS_CONNECTED:
        // Solid - no blinking needed
        set_led_color(0, 128, 0);  // Green
        return;
    case LED_STATUS_OFF:
    default:
        set_led_color(0, 0, 0);
        return;
    }

    if (period_ms > 0) {
        xTimerChangePeriod(s_blink_timer, pdMS_TO_TICKS(period_ms), 0);
        s_blink_state = false;
        xTimerStart(s_blink_timer, 0);
        // Trigger first state immediately
        blink_timer_callback(s_blink_timer);
    }
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t led_status_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Configure RMT channel
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = RGB_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_config, &s_rmt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create LED encoder
    ret = create_led_encoder(&s_led_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(s_rmt_channel);
        s_rmt_channel = NULL;
        return ret;
    }

    // Enable RMT channel
    ret = rmt_enable(s_rmt_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(s_led_encoder);
        rmt_del_channel(s_rmt_channel);
        s_led_encoder = NULL;
        s_rmt_channel = NULL;
        return ret;
    }

    // Create blink timer
    s_blink_timer = xTimerCreate(
        "led_blink",
        pdMS_TO_TICKS(500),
        pdTRUE,  // Auto-reload
        NULL,
        blink_timer_callback
    );

    if (!s_blink_timer) {
        ESP_LOGE(TAG, "Failed to create blink timer");
        rmt_disable(s_rmt_channel);
        rmt_del_encoder(s_led_encoder);
        rmt_del_channel(s_rmt_channel);
        s_led_encoder = NULL;
        s_rmt_channel = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    s_current_mode = LED_STATUS_OFF;

    // Start with LED off
    set_led_color(0, 0, 0);

    ESP_LOGI(TAG, "LED status initialized (GPIO %d)", RGB_LED_GPIO);
    return ESP_OK;
}

void led_status_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    // Stop and delete timer
    if (s_blink_timer) {
        xTimerStop(s_blink_timer, portMAX_DELAY);
        xTimerDelete(s_blink_timer, portMAX_DELAY);
        s_blink_timer = NULL;
    }

    // Turn off LED
    set_led_color(0, 0, 0);

    // Clean up RMT
    if (s_rmt_channel) {
        rmt_disable(s_rmt_channel);
    }
    if (s_led_encoder) {
        rmt_del_encoder(s_led_encoder);
        s_led_encoder = NULL;
    }
    if (s_rmt_channel) {
        rmt_del_channel(s_rmt_channel);
        s_rmt_channel = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "LED status de-initialized");
}

void led_status_set(led_status_mode_t mode)
{
    if (!s_initialized) {
        return;
    }

    if (s_current_mode == mode) {
        return;  // No change
    }

    s_current_mode = mode;
    ESP_LOGD(TAG, "LED mode: %d", mode);

    update_blink_timer();
}

led_status_mode_t led_status_get(void)
{
    return s_current_mode;
}

void led_status_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_initialized) {
        return;
    }

    // Stop blink timer when using custom color
    if (s_blink_timer) {
        xTimerStop(s_blink_timer, 0);
    }

    set_led_color(r, g, b);
}

void led_status_off(void)
{
    led_status_set(LED_STATUS_OFF);
}
