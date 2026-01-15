/**
 * @file ch9329.c
 * @brief CH9329 UART to USB HID controller driver implementation
 *
 * Optimizations applied:
 * - Pre-built frame header (avoid repeated construction)
 * - Pre-computed checksum base for header bytes
 * - Non-blocking UART transmission
 * - uint64_t comparison for keyboard reports
 */

#include "ch9329.h"
#include "config.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "CH9329";

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static bool s_initialized = false;

// Pre-built frame buffer with header already filled
// Layout: [0x57][0xAB][0x00][0x02][0x08][data x 8][checksum]
static uint8_t s_frame_buf[CH9329_FRAME_BUF_SIZE];

// Last sent report for duplicate detection (as uint64_t for fast comparison)
static uint64_t s_last_report = 0;
static bool s_last_report_valid = false;

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Calculate checksum for keyboard data only (header checksum is pre-computed)
 * @param data 8-byte keyboard data
 * @return Full checksum byte
 */
static inline uint8_t calculate_keyboard_checksum(const uint8_t *data)
{
    // Start with pre-computed header checksum base
    uint8_t sum = CH9329_KB_HEADER_CHECKSUM_BASE;

    // Add data bytes (unrolled for speed)
    sum += data[0];
    sum += data[1];
    sum += data[2];
    sum += data[3];
    sum += data[4];
    sum += data[5];
    sum += data[6];
    sum += data[7];

    return sum;
}

/**
 * @brief Send pre-built keyboard frame (optimized fast path)
 * @param data 8-byte keyboard data
 * @return ESP_OK on success
 */
static esp_err_t send_keyboard_frame(const uint8_t *data)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Copy keyboard data to frame buffer (header already set)
    memcpy(&s_frame_buf[5], data, 8);

    // Calculate and set checksum
    s_frame_buf[13] = calculate_keyboard_checksum(data);

    // Send frame (non-blocking - TX buffer handles queuing)
    // Frame size: header(5) + data(8) + checksum(1) = 14 bytes
    int written = uart_write_bytes(CH9329_UART_NUM, s_frame_buf, 14);
    if (written != 14) {
        ESP_LOGE(TAG, "UART write failed: %d/14", written);
        return ESP_FAIL;
    }

    // NOTE: uart_wait_tx_done() removed for performance
    // TX buffer (512 bytes) can hold ~36 frames, more than enough
    // for keyboard input rates (even at 1000Hz polling)

    return ESP_OK;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t ch9329_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // UART configuration
    const uart_config_t uart_config = {
        .baud_rate = CH9329_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver with larger TX buffer for non-blocking writes
    esp_err_t ret = uart_driver_install(CH9329_UART_NUM,
                                        CH9329_UART_RX_BUF_SIZE,
                                        CH9329_UART_TX_BUF_SIZE,
                                        0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(CH9329_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(CH9329_UART_NUM);
        return ret;
    }

    ret = uart_set_pin(CH9329_UART_NUM, CH9329_UART_TX_PIN, CH9329_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(CH9329_UART_NUM);
        return ret;
    }

    // Pre-build frame header (never changes for keyboard commands)
    s_frame_buf[0] = CH9329_HEADER_1;      // 0x57
    s_frame_buf[1] = CH9329_HEADER_2;      // 0xAB
    s_frame_buf[2] = CH9329_ADDR_DEFAULT;  // 0x00
    s_frame_buf[3] = CH9329_CMD_KEYBOARD;  // 0x02
    s_frame_buf[4] = CH9329_KB_REPORT_SIZE; // 0x08

    s_initialized = true;
    s_last_report_valid = false;
    s_last_report = 0;

    ESP_LOGI(TAG, "Initialized (TX:%d, RX:%d, Baud:%d, TxBuf:%d)",
             CH9329_UART_TX_PIN, CH9329_UART_RX_PIN,
             CH9329_UART_BAUD_RATE, CH9329_UART_TX_BUF_SIZE);

    // Send initial release to ensure clean state
    ch9329_release_all_keys();

    return ESP_OK;
}

void ch9329_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    // Release all keys before shutdown
    ch9329_release_all_keys();

    // Wait for TX buffer to flush before deleting driver
    uart_wait_tx_done(CH9329_UART_NUM, pdMS_TO_TICKS(100));

    uart_driver_delete(CH9329_UART_NUM);
    s_initialized = false;
    s_last_report_valid = false;

    ESP_LOGI(TAG, "De-initialized");
}

esp_err_t ch9329_send_keyboard_report(const ch9329_keyboard_report_t *report)
{
    if (report == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ch9329_send_keyboard_raw((const uint8_t *)report);
}

esp_err_t ch9329_send_keyboard_raw(const uint8_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Fast comparison using uint64_t (8 bytes = 1 comparison)
    uint64_t current_report = *(const uint64_t *)data;

    // Skip if report is identical to last sent (reduces UART traffic)
    if (s_last_report_valid && current_report == s_last_report) {
        return ESP_OK;
    }

    esp_err_t ret = send_keyboard_frame(data);

    if (ret == ESP_OK) {
        s_last_report = current_report;
        s_last_report_valid = true;
    }

#if DEBUG_VERBOSE
    ESP_LOGD(TAG, "KB: %02X %02X %02X %02X %02X %02X %02X %02X",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
#endif

    return ret;
}

esp_err_t ch9329_release_all_keys(void)
{
    static const uint8_t empty_report[8] = {0};

    // Force send even if last report was already empty
    // This ensures clean state after reconnection
    s_last_report_valid = false;

    return ch9329_send_keyboard_raw(empty_report);
}

bool ch9329_is_ready(void)
{
    return s_initialized;
}
