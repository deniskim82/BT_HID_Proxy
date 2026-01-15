/**
 * @file ch9329.c
 * @brief CH9329 UART to USB HID controller driver implementation
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
static uint8_t s_frame_buf[CH9329_FRAME_BUF_SIZE];

// Track last sent report to avoid redundant transmissions
static ch9329_keyboard_report_t s_last_report = {0};
static bool s_last_report_valid = false;

/* ============================================================================
 * Private Functions
 * ============================================================================ */

/**
 * @brief Calculate checksum for CH9329 frame
 * @param data Frame data (excluding checksum)
 * @param len Length of data
 * @return Checksum byte
 */
static uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief Build and send CH9329 frame
 * @param cmd Command byte
 * @param data Payload data
 * @param data_len Payload length
 * @return ESP_OK on success
 */
static esp_err_t send_frame(uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Build frame: [HEADER1][HEADER2][ADDR][CMD][LEN][DATA...][CHECKSUM]
    size_t frame_len = 5 + data_len + 1;  // header(2) + addr(1) + cmd(1) + len(1) + data + checksum(1)

    if (frame_len > CH9329_FRAME_BUF_SIZE) {
        ESP_LOGE(TAG, "Frame too large: %d", frame_len);
        return ESP_ERR_INVALID_SIZE;
    }

    s_frame_buf[0] = CH9329_HEADER_1;
    s_frame_buf[1] = CH9329_HEADER_2;
    s_frame_buf[2] = CH9329_ADDR_DEFAULT;
    s_frame_buf[3] = cmd;
    s_frame_buf[4] = data_len;

    if (data_len > 0 && data != NULL) {
        memcpy(&s_frame_buf[5], data, data_len);
    }

    // Calculate checksum over header + addr + cmd + len + data
    s_frame_buf[5 + data_len] = calculate_checksum(s_frame_buf, 5 + data_len);

    // Send frame
    int written = uart_write_bytes(CH9329_UART_NUM, s_frame_buf, frame_len);
    if (written != frame_len) {
        ESP_LOGE(TAG, "UART write failed: %d/%d", written, frame_len);
        return ESP_FAIL;
    }

    // Wait for transmission to complete
    esp_err_t ret = uart_wait_tx_done(CH9329_UART_NUM, pdMS_TO_TICKS(CH9329_RESPONSE_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART TX timeout");
    }

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

    // Install UART driver
    esp_err_t ret = uart_driver_install(CH9329_UART_NUM, 256, 256, 0, NULL, 0);
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

    s_initialized = true;
    s_last_report_valid = false;
    memset(&s_last_report, 0, sizeof(s_last_report));

    ESP_LOGI(TAG, "Initialized (TX:%d, RX:%d, Baud:%d)",
             CH9329_UART_TX_PIN, CH9329_UART_RX_PIN, CH9329_UART_BAUD_RATE);

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

    // Skip if report is identical to last sent (reduces UART traffic)
    if (s_last_report_valid &&
        memcmp(report, &s_last_report, sizeof(ch9329_keyboard_report_t)) == 0) {
        return ESP_OK;
    }

    esp_err_t ret = send_frame(CH9329_CMD_KEYBOARD, (const uint8_t *)report, CH9329_KB_REPORT_SIZE);

    if (ret == ESP_OK) {
        memcpy(&s_last_report, report, sizeof(ch9329_keyboard_report_t));
        s_last_report_valid = true;
    }

    return ret;
}

esp_err_t ch9329_send_keyboard_raw(const uint8_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Skip if data is identical to last sent
    if (s_last_report_valid &&
        memcmp(data, &s_last_report, CH9329_KB_REPORT_SIZE) == 0) {
        return ESP_OK;
    }

    esp_err_t ret = send_frame(CH9329_CMD_KEYBOARD, data, CH9329_KB_REPORT_SIZE);

    if (ret == ESP_OK) {
        memcpy(&s_last_report, data, CH9329_KB_REPORT_SIZE);
        s_last_report_valid = true;
    }

    return ret;
}

esp_err_t ch9329_release_all_keys(void)
{
    ch9329_keyboard_report_t empty_report = {0};

    // Force send even if last report was already empty
    // This ensures clean state after reconnection
    s_last_report_valid = false;

    return ch9329_send_keyboard_report(&empty_report);
}

bool ch9329_is_ready(void)
{
    return s_initialized;
}
