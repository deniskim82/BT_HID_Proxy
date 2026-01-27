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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

// LED status callback
static ch9329_led_cb_t s_led_callback = NULL;

// RX task handle
static TaskHandle_t s_rx_task_handle = NULL;
static volatile bool s_rx_task_running = false;

// Last LED status for duplicate detection
static uint8_t s_last_led_status = 0xFF; // Initialize to invalid value

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

void ch9329_set_led_callback(ch9329_led_cb_t cb)
{
    s_led_callback = cb;
}

/**
 * @brief Parse CH9329 response frame for LED status
 *
 * CH9329 sends responses in frame format:
 * [0x57][0xAB][ADDR][CMD][LEN][DATA...][CHECKSUM]
 *
 * Response types:
 * - CMD=0x82: Keyboard ACK (ignore - too frequent)
 * - CMD=0x8D: LED status response (Bit0=Num, Bit1=Caps, Bit2=Scroll)
 */
static void parse_rx_frame(const uint8_t *data, size_t len)
{
    // Minimum frame size: header(2) + addr(1) + cmd(1) + len(1) + checksum(1) = 6
    if (len < 6) {
        ESP_LOGW(TAG, "Frame too short: %d bytes (min 6)", (int)len);
        return;
    }

    // Verify header
    if (data[0] != CH9329_HEADER_1 || data[1] != CH9329_HEADER_2) {
        ESP_LOGW(TAG, "Invalid header: 0x%02X 0x%02X (expected 0x57 0xAB)", data[0], data[1]);
        return;
    }

    uint8_t addr = data[2];
    uint8_t cmd = data[3];
    uint8_t data_len = data[4];

    // Log all received frames (except keyboard ACK which is too frequent)
    if (cmd != CH9329_CMD_KB_ACK) {
        ESP_LOGI(TAG, "RX frame: ADDR=0x%02X CMD=0x%02X LEN=%d", addr, cmd, data_len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
    }

    // Sanity check data_len to prevent issues
    if (data_len > 64 || len < (size_t)(5 + data_len + 1)) {
        ESP_LOGW(TAG, "Invalid data_len: %d (frame_len=%d)", data_len, (int)len);
        return;
    }

    // Ignore keyboard ACK (0x82) - comes for every keystroke, too frequent
    if (cmd == CH9329_CMD_KB_ACK) {
        return;
    }

    // Check for LED status response (0x8D)
    if (cmd == CH9329_CMD_LED_RESPONSE && data_len >= 1) {
        uint8_t led_status = data[5];

        ESP_LOGI(TAG, "LED response: 0x%02X (Num=%d Caps=%d Scroll=%d) last=0x%02X",
                 led_status,
                 (led_status & 0x01) ? 1 : 0,
                 (led_status & 0x02) ? 1 : 0,
                 (led_status & 0x04) ? 1 : 0,
                 s_last_led_status);

        // Only process if changed
        if (led_status != s_last_led_status) {
            s_last_led_status = led_status;

            if (s_led_callback) {
                ESP_LOGI(TAG, "Calling LED callback with status 0x%02X", led_status);
                s_led_callback(led_status);
            } else {
                ESP_LOGW(TAG, "LED callback not registered!");
            }
        } else {
            ESP_LOGI(TAG, "LED status unchanged, skipping callback");
        }
    } else if (cmd != CH9329_CMD_KB_ACK) {
        // Log other unexpected commands
        ESP_LOGI(TAG, "Unhandled CMD: 0x%02X (known: KB_ACK=0x82, LED=0x8D)", cmd);
    }
}

/**
 * @brief UART RX task for receiving LED status from CH9329
 */
static void uart_rx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART RX task started");
    s_rx_task_running = true;

    uint8_t rx_buffer[64];
    size_t rx_pos = 0;
    uint32_t total_bytes_received = 0;  // Debug counter

    while (s_rx_task_running) {
        // Read available data (100ms timeout)
        int len = uart_read_bytes(CH9329_UART_NUM, &rx_buffer[rx_pos],
                                  sizeof(rx_buffer) - rx_pos,
                                  pdMS_TO_TICKS(100));

        if (len > 0) {
            total_bytes_received += len;
            // Log raw received data for debugging
            ESP_LOGI(TAG, "UART RX: %d bytes (total=%lu), pos=%d, raw:",
                     len, (unsigned long)total_bytes_received, (int)rx_pos);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rx_buffer[rx_pos], len, ESP_LOG_INFO);

            rx_pos += len;

            // Try to find and parse complete frames
            size_t i = 0;
            while (i < rx_pos) {
                // Find header
                if (rx_buffer[i] == CH9329_HEADER_1 && i + 1 < rx_pos && rx_buffer[i + 1] == CH9329_HEADER_2) {
                    // Found header, check if we have enough for length field
                    if (i + 5 <= rx_pos) {
                        uint8_t data_len = rx_buffer[i + 4];

                        // Sanity check
                        if (data_len > 64) {
                            i++;
                            continue;
                        }

                        size_t frame_len = 5 + data_len + 1;

                        if (i + frame_len <= rx_pos) {
                            // Complete frame found
                            parse_rx_frame(&rx_buffer[i], frame_len);
                            i += frame_len;
                            continue;
                        } else {
                            // Incomplete frame, wait for more data
                            break;
                        }
                    } else {
                        // Not enough data for length field
                        break;
                    }
                }
                i++;
            }

            // Move remaining data to beginning of buffer
            if (i > 0 && i < rx_pos) {
                memmove(rx_buffer, &rx_buffer[i], rx_pos - i);
                rx_pos -= i;
            } else if (i >= rx_pos) {
                rx_pos = 0;
            }

            // Prevent buffer overflow
            if (rx_pos >= sizeof(rx_buffer) - 16) {
                rx_pos = 0;
            }
        }
    }

    ESP_LOGI(TAG, "UART RX task stopped");
    s_rx_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t ch9329_start_rx_task(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_rx_task_handle != NULL) {
        ESP_LOGW(TAG, "RX task already running");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreate(
        uart_rx_task,
        "ch9329_rx",
        4096,   // Increased stack for safety
        NULL,
        3,  // Priority
        &s_rx_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void ch9329_stop_rx_task(void)
{
    if (s_rx_task_handle == NULL) {
        return;
    }

    s_rx_task_running = false;

    // Wait for task to finish
    int timeout = 20;
    while (s_rx_task_handle != NULL && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout--;
    }
}

esp_err_t ch9329_request_led_status(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "request_led_status: not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Build GET_LED_STATUS command frame
    // [0x57][0xAB][ADDR][CMD][LEN][CHECKSUM]
    uint8_t cmd_frame[6];
    cmd_frame[0] = CH9329_HEADER_1;      // 0x57
    cmd_frame[1] = CH9329_HEADER_2;      // 0xAB
    cmd_frame[2] = CH9329_ADDR_DEFAULT;  // 0x00
    cmd_frame[3] = CH9329_CMD_GET_LED;   // 0x0D
    cmd_frame[4] = 0x00;                 // LEN = 0 (no data)

    // Calculate checksum
    uint8_t sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += cmd_frame[i];
    }
    cmd_frame[5] = sum;

    ESP_LOGI(TAG, "Sending GET_LED_STATUS request:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, cmd_frame, 6, ESP_LOG_INFO);

    int written = uart_write_bytes(CH9329_UART_NUM, cmd_frame, 6);
    if (written != 6) {
        ESP_LOGE(TAG, "Failed to send GET_LED_STATUS: wrote %d/6 bytes", written);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GET_LED_STATUS sent successfully");
    return ESP_OK;
}
