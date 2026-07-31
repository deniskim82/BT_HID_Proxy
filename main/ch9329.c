/**
 * @file ch9329.c
 * @brief CH9329 UART to USB HID controller driver implementation
 *
 * Single-writer design: every outgoing frame is queued to one TX task, so a
 * keyboard frame can never be interleaved with a control frame (that was a
 * source of dropped keys in the previous implementation).
 */

#include "ch9329.h"
#include "config.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "CH9329";

/* ============================================================================
 * Private Types & Variables
 * ============================================================================ */

typedef enum {
    TX_MSG_KB_REPORT = 0,   // 8-byte keyboard report
    TX_MSG_RELEASE_ALL,     // Force-send empty report, reset dedup state
    TX_MSG_LED_REQUEST,     // GET_LED_STATUS command
} tx_msg_type_t;

typedef struct {
    tx_msg_type_t type;
    uint8_t data[8];
} tx_msg_t;

static bool s_initialized = false;
static QueueHandle_t s_tx_queue = NULL;
static TaskHandle_t s_tx_task_handle = NULL;
static TaskHandle_t s_rx_task_handle = NULL;
static volatile bool s_tasks_running = false;

static ch9329_led_cb_t s_led_callback = NULL;

/* ============================================================================
 * TX Task
 * ============================================================================ */

static void send_frame(uint8_t cmd, const uint8_t *data, uint8_t data_len)
{
    uint8_t frame[5 + 8 + 1];

    frame[0] = CH9329_HEADER_1;
    frame[1] = CH9329_HEADER_2;
    frame[2] = CH9329_ADDR_DEFAULT;
    frame[3] = cmd;
    frame[4] = data_len;

    uint8_t sum = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    for (uint8_t i = 0; i < data_len; i++) {
        frame[5 + i] = data[i];
        sum += data[i];
    }
    frame[5 + data_len] = sum;

    int len = 5 + data_len + 1;
    int written = uart_write_bytes(CH9329_UART_NUM, frame, len);
    if (written != len) {
        ESP_LOGE(TAG, "UART write failed: %d/%d", written, len);
    }
}

static void tx_task(void *pvParameters)
{
    tx_msg_t msg;
    uint8_t last_report[8] = {0};
    bool last_report_valid = false;
    static const uint8_t empty_report[8] = {0};

    // Diagnostics: confirm the first few frames actually reach the wire
    int log_budget = 10;

    while (s_tasks_running) {
        if (xQueueReceive(s_tx_queue, &msg, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }

        switch (msg.type) {
        case TX_MSG_KB_REPORT:
            // Suppress exact duplicates to reduce UART traffic
            if (last_report_valid && memcmp(msg.data, last_report, 8) == 0) {
                break;
            }
            send_frame(CH9329_CMD_KEYBOARD, msg.data, 8);
            if (log_budget > 0) {
                log_budget--;
                // uart_write_bytes only queues into the TX ring buffer, so it
                // succeeding proves nothing about the wire. Draining the FIFO
                // does: if this returns ESP_OK the bytes were physically
                // clocked out of the TX pin, which moves the fault outside
                // the ESP32 entirely. Only done for these first few frames -
                // the hot path never waits.
                esp_err_t drain = uart_wait_tx_done(CH9329_UART_NUM, pdMS_TO_TICKS(100));
                ESP_LOGI(TAG, "TX KB: %02X %02X %02X %02X %02X %02X %02X %02X (drain=%s)",
                         msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                         msg.data[4], msg.data[5], msg.data[6], msg.data[7],
                         esp_err_to_name(drain));
            }
            memcpy(last_report, msg.data, 8);
            last_report_valid = true;
            break;

        case TX_MSG_RELEASE_ALL:
            send_frame(CH9329_CMD_KEYBOARD, empty_report, 8);
            memcpy(last_report, empty_report, 8);
            last_report_valid = true;
            break;

        case TX_MSG_LED_REQUEST:
            send_frame(CH9329_CMD_GET_LED, NULL, 0);
            break;
        }
    }

    s_tx_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
 * RX Task
 * ============================================================================ */

/**
 * @brief Parse a complete CH9329 response frame
 *
 * Frame: [0x57][0xAB][ADDR][CMD][LEN][DATA...][CHECKSUM]
 */
// Diagnostics: log the first few RX frames, including the keyboard ACKs that
// are normally silent. Receiving an ACK proves the CH9329 is powered, at the
// right baud rate, and actually receiving our frames - which is otherwise
// impossible to tell apart from a dead TX line.
static int s_rx_log_budget = 10;

static void parse_rx_frame(const uint8_t *data, size_t len)
{
    uint8_t cmd = data[3];
    uint8_t data_len = data[4];

    if (s_rx_log_budget > 0) {
        s_rx_log_budget--;
        ESP_LOGI(TAG, "RX frame: CMD=0x%02X LEN=%d%s", cmd, data_len,
                 cmd == CH9329_CMD_KB_ACK ? " (keyboard ACK)" : "");
    }

    // Verify checksum
    uint8_t sum = 0;
    for (size_t i = 0; i < len - 1; i++) {
        sum += data[i];
    }
    if (sum != data[len - 1]) {
        ESP_LOGD(TAG, "RX checksum mismatch (cmd=0x%02X)", cmd);
        return;
    }

    // Keyboard ACK arrives for every keystroke - ignore silently
    if (cmd == CH9329_CMD_KB_ACK) {
        return;
    }

    if (cmd == CH9329_CMD_LED_RESPONSE && data_len >= 1) {
        static uint8_t s_last_led = 0xFF;
        uint8_t led_status = data[5];

        if (led_status != s_last_led) {
            s_last_led = led_status;
            ESP_LOGD(TAG, "LED status: 0x%02X", led_status);
            if (s_led_callback) {
                s_led_callback(led_status);
            }
        }
        return;
    }

    ESP_LOGD(TAG, "RX frame: CMD=0x%02X LEN=%d", cmd, data_len);
}

static void rx_task(void *pvParameters)
{
    uint8_t rx_buffer[128];
    size_t rx_pos = 0;

    while (s_tasks_running) {
        int len = uart_read_bytes(CH9329_UART_NUM, &rx_buffer[rx_pos],
                                  sizeof(rx_buffer) - rx_pos,
                                  pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        // Raw dump too: a baud rate mismatch produces garbage that never
        // forms a valid frame, so it would otherwise be invisible.
        if (s_rx_log_budget > 0) {
            ESP_LOGI(TAG, "RX %d bytes:", len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rx_buffer[rx_pos], len, ESP_LOG_INFO);
        }

        rx_pos += len;

        // Extract complete frames
        size_t i = 0;
        while (i < rx_pos) {
            if (rx_buffer[i] != CH9329_HEADER_1) {
                i++;
                continue;
            }
            if (i + 1 >= rx_pos) {
                break;  // Need more data
            }
            if (rx_buffer[i + 1] != CH9329_HEADER_2) {
                i++;
                continue;
            }
            if (i + 5 > rx_pos) {
                break;  // Header incomplete
            }

            uint8_t data_len = rx_buffer[i + 4];
            if (data_len > 64) {
                i++;  // Implausible length - resync
                continue;
            }

            size_t frame_len = 5 + data_len + 1;
            if (i + frame_len > rx_pos) {
                break;  // Frame incomplete
            }

            parse_rx_frame(&rx_buffer[i], frame_len);
            i += frame_len;
        }

        // Compact buffer
        if (i > 0) {
            if (i < rx_pos) {
                memmove(rx_buffer, &rx_buffer[i], rx_pos - i);
                rx_pos -= i;
            } else {
                rx_pos = 0;
            }
        }

        // Overflow guard: drop stale partial data
        if (rx_pos >= sizeof(rx_buffer) - 8) {
            rx_pos = 0;
        }
    }

    s_rx_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
 * Queue Helpers
 * ============================================================================ */

static esp_err_t enqueue_msg(const tx_msg_t *msg)
{
    if (!s_initialized || s_tx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Short timeout: with a 32-deep queue this only blocks under pathological
    // conditions; never called from ISR context.
    if (xQueueSend(s_tx_queue, msg, pdMS_TO_TICKS(20)) != pdTRUE) {
        ESP_LOGW(TAG, "TX queue full, dropping msg type %d", msg->type);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t ch9329_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    const uart_config_t uart_config = {
        .baud_rate = CH9329_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(CH9329_UART_NUM,
                                        CH9329_UART_RX_BUF_SIZE,
                                        CH9329_UART_TX_BUF_SIZE,
                                        0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(CH9329_UART_NUM, &uart_config);
    if (ret == ESP_OK) {
        ret = uart_set_pin(CH9329_UART_NUM, CH9329_UART_TX_PIN, CH9329_UART_RX_PIN,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(ret));
        uart_driver_delete(CH9329_UART_NUM);
        return ret;
    }

    s_tx_queue = xQueueCreate(CH9329_TX_QUEUE_LEN, sizeof(tx_msg_t));
    if (s_tx_queue == NULL) {
        uart_driver_delete(CH9329_UART_NUM);
        return ESP_ERR_NO_MEM;
    }

    s_tasks_running = true;

    if (xTaskCreate(tx_task, "ch9329_tx", 3072, NULL, 10, &s_tx_task_handle) != pdPASS ||
        xTaskCreate(rx_task, "ch9329_rx", 3072, NULL, 3, &s_rx_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create tasks");
        s_tasks_running = false;
        vTaskDelay(pdMS_TO_TICKS(200));
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
        uart_driver_delete(CH9329_UART_NUM);
        return ESP_FAIL;
    }

    s_initialized = true;

    ESP_LOGI(TAG, "Initialized (TX:%d RX:%d baud:%d)",
             CH9329_UART_TX_PIN, CH9329_UART_RX_PIN, CH9329_UART_BAUD_RATE);

    // Start from a clean state
    ch9329_release_all_keys();

    return ESP_OK;
}

void ch9329_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    ch9329_release_all_keys();

    s_tasks_running = false;
    int timeout = 20;
    while ((s_tx_task_handle != NULL || s_rx_task_handle != NULL) && timeout-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    uart_wait_tx_done(CH9329_UART_NUM, pdMS_TO_TICKS(100));
    uart_driver_delete(CH9329_UART_NUM);

    vQueueDelete(s_tx_queue);
    s_tx_queue = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "De-initialized");
}

esp_err_t ch9329_send_keyboard_report(const uint8_t data[8])
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tx_msg_t msg = { .type = TX_MSG_KB_REPORT };
    memcpy(msg.data, data, 8);
    return enqueue_msg(&msg);
}

esp_err_t ch9329_release_all_keys(void)
{
    tx_msg_t msg = { .type = TX_MSG_RELEASE_ALL };
    return enqueue_msg(&msg);
}

esp_err_t ch9329_request_led_status(void)
{
    tx_msg_t msg = { .type = TX_MSG_LED_REQUEST };
    return enqueue_msg(&msg);
}

void ch9329_set_led_callback(ch9329_led_cb_t cb)
{
    s_led_callback = cb;
}

bool ch9329_is_ready(void)
{
    return s_initialized;
}
