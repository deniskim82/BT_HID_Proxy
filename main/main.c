/**
 * @file main.c
 * @brief BT HID Proxy - Bluetooth Keyboard to USB HID Bridge
 *
 * Main application entry point. Coordinates:
 * - Bluetooth HID Host (keyboard input)
 * - CH9329 UART communication (USB HID output)
 * - NVS storage (pairing persistence)
 * - Button handling (pairing mode trigger)
 *
 * Hardware: M5Stamp Pico (ESP32-PICO-D4)
 *
 * Operation:
 * 1. On boot, load last paired device from NVS
 * 2. Scan and connect to paired device
 * 3. Relay keyboard input from BT to USB via CH9329
 * 4. Long-press button (5s) to enter pairing mode
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "config.h"
#include "storage.h"
#include "ch9329.h"
#include "bt_hid_host.h"
#include "button.h"
#include "led_status.h"

static const char *TAG = "MAIN";

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static paired_device_t s_paired_device = {0};
static bool s_has_paired = false;

// Button event handling task
static TaskHandle_t s_button_task_handle = NULL;

// Button event queue - used to offload heavy BT operations from timer context
typedef enum {
    BUTTON_TASK_EVENT_SHORT_PRESS = 1,
    BUTTON_TASK_EVENT_LONG_PRESS = 2,
} button_task_event_t;

/* ============================================================================
 * Button Event Handler Task
 * ============================================================================ */

/**
 * @brief Dedicated task for handling button events
 *
 * This task offloads heavy Bluetooth operations from the Timer Service task
 * context to prevent stack overflow. Button callbacks from timer run in
 * Timer Service task with limited stack (2048 bytes default), which is
 * insufficient for BT API calls that internally allocate large structures.
 */
static void button_event_task(void *pvParameters)
{
    uint32_t notification_value;

    ESP_LOGI(TAG, "Button event handler task started (stack: %d bytes)",
             CONFIG_ESP_MAIN_TASK_STACK_SIZE);

    while (1) {
        // Wait for button event notification from timer callback
        if (xTaskNotifyWait(0, ULONG_MAX, &notification_value, portMAX_DELAY) == pdTRUE) {
            button_task_event_t event = (button_task_event_t)notification_value;

            switch (event) {
            case BUTTON_TASK_EVENT_SHORT_PRESS:
                ESP_LOGI(TAG, "Button: Short press");
                // Short press - could be used for future features
                break;

            case BUTTON_TASK_EVENT_LONG_PRESS:
                ESP_LOGI(TAG, "Button: Long press - entering pairing mode");

                // Release all keys before changing mode
                ch9329_release_all_keys();

                // Start pairing (heavy BT operation - safe in dedicated task)
                esp_err_t ret = bt_hid_host_start_pairing();
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to start pairing: %s", esp_err_to_name(ret));
                }
                break;

            default:
                ESP_LOGW(TAG, "Unknown button event: %lu", (unsigned long)notification_value);
                break;
            }
        }
    }
}

/* ============================================================================
 * Callback Functions
 * ============================================================================ */

/**
 * @brief Keyboard input callback from BT HID Host
 *
 * Called when keyboard data is received. Forwards to CH9329.
 * Data format: [modifier, reserved, key1, key2, key3, key4, key5, key6]
 */
static void on_keyboard_input(const uint8_t *data)
{
    // Forward keyboard report to CH9329
    esp_err_t ret = ch9329_send_keyboard_raw(data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send keyboard data: %s", esp_err_to_name(ret));
    }

    // Debug logging (minimal to reduce latency)
    ESP_LOGD(TAG, "KB: %02X %02X %02X %02X %02X %02X %02X %02X",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

/**
 * @brief BT connection state change callback
 */
static void on_bt_state_change(bt_hid_state_t state)
{
    switch (state) {
    case BT_HID_STATE_IDLE:
        ESP_LOGI(TAG, "BT: Idle");
        led_status_set(LED_STATUS_IDLE);
        break;
    case BT_HID_STATE_SCANNING:
        ESP_LOGI(TAG, "BT: Scanning...");
        led_status_set(LED_STATUS_SCANNING);
        break;
    case BT_HID_STATE_CONNECTING:
        ESP_LOGI(TAG, "BT: Connecting...");
        led_status_set(LED_STATUS_CONNECTING);
        break;
    case BT_HID_STATE_CONNECTED:
        ESP_LOGI(TAG, "BT: Connected!");
        led_status_set(LED_STATUS_CONNECTED);
        // Ensure keys are released on new connection
        ch9329_release_all_keys();
        break;
    case BT_HID_STATE_PAIRING:
        ESP_LOGI(TAG, "BT: Pairing mode");
        led_status_set(LED_STATUS_PAIRING);
        break;
    case BT_HID_STATE_ERROR:
        ESP_LOGW(TAG, "BT: Error");
        led_status_set(LED_STATUS_ERROR);
        break;
    }
}

/**
 * @brief Button event callback (runs in Timer Service task context)
 *
 * This callback runs in FreeRTOS Timer Service task, which has limited stack.
 * We MUST NOT call heavy Bluetooth APIs here. Instead, notify the dedicated
 * button_event_task to handle the actual work.
 */
static void on_button_event(button_event_t event)
{
    if (s_button_task_handle == NULL) {
        ESP_LOGW(TAG, "Button task not initialized");
        return;
    }

    // Map button event to task event
    button_task_event_t task_event;
    switch (event) {
    case BUTTON_EVENT_SHORT_PRESS:
        task_event = BUTTON_TASK_EVENT_SHORT_PRESS;
        break;
    case BUTTON_EVENT_LONG_PRESS:
        task_event = BUTTON_TASK_EVENT_LONG_PRESS;
        break;
    default:
        return;
    }

    // Send notification to button task (non-blocking, safe from timer context)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(s_button_task_handle, task_event, eSetValueWithOverwrite,
                       &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

static esp_err_t init_all(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "BT HID Proxy - Initializing");
    ESP_LOGI(TAG, "Target: M5Stamp Pico (ESP32-PICO-D4)");
    ESP_LOGI(TAG, "========================================");

    // 1. Initialize NVS storage
    ESP_LOGI(TAG, "Initializing storage...");
    ret = storage_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Storage init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. Load paired device
    ret = storage_load_paired_device(&s_paired_device);
    if (ret == ESP_OK && s_paired_device.valid) {
        s_has_paired = true;
        char addr_str[18];
        storage_bd_addr_to_str(s_paired_device.bd_addr, addr_str);
        ESP_LOGI(TAG, "Loaded paired device: %s (BLE=%d)", addr_str, s_paired_device.is_ble);
    } else {
        s_has_paired = false;
        ESP_LOGI(TAG, "No paired device stored");
    }

    // 3. Initialize CH9329 UART
    ESP_LOGI(TAG, "Initializing CH9329...");
    ret = ch9329_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CH9329 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 4. Initialize Bluetooth HID Host
    ESP_LOGI(TAG, "Initializing Bluetooth HID Host...");
    ret = bt_hid_host_init(on_keyboard_input, on_bt_state_change);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT HID Host init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 5. Create button event handler task
    ESP_LOGI(TAG, "Creating button event handler task...");
    BaseType_t task_ret = xTaskCreate(
        button_event_task,
        "button_evt",
        4096,  // Stack size (bytes) - needs to be large enough for BT API calls
        NULL,
        5,     // Priority (same as main task)
        &s_button_task_handle
    );
    if (task_ret != pdPASS || s_button_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create button event task");
        return ESP_FAIL;
    }

    // 6. Initialize button handler
    ESP_LOGI(TAG, "Initializing button handler...");
    ret = button_init(on_button_event);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Button init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 7. Initialize LED status indicator
    ESP_LOGI(TAG, "Initializing LED status...");
    ret = led_status_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed (non-critical): %s", esp_err_to_name(ret));
        // Continue without LED - not critical
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Initialization complete");
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

/* ============================================================================
 * Main Entry Point
 * ============================================================================ */

void app_main(void)
{
    // Initialize all modules
    esp_err_t ret = init_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialization failed, restarting in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }

    // Start operation
    if (s_has_paired) {
        // Scan for paired device
        ESP_LOGI(TAG, "Scanning for paired device...");
        bt_hid_host_scan_for_device(s_paired_device.bd_addr, s_paired_device.is_ble);
    } else {
        // No paired device - enter pairing mode
        ESP_LOGI(TAG, "No paired device, entering pairing mode...");
        bt_hid_host_start_pairing();
    }

    // Main loop - monitoring and maintenance
    while (1) {
        // Periodic status check (every 30 seconds)
        vTaskDelay(pdMS_TO_TICKS(30000));

        bt_hid_state_t state = bt_hid_host_get_state();
        if (state == BT_HID_STATE_CONNECTED) {
            ESP_LOGI(TAG, "Status: Connected");
        } else {
            ESP_LOGI(TAG, "Status: %d (not connected)", state);
        }

        // Print free heap for debugging
        ESP_LOGD(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    }
}
