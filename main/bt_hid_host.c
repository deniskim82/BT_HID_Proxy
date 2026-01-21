/**
 * @file bt_hid_host.c
 * @brief Bluetooth HID Host implementation
 *
 * Refactored to use ESP-IDF's esp_hid_gap module for reliable BT initialization
 * and scanning. Based on the official esp_hid_host example.
 *
 * Key changes from custom implementation:
 * - Uses esp_hid_gap_init() for proper BT stack initialization
 * - Uses esp_hid_scan() for synchronous, reliable device discovery
 * - Simplified state management
 * - Dedicated task for scan/connect operations
 */

#include "bt_hid_host.h"
#include "esp_hid_gap.h"
#include "config.h"
#include "storage.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_hidh.h"
#include "esp_hid_common.h"

#if CONFIG_BT_BLE_ENABLED
#include "esp_hidh_gattc.h"
#endif

static const char *TAG = "BT_HID";

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define SCAN_DURATION_SECONDS   5
#define HID_TASK_STACK_SIZE     (6 * 1024)
#define HID_TASK_PRIORITY       2

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static bool s_initialized = false;
static bt_hid_state_t s_state = BT_HID_STATE_IDLE;
static SemaphoreHandle_t s_state_mutex = NULL;

// Callbacks
static bt_hid_keyboard_cb_t s_keyboard_cb = NULL;
static bt_hid_state_cb_t s_state_cb = NULL;

// Current connection
static esp_hidh_dev_t *s_connected_dev = NULL;
static esp_bd_addr_t s_connected_addr = {0};
static bool s_connected_is_ble = false;

// Target device for scanning (from NVS)
static esp_bd_addr_t s_target_addr = {0};
static bool s_target_is_ble = false;
static uint8_t s_target_addr_type = 0;  // BLE address type
static bool s_has_target = false;

// Direct connection mode (skip scan)
static volatile bool s_direct_connect_requested = false;

// Task control
static TaskHandle_t s_hid_task_handle = NULL;

// Task commands
typedef enum {
    HID_CMD_NONE = 0,
    HID_CMD_SCAN_TARGET,
    HID_CMD_PAIRING,
    HID_CMD_STOP,
    HID_CMD_DIRECT_CONNECT,  // Direct connection without scan
} hid_task_cmd_t;

static volatile hid_task_cmd_t s_pending_cmd = HID_CMD_NONE;
static volatile bool s_task_running = false;

/* ============================================================================
 * Private Functions - Forward Declarations
 * ============================================================================ */

static void set_state(bt_hid_state_t new_state);
static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
static void send_key_release(void);
static void hid_task(void *pvParameters);

/* ============================================================================
 * Private Functions - State Management
 * ============================================================================ */

static void set_state(bt_hid_state_t new_state)
{
    if (s_state_mutex) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    if (s_state != new_state) {
        ESP_LOGI(TAG, "State: %d -> %d", s_state, new_state);
        s_state = new_state;

        if (s_state_cb) {
            s_state_cb(new_state);
        }
    }

    if (s_state_mutex) {
        xSemaphoreGive(s_state_mutex);
    }
}

static bt_hid_state_t get_state(void)
{
    bt_hid_state_t state;

    if (s_state_mutex) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    state = s_state;

    if (s_state_mutex) {
        xSemaphoreGive(s_state_mutex);
    }

    return state;
}

/* ============================================================================
 * Private Functions - Key Release
 * ============================================================================ */

static void send_key_release(void)
{
    static const uint8_t empty_report[8] = {0};

    if (s_keyboard_cb) {
        s_keyboard_cb(empty_report);
    }
}

/* ============================================================================
 * Private Functions - HID Host Callback
 * ============================================================================ */

static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK && param->open.dev != NULL) {
            esp_hidh_dev_t *dev = param->open.dev;
            const uint8_t *addr = esp_hidh_dev_bda_get(dev);
            const char *name = esp_hidh_dev_name_get(dev);

            char addr_str[18];
            storage_bd_addr_to_str(addr, addr_str);
            ESP_LOGI(TAG, "OPEN: %s (%s)", addr_str, name ? name : "unknown");

            // Dump device info for debugging
            esp_hidh_dev_dump(dev, stdout);

            s_connected_dev = dev;
            memcpy(s_connected_addr, addr, sizeof(esp_bd_addr_t));
            s_connected_is_ble = (esp_hidh_dev_transport_get(dev) == ESP_HID_TRANSPORT_BLE);

            // Send key release to ensure clean state
            send_key_release();

            set_state(BT_HID_STATE_CONNECTED);
        } else {
            ESP_LOGE(TAG, "OPEN failed: %s", esp_err_to_name(param->open.status));
            set_state(BT_HID_STATE_ERROR);
        }
        break;
    }

    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *addr = esp_hidh_dev_bda_get(param->close.dev);
        const char *name = esp_hidh_dev_name_get(param->close.dev);
        char addr_str[18];
        storage_bd_addr_to_str(addr, addr_str);
        ESP_LOGI(TAG, "CLOSE: %s (%s)", addr_str, name ? name : "unknown");

        // Send key release to prevent stuck keys
        send_key_release();

        s_connected_dev = NULL;
        set_state(BT_HID_STATE_IDLE);
        break;
    }

    case ESP_HIDH_INPUT_EVENT: {
        // Log all input events for debugging
        ESP_LOGI(TAG, "INPUT: usage=%s map=%u id=%u len=%d",
                 esp_hid_usage_str(param->input.usage),
                 param->input.map_index,
                 param->input.report_id,
                 param->input.length);
        ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);

        // Process HID input report
        // Standard keyboard report: 8 bytes (modifier, reserved, 6 keycodes)
        // Accept if usage is KEYBOARD or if it looks like a keyboard report (8 bytes)
        esp_hid_usage_t usage = param->input.usage;
        bool is_keyboard_report = (usage == ESP_HID_USAGE_KEYBOARD) ||
                                   (param->input.length == 8);

        if (is_keyboard_report && s_keyboard_cb) {
            s_keyboard_cb(param->input.data);
        }
        break;
    }

    case ESP_HIDH_BATTERY_EVENT: {
        ESP_LOGI(TAG, "Battery: %d%%", param->battery.level);
        break;
    }

    case ESP_HIDH_FEATURE_EVENT: {
        ESP_LOGD(TAG, "Feature report received");
        break;
    }

    default:
        ESP_LOGD(TAG, "HID event: %d", event);
        break;
    }
}

/* ============================================================================
 * Private Functions - HID Task
 * ============================================================================ */

/**
 * @brief Find best matching device from scan results
 *
 * For pairing mode: Select keyboard with strongest signal
 * For target mode: Find the specific target device
 */
static esp_hid_scan_result_t *find_best_device(esp_hid_scan_result_t *results,
                                                size_t num_results,
                                                bool pairing_mode)
{
    if (results == NULL || num_results == 0) {
        return NULL;
    }

    esp_hid_scan_result_t *best = NULL;
    esp_hid_scan_result_t *r = results;

    while (r) {
        char addr_str[18];
        storage_bd_addr_to_str(r->bda, addr_str);

        ESP_LOGI(TAG, "Found: %s %s RSSI=%d Usage=%s Name=%s",
                 (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT",
                 addr_str, r->rssi,
                 esp_hid_usage_str(r->usage),
                 r->name ? r->name : "unknown");

        if (pairing_mode) {
            // Pairing mode: prefer keyboards with strongest signal
            bool is_keyboard = (r->usage == ESP_HID_USAGE_KEYBOARD);

            // Also check BLE appearance directly (0x03C0-0x03C3 = keyboard)
            // because esp_hid_usage_from_appearance() may not work correctly in v5.1.2
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                uint16_t appearance = r->ble.appearance;
                if (appearance >= 0x03C0 && appearance <= 0x03C3) {
                    is_keyboard = true;
                    ESP_LOGI(TAG, "  -> Keyboard detected by appearance (0x%04x)", appearance);
                }
            }

            if (is_keyboard) {
                if (best == NULL || r->rssi > best->rssi) {
                    best = r;
                }
            }
        } else {
            // Target mode: find specific device
            if (s_has_target && storage_bd_addr_equal(r->bda, s_target_addr)) {
                return r;  // Found target
            }
        }

        r = r->next;
    }

    return best;
}

/**
 * @brief Attempt direct connection to saved device without scanning
 * @return true if connection attempt started, false otherwise
 */
static bool try_direct_connect(void)
{
    if (!s_has_target) {
        return false;
    }

    char addr_str[18];
    storage_bd_addr_to_str(s_target_addr, addr_str);
    ESP_LOGI(TAG, "Direct connect to %s (BLE=%d, addr_type=%d)",
             addr_str, s_target_is_ble, s_target_addr_type);

    set_state(BT_HID_STATE_CONNECTING);

    esp_hid_transport_t transport = s_target_is_ble ? ESP_HID_TRANSPORT_BLE : ESP_HID_TRANSPORT_BT;

    esp_hidh_dev_t *dev = esp_hidh_dev_open(s_target_addr, transport, s_target_addr_type);

    if (dev == NULL) {
        ESP_LOGW(TAG, "esp_hidh_dev_open failed for direct connect");
        return false;
    }

    // Wait for connection result (shorter timeout for direct connect)
    int timeout_count = 0;
    while (get_state() == BT_HID_STATE_CONNECTING && timeout_count < 20) {
        vTaskDelay(pdMS_TO_TICKS(250));
        timeout_count++;
    }

    return (get_state() == BT_HID_STATE_CONNECTED);
}

/**
 * @brief Main HID task - handles scanning and connection
 *
 * This task runs scan and connect operations which can block.
 * It's triggered by notifications from bt_hid_host_start_pairing()
 * or bt_hid_host_scan_for_device().
 *
 * Reconnection strategy:
 * 1. If paired device exists, try direct connection first (faster)
 * 2. If direct connection fails, fall back to scanning
 * 3. No long timeouts - continuously retry until connected
 */
static void hid_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HID task started");
    s_task_running = true;

    int reconnect_attempt = 0;
    bool was_connected = false;

    while (1) {
        // Wait for command or timeout
        uint32_t cmd = 0;
        TickType_t wait_time;

        // Shorter wait when we have a target and are idle (fast reconnect)
        if (s_has_target && get_state() == BT_HID_STATE_IDLE) {
            // Exponential backoff: 500ms, 1s, 2s, 3s (max)
            wait_time = pdMS_TO_TICKS(500 + (reconnect_attempt > 5 ? 2500 : reconnect_attempt * 500));
        } else {
            wait_time = pdMS_TO_TICKS(1000);
        }

        if (xTaskNotifyWait(0, ULONG_MAX, &cmd, wait_time) != pdTRUE) {
            // Timeout - check if we should auto-reconnect
            if (s_has_target && get_state() == BT_HID_STATE_IDLE) {
                // First try direct connection (faster)
                cmd = HID_CMD_DIRECT_CONNECT;
            } else {
                continue;
            }
        }

        if (cmd == HID_CMD_STOP) {
            break;
        }

        // If already connected, skip
        if (get_state() == BT_HID_STATE_CONNECTED) {
            reconnect_attempt = 0;
            was_connected = true;
            continue;
        }

        bool pairing_mode = (cmd == HID_CMD_PAIRING);
        bool direct_mode = (cmd == HID_CMD_DIRECT_CONNECT);

        // Handle direct connection (reconnection without scan)
        if (direct_mode && s_has_target) {
            ESP_LOGI(TAG, "Attempting direct reconnection (attempt %d)...", reconnect_attempt + 1);

            if (try_direct_connect()) {
                ESP_LOGI(TAG, "Direct connection successful!");
                reconnect_attempt = 0;
                was_connected = true;
                continue;
            }

            // Direct connection failed
            reconnect_attempt++;

            // After several failed direct attempts, try scanning
            if (reconnect_attempt >= 3 && !was_connected) {
                // If we've never connected before, fall back to scan
                ESP_LOGI(TAG, "Direct connection failed, falling back to scan");
                cmd = HID_CMD_SCAN_TARGET;
                direct_mode = false;
            } else {
                // For known devices, keep trying direct connection
                set_state(BT_HID_STATE_IDLE);
                continue;
            }
        }

        // Scan mode (pairing or reconnect via scan)
        if (pairing_mode) {
            set_state(BT_HID_STATE_PAIRING);
        } else {
            set_state(BT_HID_STATE_SCANNING);
        }

        // Perform scan
        size_t num_results = 0;
        esp_hid_scan_result_t *results = NULL;

        ESP_LOGI(TAG, "Starting scan (%s mode)...",
                 pairing_mode ? "pairing" : "reconnect");

        esp_err_t ret = esp_hid_scan(SCAN_DURATION_SECONDS, &num_results, &results);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
            set_state(BT_HID_STATE_IDLE);
            // Short delay before retry
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        ESP_LOGI(TAG, "Scan complete: %u devices found", num_results);

        if (num_results == 0) {
            ESP_LOGW(TAG, "No HID devices found");
            set_state(BT_HID_STATE_IDLE);
            // Short delay before retry (will attempt direct connect next)
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Find best device
        esp_hid_scan_result_t *target = find_best_device(results, num_results, pairing_mode);

        if (target == NULL) {
            ESP_LOGW(TAG, "No suitable device found");
            esp_hid_scan_results_free(results);
            set_state(BT_HID_STATE_IDLE);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Connect to device
        char addr_str[18];
        storage_bd_addr_to_str(target->bda, addr_str);
        ESP_LOGI(TAG, "Connecting to %s (%s)...",
                 addr_str, target->name ? target->name : "unknown");

        set_state(BT_HID_STATE_CONNECTING);

        // Open connection
        esp_hidh_dev_t *dev = esp_hidh_dev_open(target->bda,
                                                 target->transport,
                                                 target->ble.addr_type);

        if (dev == NULL) {
            ESP_LOGE(TAG, "esp_hidh_dev_open failed");
            esp_hid_scan_results_free(results);
            set_state(BT_HID_STATE_IDLE);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Save device info for pairing mode
        if (pairing_mode) {
            paired_device_t new_device = {
                .addr_type = target->ble.addr_type,
                .is_ble = (target->transport == ESP_HID_TRANSPORT_BLE),
                .valid = true,
            };
            memcpy(new_device.bd_addr, target->bda, sizeof(esp_bd_addr_t));

            // Update target
            memcpy(s_target_addr, target->bda, sizeof(esp_bd_addr_t));
            s_target_is_ble = new_device.is_ble;
            s_target_addr_type = target->ble.addr_type;
            s_has_target = true;

            // Save to NVS
            storage_save_paired_device(&new_device);
            ESP_LOGI(TAG, "Saved paired device: %s", addr_str);
        }

        esp_hid_scan_results_free(results);

        // Connection is async - wait for OPEN event or timeout
        int timeout_count = 0;
        while (get_state() == BT_HID_STATE_CONNECTING && timeout_count < 20) {
            vTaskDelay(pdMS_TO_TICKS(250));
            timeout_count++;
        }

        if (get_state() == BT_HID_STATE_CONNECTED) {
            ESP_LOGI(TAG, "Connection established!");
            reconnect_attempt = 0;
            was_connected = true;
        } else {
            ESP_LOGW(TAG, "Connection timeout or failed");
            set_state(BT_HID_STATE_IDLE);
            reconnect_attempt++;
        }

        // Very short delay before next operation
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    s_task_running = false;
    s_hid_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t bt_hid_host_init(bt_hid_keyboard_cb_t keyboard_cb, bt_hid_state_cb_t state_cb)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (keyboard_cb == NULL) {
        ESP_LOGE(TAG, "Keyboard callback required");
        return ESP_ERR_INVALID_ARG;
    }

    s_keyboard_cb = keyboard_cb;
    s_state_cb = state_cb;

    // Create mutex
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initializing HID GAP (mode=%d)...", HID_HOST_MODE);

    // Initialize BT stack using esp_hid_gap module
    esp_err_t ret = esp_hid_gap_init(HID_HOST_MODE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hid_gap_init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

#if CONFIG_BT_BLE_ENABLED
    // Register GATTC callback for BLE HID
    ret = esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC register failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
#endif

    // Initialize HID Host
    esp_hidh_config_t hidh_config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };

    ret = esp_hidh_init(&hidh_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidh_init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Create HID task
    BaseType_t task_ret = xTaskCreate(
        hid_task,
        "hid_task",
        HID_TASK_STACK_SIZE,
        NULL,
        HID_TASK_PRIORITY,
        &s_hid_task_handle
    );

    if (task_ret != pdPASS || s_hid_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create HID task");
        ret = ESP_FAIL;
        goto cleanup;
    }

    s_initialized = true;
    set_state(BT_HID_STATE_IDLE);

    ESP_LOGI(TAG, "Bluetooth HID Host initialized");
    return ESP_OK;

cleanup:
    if (s_state_mutex) {
        vSemaphoreDelete(s_state_mutex);
        s_state_mutex = NULL;
    }
    return ret;
}

void bt_hid_host_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    // Stop task
    if (s_hid_task_handle) {
        xTaskNotify(s_hid_task_handle, HID_CMD_STOP, eSetValueWithOverwrite);
        // Wait for task to finish
        int timeout = 10;
        while (s_task_running && timeout > 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            timeout--;
        }
    }

    // Disconnect and send key release
    if (s_connected_dev) {
        send_key_release();
        esp_hidh_dev_close(s_connected_dev);
        s_connected_dev = NULL;
    }

    // Deinit HID Host
    esp_hidh_deinit();

    // Note: esp_hid_gap doesn't have a deinit function
    // BT stack stays initialized

    if (s_state_mutex) {
        vSemaphoreDelete(s_state_mutex);
        s_state_mutex = NULL;
    }

    s_initialized = false;
    s_keyboard_cb = NULL;
    s_state_cb = NULL;

    ESP_LOGI(TAG, "Bluetooth HID Host de-initialized");
}

esp_err_t bt_hid_host_scan_for_device(const esp_bd_addr_t target_addr, bool is_ble)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bt_hid_state_t state = get_state();
    if (state == BT_HID_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Already connected");
        return ESP_OK;
    }

    // Set target
    if (target_addr) {
        memcpy(s_target_addr, target_addr, sizeof(esp_bd_addr_t));
        s_target_is_ble = is_ble;
        s_has_target = true;

        char addr_str[18];
        storage_bd_addr_to_str(s_target_addr, addr_str);
        ESP_LOGI(TAG, "Scanning for device: %s (BLE=%d)", addr_str, is_ble);
    }

    // Signal task to scan for target
    if (s_hid_task_handle) {
        xTaskNotify(s_hid_task_handle, HID_CMD_SCAN_TARGET, eSetValueWithOverwrite);
    }

    return ESP_OK;
}

esp_err_t bt_hid_host_start_pairing(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bt_hid_state_t state = get_state();
    if (state == BT_HID_STATE_CONNECTED) {
        // Disconnect current device first
        ESP_LOGI(TAG, "Disconnecting current device before pairing...");
        bt_hid_host_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "Entering pairing mode");

    // Signal task to start pairing
    if (s_hid_task_handle) {
        xTaskNotify(s_hid_task_handle, HID_CMD_PAIRING, eSetValueWithOverwrite);
    }

    return ESP_OK;
}

void bt_hid_host_stop(void)
{
    if (!s_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Stopping scan/operation");
    set_state(BT_HID_STATE_IDLE);
}

void bt_hid_host_disconnect(void)
{
    if (!s_initialized) {
        return;
    }

    // Send key release before disconnect
    send_key_release();

    if (s_connected_dev) {
        ESP_LOGI(TAG, "Disconnecting device");
        esp_hidh_dev_close(s_connected_dev);
        s_connected_dev = NULL;
    }

    set_state(BT_HID_STATE_IDLE);
}

bt_hid_state_t bt_hid_host_get_state(void)
{
    return get_state();
}

bool bt_hid_host_is_connected(void)
{
    return (get_state() == BT_HID_STATE_CONNECTED && s_connected_dev != NULL);
}

esp_err_t bt_hid_host_get_connected_device(esp_bd_addr_t bd_addr, bool *is_ble)
{
    if (!bt_hid_host_is_connected()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (bd_addr) {
        memcpy(bd_addr, s_connected_addr, sizeof(esp_bd_addr_t));
    }

    if (is_ble) {
        *is_ble = s_connected_is_ble;
    }

    return ESP_OK;
}

esp_err_t bt_hid_host_send_led_status(uint8_t led_status)
{
    if (!s_initialized || s_connected_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Sending LED status: 0x%02X (Num=%d Caps=%d Scroll=%d)",
             led_status,
             (led_status & 0x01) ? 1 : 0,
             (led_status & 0x02) ? 1 : 0,
             (led_status & 0x04) ? 1 : 0);

    // Send output report to keyboard
    // For keyboard LED, report_id is typically 0, map_index is 0
    // LED status is 1 byte
    esp_err_t ret = esp_hidh_dev_output_set(s_connected_dev, 0, 0, &led_status, 1);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send LED status: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t bt_hid_host_connect_direct(const esp_bd_addr_t target_addr, bool is_ble, uint8_t addr_type)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bt_hid_state_t state = get_state();
    if (state == BT_HID_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Already connected");
        return ESP_OK;
    }

    // Set target
    if (target_addr) {
        memcpy(s_target_addr, target_addr, sizeof(esp_bd_addr_t));
        s_target_is_ble = is_ble;
        s_target_addr_type = addr_type;
        s_has_target = true;

        char addr_str[18];
        storage_bd_addr_to_str(s_target_addr, addr_str);
        ESP_LOGI(TAG, "Direct connecting to: %s (BLE=%d)", addr_str, is_ble);
    }

    // Signal task to do direct connect
    if (s_hid_task_handle) {
        xTaskNotify(s_hid_task_handle, HID_CMD_DIRECT_CONNECT, eSetValueWithOverwrite);
    }

    return ESP_OK;
}

void bt_hid_host_set_target(const esp_bd_addr_t target_addr, bool is_ble, uint8_t addr_type)
{
    if (target_addr == NULL) {
        return;
    }

    memcpy(s_target_addr, target_addr, sizeof(esp_bd_addr_t));
    s_target_is_ble = is_ble;
    s_target_addr_type = addr_type;
    s_has_target = true;

    char addr_str[18];
    storage_bd_addr_to_str(s_target_addr, addr_str);
    ESP_LOGI(TAG, "Target device set: %s (BLE=%d, addr_type=%d)", addr_str, is_ble, addr_type);
}

void bt_hid_host_clear_target(void)
{
    s_has_target = false;
    memset(s_target_addr, 0, sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG, "Target device cleared");
}

bool bt_hid_host_has_target(void)
{
    return s_has_target;
}
