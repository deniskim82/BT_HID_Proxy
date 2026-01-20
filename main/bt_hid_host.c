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
static bool s_has_target = false;

// Task control
static TaskHandle_t s_hid_task_handle = NULL;

// Task commands
typedef enum {
    HID_CMD_NONE = 0,
    HID_CMD_SCAN_TARGET,
    HID_CMD_PAIRING,
    HID_CMD_STOP,
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
        // Process HID input report
        esp_hid_usage_t usage = param->input.usage;

        if (usage == ESP_HID_USAGE_KEYBOARD && param->input.length >= 8) {
            // Keyboard report - forward to callback
            if (s_keyboard_cb) {
                s_keyboard_cb(param->input.data);
            }
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
 * @brief Main HID task - handles scanning and connection
 *
 * This task runs scan and connect operations which can block.
 * It's triggered by notifications from bt_hid_host_start_pairing()
 * or bt_hid_host_scan_for_device().
 */
static void hid_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HID task started");
    s_task_running = true;

    while (1) {
        // Wait for command
        uint32_t cmd = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &cmd, pdMS_TO_TICKS(1000)) != pdTRUE) {
            // Timeout - check if we should auto-reconnect
            if (s_has_target && get_state() == BT_HID_STATE_IDLE) {
                cmd = HID_CMD_SCAN_TARGET;
            } else {
                continue;
            }
        }

        if (cmd == HID_CMD_STOP) {
            break;
        }

        bool pairing_mode = (cmd == HID_CMD_PAIRING);

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
            set_state(BT_HID_STATE_ERROR);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        ESP_LOGI(TAG, "Scan complete: %u devices found", num_results);

        if (num_results == 0) {
            ESP_LOGW(TAG, "No HID devices found");
            set_state(BT_HID_STATE_IDLE);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Find best device
        esp_hid_scan_result_t *target = find_best_device(results, num_results, pairing_mode);

        if (target == NULL) {
            ESP_LOGW(TAG, "No suitable device found");
            esp_hid_scan_results_free(results);
            set_state(BT_HID_STATE_IDLE);
            vTaskDelay(pdMS_TO_TICKS(2000));
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
            set_state(BT_HID_STATE_ERROR);
            vTaskDelay(pdMS_TO_TICKS(2000));
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
            s_has_target = true;

            // Save to NVS
            storage_save_paired_device(&new_device);
            ESP_LOGI(TAG, "Saved paired device: %s", addr_str);
        }

        esp_hid_scan_results_free(results);

        // Connection is async - wait for OPEN event or timeout
        // The hidh_callback will update state to CONNECTED on success
        int timeout_count = 0;
        while (get_state() == BT_HID_STATE_CONNECTING && timeout_count < 30) {
            vTaskDelay(pdMS_TO_TICKS(500));
            timeout_count++;
        }

        if (get_state() != BT_HID_STATE_CONNECTED) {
            ESP_LOGW(TAG, "Connection timeout or failed");
            set_state(BT_HID_STATE_IDLE);
        }

        // Small delay before next operation
        vTaskDelay(pdMS_TO_TICKS(1000));
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
