/**
 * @file bt_hid_host.c
 * @brief Bluetooth HID Host implementation
 *
 * Based on ESP-IDF esp_hid_host example with modifications for stability.
 *
 * Key improvements over reference:
 * - Explicit key release on disconnect to prevent stuck keys
 * - Debouncing for rapid connect/disconnect cycles
 * - State machine for clean operation transitions
 */

#include "bt_hid_host.h"
#include "config.h"
#include "storage.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_hidh.h"
#include "esp_hid_common.h"

static const char *TAG = "BT_HID";

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

// Target device for scanning
static esp_bd_addr_t s_target_addr = {0};
static bool s_target_is_ble = false;
static bool s_has_target = false;

// Pairing mode flag
static bool s_pairing_mode = false;

// Auto-reconnect timer
static TimerHandle_t s_reconnect_timer = NULL;
static bool s_auto_reconnect = true;

// Key release safety timer
static TimerHandle_t s_key_release_timer = NULL;
static uint8_t s_last_keyboard_report[8] = {0};
static int64_t s_last_report_time = 0;

/* ============================================================================
 * Private Functions - Forward Declarations
 * ============================================================================ */

static void set_state(bt_hid_state_t new_state);
static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void reconnect_timer_callback(TimerHandle_t timer);
static void key_release_timer_callback(TimerHandle_t timer);
static void send_key_release(void);

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
 * Private Functions - Key Release Safety
 * ============================================================================ */

static void send_key_release(void)
{
    static const uint8_t empty_report[8] = {0};

    // Always send release to ensure no stuck keys
    if (s_keyboard_cb) {
        s_keyboard_cb(empty_report);
    }

    memset(s_last_keyboard_report, 0, sizeof(s_last_keyboard_report));
}

static void key_release_timer_callback(TimerHandle_t timer)
{
    // If no key activity for KEY_RELEASE_TIMEOUT_MS and keys were pressed,
    // send a release to prevent stuck keys
    int64_t now = esp_timer_get_time();
    int64_t elapsed_ms = (now - s_last_report_time) / 1000;

    if (elapsed_ms >= KEY_RELEASE_TIMEOUT_MS) {
        bool has_keys = false;
        for (int i = 0; i < 8; i++) {
            if (s_last_keyboard_report[i] != 0) {
                has_keys = true;
                break;
            }
        }

        if (has_keys) {
            ESP_LOGW(TAG, "Key release timeout - forcing release");
            send_key_release();
        }
    }
}

/* ============================================================================
 * Private Functions - Reconnection
 * ============================================================================ */

static void reconnect_timer_callback(TimerHandle_t timer)
{
    if (!s_auto_reconnect || !s_has_target) {
        return;
    }

    bt_hid_state_t state = get_state();
    if (state == BT_HID_STATE_IDLE || state == BT_HID_STATE_ERROR) {
        ESP_LOGI(TAG, "Auto-reconnect: starting scan");
        bt_hid_host_scan_for_device(s_target_addr, s_target_is_ble);
    }
}

static void schedule_reconnect(void)
{
    if (s_reconnect_timer && s_auto_reconnect) {
        xTimerStart(s_reconnect_timer, 0);
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
            const esp_hidh_dev_t *dev = param->open.dev;
            const uint8_t *addr = esp_hidh_dev_bda_get(dev);

            char addr_str[18];
            storage_bd_addr_to_str(addr, addr_str);
            ESP_LOGI(TAG, "Connected to %s", addr_str);

            s_connected_dev = param->open.dev;
            memcpy(s_connected_addr, addr, sizeof(esp_bd_addr_t));
            s_connected_is_ble = (esp_hidh_dev_transport_get(dev) == ESP_HID_TRANSPORT_BLE);

            // Cancel reconnect timer
            if (s_reconnect_timer) {
                xTimerStop(s_reconnect_timer, 0);
            }

            // Send key release to ensure clean state
            send_key_release();

            set_state(BT_HID_STATE_CONNECTED);

            // Save device if in pairing mode
            if (s_pairing_mode) {
                paired_device_t new_device = {
                    .addr_type = 0,
                    .is_ble = s_connected_is_ble,
                    .valid = true,
                };
                memcpy(new_device.bd_addr, s_connected_addr, sizeof(esp_bd_addr_t));
                storage_save_paired_device(&new_device);

                // Update target
                memcpy(s_target_addr, s_connected_addr, sizeof(esp_bd_addr_t));
                s_target_is_ble = s_connected_is_ble;
                s_has_target = true;

                s_pairing_mode = false;
            }
        } else {
            ESP_LOGW(TAG, "Open failed: %s", esp_err_to_name(param->open.status));
            set_state(BT_HID_STATE_ERROR);
            schedule_reconnect();
        }
        break;
    }

    case ESP_HIDH_CLOSE_EVENT: {
        ESP_LOGI(TAG, "Connection closed (reason: %d)", param->close.reason);

        // CRITICAL: Send key release immediately on disconnect
        // This prevents stuck keys when switching between multi-paired devices
        send_key_release();

        s_connected_dev = NULL;
        set_state(BT_HID_STATE_IDLE);

        // Schedule reconnect if we have a target
        if (s_has_target && s_auto_reconnect) {
            schedule_reconnect();
        }
        break;
    }

    case ESP_HIDH_INPUT_EVENT: {
        if (param->input.dev == s_connected_dev) {
            // Process HID input report
            esp_hid_usage_t usage = param->input.usage;

            if (usage == ESP_HID_USAGE_KEYBOARD) {
                // Keyboard report - 8 bytes
                if (param->input.length >= 8) {
                    const uint8_t *data = param->input.data;

                    // Update last report and time
                    memcpy(s_last_keyboard_report, data, 8);
                    s_last_report_time = esp_timer_get_time();

                    // Forward to callback
                    if (s_keyboard_cb) {
                        s_keyboard_cb(data);
                    }
                }
            }
            // Note: Consumer/media keys could be handled here in the future
        }
        break;
    }

    case ESP_HIDH_BATTERY_EVENT:
        ESP_LOGD(TAG, "Battery: %d%%", param->battery.level);
        break;

    default:
        ESP_LOGD(TAG, "HID event: %d", event);
        break;
    }
}

/* ============================================================================
 * Private Functions - GAP Callbacks
 * ============================================================================ */

static void check_and_connect_device(const esp_bd_addr_t bd_addr, const char *name,
                                      esp_bt_cod_t *cod, int rssi, bool is_ble)
{
    char addr_str[18];
    storage_bd_addr_to_str(bd_addr, addr_str);

    // In pairing mode, connect to any HID device
    if (s_pairing_mode) {
        // Check if it's a keyboard (COD for Classic BT)
        if (!is_ble && cod) {
            // COD Major Device Class: bits 12-8 of full COD
            // 0x05 = Peripheral (keyboard, mouse, etc.)
            uint32_t cod_full = (cod->major << 8) | cod->minor;
            uint8_t major_class = (cod_full >> 8) & 0x1F;
            uint8_t minor_class = cod->minor;

            // Major class 0x05 = Peripheral, Minor 0x40 = Keyboard
            if (major_class != 0x05) {  // Not a peripheral device
                return;
            }
            if ((minor_class & 0xC0) != 0x40) {  // Not a keyboard
                return;
            }
        }

        ESP_LOGI(TAG, "Pairing: Found HID device %s (%s)", addr_str, name ? name : "unknown");

        // Stop scan and connect
        if (is_ble) {
            esp_ble_gap_stop_scanning();
        } else {
            esp_bt_gap_cancel_discovery();
        }

        set_state(BT_HID_STATE_CONNECTING);
        esp_hidh_dev_open(bd_addr, is_ble ? ESP_HID_TRANSPORT_BLE : ESP_HID_TRANSPORT_BT, 0);
        return;
    }

    // Not pairing - check if this is our target device
    if (s_has_target && storage_bd_addr_equal(bd_addr, s_target_addr)) {
        ESP_LOGI(TAG, "Found target device %s", addr_str);

        // Stop scan and connect
        if (is_ble) {
            esp_ble_gap_stop_scanning();
        } else {
            esp_bt_gap_cancel_discovery();
        }

        set_state(BT_HID_STATE_CONNECTING);
        esp_hidh_dev_open(bd_addr, s_target_is_ble ? ESP_HID_TRANSPORT_BLE : ESP_HID_TRANSPORT_BT, 0);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // Found a BLE device
            check_and_connect_device(
                param->scan_rst.bda,
                NULL,  // Name not available in scan result
                NULL,
                param->scan_rst.rssi,
                true
            );
        } else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            ESP_LOGI(TAG, "BLE scan complete");
            if (get_state() == BT_HID_STATE_SCANNING) {
                set_state(BT_HID_STATE_IDLE);
                schedule_reconnect();
            }
        }
        break;

    default:
        break;
    }
}

static void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        // Found a Classic BT device
        char *name = NULL;

        // Extract device name from EIR
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR) {
                uint8_t *eir = param->disc_res.prop[i].val;
                uint8_t len;
                name = (char *)esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
                if (!name) {
                    name = (char *)esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
                }
            }
        }

        check_and_connect_device(
            param->disc_res.bda,
            name,
            &param->disc_res.cod,
            param->disc_res.rssi,
            false
        );
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "BT scan stopped");
            if (get_state() == BT_HID_STATE_SCANNING) {
                set_state(BT_HID_STATE_IDLE);
                schedule_reconnect();
            }
        }
        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Authentication success");
        } else {
            ESP_LOGW(TAG, "Authentication failed: %d", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
        // Auto-respond with default PIN "0000"
        ESP_LOGI(TAG, "PIN requested, sending default");
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;

    default:
        break;
    }
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

    // Release BLE memory not needed (we need both Classic and BLE)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_IDLE));

    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Register GAP callbacks
    esp_bt_gap_register_callback(bt_gap_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);

    // Configure Classic BT
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;  // No input/output for pairing
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    // Set scan mode (discoverable for pairing)
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Configure BLE security
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t ble_iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &ble_iocap, sizeof(ble_iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));

    // Initialize HID Host
    esp_hidh_config_t hidh_config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };

    ret = esp_hidh_init(&hidh_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID Host init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Create reconnect timer
    s_reconnect_timer = xTimerCreate(
        "bt_reconn",
        pdMS_TO_TICKS(BT_RETRY_DELAY_MS),
        pdFALSE,  // One-shot
        NULL,
        reconnect_timer_callback
    );

    // Create key release safety timer
    s_key_release_timer = xTimerCreate(
        "key_rel",
        pdMS_TO_TICKS(KEY_RELEASE_TIMEOUT_MS / 2),
        pdTRUE,  // Auto-reload
        NULL,
        key_release_timer_callback
    );

    if (s_key_release_timer) {
        xTimerStart(s_key_release_timer, 0);
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

    // Stop timers
    if (s_reconnect_timer) {
        xTimerStop(s_reconnect_timer, portMAX_DELAY);
        xTimerDelete(s_reconnect_timer, portMAX_DELAY);
        s_reconnect_timer = NULL;
    }

    if (s_key_release_timer) {
        xTimerStop(s_key_release_timer, portMAX_DELAY);
        xTimerDelete(s_key_release_timer, portMAX_DELAY);
        s_key_release_timer = NULL;
    }

    // Disconnect and send key release
    if (s_connected_dev) {
        send_key_release();
        esp_hidh_dev_close(s_connected_dev);
        s_connected_dev = NULL;
    }

    // Deinit HID Host
    esp_hidh_deinit();

    // Deinit Bluetooth
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

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
    if (state == BT_HID_STATE_CONNECTED || state == BT_HID_STATE_CONNECTING) {
        ESP_LOGW(TAG, "Already connected or connecting");
        return ESP_ERR_INVALID_STATE;
    }

    // Set target
    if (target_addr) {
        memcpy(s_target_addr, target_addr, sizeof(esp_bd_addr_t));
        s_target_is_ble = is_ble;
        s_has_target = true;

        char addr_str[18];
        storage_bd_addr_to_str(s_target_addr, addr_str);
        ESP_LOGI(TAG, "Scanning for device: %s (BLE=%d)", addr_str, is_ble);
    } else {
        s_has_target = false;
        ESP_LOGI(TAG, "Scanning for any HID device");
    }

    s_pairing_mode = false;
    s_auto_reconnect = true;
    set_state(BT_HID_STATE_SCANNING);

    // Start scan based on target type
    if (is_ble || !s_has_target) {
        // BLE scan
        esp_ble_scan_params_t scan_params = {
            .scan_type = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval = 0x50,
            .scan_window = 0x30,
            .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
        };
        esp_ble_gap_set_scan_params(&scan_params);
        esp_ble_gap_start_scanning(BT_SCAN_TIMEOUT_SEC);
    }

    if (!is_ble || !s_has_target) {
        // Classic BT discovery
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                                   BT_SCAN_TIMEOUT_SEC, 0);
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
        bt_hid_host_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "Entering pairing mode");

    s_pairing_mode = true;
    s_auto_reconnect = false;
    set_state(BT_HID_STATE_PAIRING);

    // Start both BLE and Classic BT scan
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
    };
    esp_ble_gap_set_scan_params(&scan_params);
    esp_ble_gap_start_scanning(BT_SCAN_TIMEOUT_SEC * 3);  // Longer timeout for pairing

    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY,
                               BT_SCAN_TIMEOUT_SEC * 3, 0);

    return ESP_OK;
}

void bt_hid_host_stop(void)
{
    if (!s_initialized) {
        return;
    }

    ESP_LOGI(TAG, "Stopping scan/operation");

    // Stop reconnect timer
    if (s_reconnect_timer) {
        xTimerStop(s_reconnect_timer, 0);
    }

    s_auto_reconnect = false;
    s_pairing_mode = false;

    // Stop scans
    esp_ble_gap_stop_scanning();
    esp_bt_gap_cancel_discovery();

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
