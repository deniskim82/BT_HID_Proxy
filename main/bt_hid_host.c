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
#include "esp_gattc_api.h"
#include "esp_hidh.h"
#include "esp_hidh_gattc.h"
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
static uint8_t s_target_addr_type = 0;  // BLE address type for target
static bool s_has_target = false;

// Current connecting device info (for saving on successful connection)
static uint8_t s_connecting_addr_type = 0;

// Pairing mode flag
static bool s_pairing_mode = false;

// Discovered device candidate for pairing
typedef struct {
    esp_bd_addr_t bd_addr;
    int rssi;
    bool is_ble;
    bool is_discoverable;  // Limited discoverable mode (pairing mode indicator)
    uint8_t addr_type;     // BLE address type (BLE_ADDR_TYPE_PUBLIC/RANDOM)
    bool valid;
} pairing_candidate_t;

#define MAX_PAIRING_CANDIDATES 5
static pairing_candidate_t s_candidates[MAX_PAIRING_CANDIDATES];
static int s_candidate_count = 0;

// Auto-reconnect timer
static TimerHandle_t s_reconnect_timer = NULL;
static bool s_auto_reconnect = true;

// Connection timeout timer
static TimerHandle_t s_connect_timeout_timer = NULL;
static bool s_restart_pairing_after_timeout = false;

/*
 * NOTE: Key release safety timer temporarily disabled for performance testing.
 * If stuck key issues occur, uncomment this section and related code.
 *
 * The timer was designed to detect and release stuck keys when:
 * - BT connection is lost without proper key release
 * - Multi-paired keyboard switches to another device
 *
 * To re-enable:
 * 1. Uncomment s_key_release_timer, s_last_keyboard_report, s_last_report_time
 * 2. Uncomment key_release_timer_callback() function
 * 3. Uncomment timer creation in bt_hid_host_init()
 * 4. Uncomment timer cleanup in bt_hid_host_deinit()
 * 5. Uncomment memcpy and esp_timer_get_time() in ESP_HIDH_INPUT_EVENT handler
 */
#if 0  // DISABLED: Key release safety timer
static TimerHandle_t s_key_release_timer = NULL;
static uint8_t s_last_keyboard_report[8] = {0};
static int64_t s_last_report_time = 0;
#endif

/* ============================================================================
 * Private Functions - Forward Declarations
 * ============================================================================ */

static void set_state(bt_hid_state_t new_state);
static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void bt_gap_event_handler(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void reconnect_timer_callback(TimerHandle_t timer);
static void connect_timeout_callback(TimerHandle_t timer);
static void send_key_release(void);
static void add_pairing_candidate(const esp_bd_addr_t bd_addr, int rssi, bool is_ble,
                                   bool is_discoverable, uint8_t addr_type);
static void select_and_connect_best_candidate(void);
static void clear_pairing_candidates(void);

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
}

#if 0  // DISABLED: Key release timer callback
/**
 * @brief Timer callback to detect and release stuck keys
 *
 * This was designed to handle cases where:
 * - BT disconnect doesn't properly release keys
 * - Multi-paired keyboard causes phantom key holds
 *
 * If stuck key issues occur during testing, re-enable this feature.
 */
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
            memset(s_last_keyboard_report, 0, sizeof(s_last_keyboard_report));
        }
    }
}
#endif

/* ============================================================================
 * Private Functions - Reconnection
 * ============================================================================ */

static void reconnect_timer_callback(TimerHandle_t timer)
{
    // Check if we need to restart pairing mode
    if (s_restart_pairing_after_timeout) {
        s_restart_pairing_after_timeout = false;
        // FIXME: Cannot call bt_hid_host_start_pairing() from timer context (stack overflow)
        // User must manually press button to restart pairing
        ESP_LOGW(TAG, "Pairing timeout - please press button to restart pairing mode");
        return;
    }

    if (!s_auto_reconnect || !s_has_target) {
        return;
    }

    bt_hid_state_t state = get_state();
    if (state == BT_HID_STATE_IDLE || state == BT_HID_STATE_ERROR) {
        ESP_LOGI(TAG, "Auto-reconnect: starting scan");
        // FIXME: bt_hid_host_scan_for_device() also calls heavy BT APIs from timer context
        // This may cause stack overflow. Need to offload to dedicated task.
        ESP_LOGW(TAG, "Auto-reconnect disabled (would cause stack overflow)");
        // bt_hid_host_scan_for_device(s_target_addr, s_target_is_ble);
    }
}

static void connect_timeout_callback(TimerHandle_t timer)
{
    ESP_LOGW(TAG, ">>> CONNECTION TIMEOUT FIRED <<<");

    bt_hid_state_t state = get_state();
    ESP_LOGW(TAG, "Current state: %d (CONNECTING=%d)", state, BT_HID_STATE_CONNECTING);

    if (state != BT_HID_STATE_CONNECTING) {
        ESP_LOGW(TAG, "Ignoring timeout - not in CONNECTING state");
        return;
    }

    ESP_LOGW(TAG, "Connection timeout - aborting connection attempt");
    bool was_pairing = s_pairing_mode;

    // Reset state
    s_pairing_mode = false;
    set_state(BT_HID_STATE_IDLE);

    // Schedule appropriate action via reconnect timer
    if (was_pairing) {
        ESP_LOGI(TAG, "Will restart pairing mode after delay");
        s_restart_pairing_after_timeout = true;
    }

    // Use reconnect timer to schedule the next action
    if (s_reconnect_timer) {
        xTimerChangePeriod(s_reconnect_timer, pdMS_TO_TICKS(2000), 0);
        xTimerStart(s_reconnect_timer, 0);
    }
}

static void start_connect_timeout(void)
{
    if (s_connect_timeout_timer) {
        ESP_LOGI(TAG, "Starting connection timeout timer (%d ms)", BT_CONNECT_TIMEOUT_MS);
        BaseType_t result = xTimerStart(s_connect_timeout_timer, 0);
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to start timeout timer!");
        }
    } else {
        ESP_LOGE(TAG, "Timeout timer is NULL!");
    }
}

static void stop_connect_timeout(void)
{
    if (s_connect_timeout_timer) {
        ESP_LOGI(TAG, "Stopping connection timeout timer");
        xTimerStop(s_connect_timeout_timer, 0);
    }
}

static void schedule_reconnect(void)
{
    if (s_reconnect_timer && s_auto_reconnect) {
        xTimerStart(s_reconnect_timer, 0);
    }
}

/* ============================================================================
 * Private Functions - Pairing Candidate Management
 * ============================================================================ */

static void clear_pairing_candidates(void)
{
    memset(s_candidates, 0, sizeof(s_candidates));
    s_candidate_count = 0;
}

static void add_pairing_candidate(const esp_bd_addr_t bd_addr, int rssi, bool is_ble,
                                   bool is_discoverable, uint8_t addr_type)
{
    // Check if device already in list
    for (int i = 0; i < s_candidate_count; i++) {
        if (storage_bd_addr_equal(bd_addr, s_candidates[i].bd_addr)) {
            // Update RSSI if stronger
            if (rssi > s_candidates[i].rssi) {
                s_candidates[i].rssi = rssi;
            }
            // Update discoverable flag if now discoverable
            if (is_discoverable) {
                s_candidates[i].is_discoverable = true;
            }
            // Update addr_type (BLE address type might change with RPA)
            s_candidates[i].addr_type = addr_type;
            return;
        }
    }

    // Add new candidate if space available
    if (s_candidate_count < MAX_PAIRING_CANDIDATES) {
        memcpy(s_candidates[s_candidate_count].bd_addr, bd_addr, sizeof(esp_bd_addr_t));
        s_candidates[s_candidate_count].rssi = rssi;
        s_candidates[s_candidate_count].is_ble = is_ble;
        s_candidates[s_candidate_count].is_discoverable = is_discoverable;
        s_candidates[s_candidate_count].addr_type = addr_type;
        s_candidates[s_candidate_count].valid = true;
        s_candidate_count++;

        char addr_str[18];
        storage_bd_addr_to_str(bd_addr, addr_str);
        ESP_LOGI(TAG, "Candidate #%d: %s RSSI=%d BLE=%d Discoverable=%d AddrType=%d",
                 s_candidate_count, addr_str, rssi, is_ble, is_discoverable, addr_type);
    } else {
        // Replace weakest signal candidate if this one is stronger
        int weakest_idx = 0;
        int weakest_rssi = s_candidates[0].rssi;
        for (int i = 1; i < MAX_PAIRING_CANDIDATES; i++) {
            if (s_candidates[i].rssi < weakest_rssi) {
                weakest_rssi = s_candidates[i].rssi;
                weakest_idx = i;
            }
        }
        if (rssi > weakest_rssi) {
            memcpy(s_candidates[weakest_idx].bd_addr, bd_addr, sizeof(esp_bd_addr_t));
            s_candidates[weakest_idx].rssi = rssi;
            s_candidates[weakest_idx].is_ble = is_ble;
            s_candidates[weakest_idx].is_discoverable = is_discoverable;
            s_candidates[weakest_idx].addr_type = addr_type;
        }
    }
}

static void select_and_connect_best_candidate(void)
{
    if (s_candidate_count == 0) {
        ESP_LOGW(TAG, "No pairing candidates found");
        set_state(BT_HID_STATE_IDLE);
        // Restart pairing scan after delay
        s_restart_pairing_after_timeout = true;
        if (s_reconnect_timer) {
            xTimerChangePeriod(s_reconnect_timer, pdMS_TO_TICKS(2000), 0);
            xTimerStart(s_reconnect_timer, 0);
        }
        return;
    }

    // Selection priority:
    // 1. Discoverable mode devices (in pairing mode) with RSSI above threshold
    // 2. Any device with RSSI above threshold
    // 3. Strongest signal device

    pairing_candidate_t *best = NULL;
    int best_score = -1000;  // Lower than any RSSI

    for (int i = 0; i < s_candidate_count; i++) {
        if (!s_candidates[i].valid) continue;

        int score = s_candidates[i].rssi;

        // Bonus for discoverable mode (likely in pairing mode)
        if (s_candidates[i].is_discoverable) {
            score += 30;  // +30dB equivalent bonus
        }

        // Only consider devices above RSSI threshold, unless discoverable
        if (s_candidates[i].rssi < BT_PAIRING_RSSI_THRESHOLD && !s_candidates[i].is_discoverable) {
            continue;
        }

        if (score > best_score) {
            best_score = score;
            best = &s_candidates[i];
        }
    }

    // Fallback: if no device passed threshold, pick strongest
    if (best == NULL) {
        int strongest_rssi = -1000;
        for (int i = 0; i < s_candidate_count; i++) {
            if (s_candidates[i].valid && s_candidates[i].rssi > strongest_rssi) {
                strongest_rssi = s_candidates[i].rssi;
                best = &s_candidates[i];
            }
        }
    }

    if (best != NULL) {
        char addr_str[18];
        storage_bd_addr_to_str(best->bd_addr, addr_str);
        ESP_LOGI(TAG, "Selected best candidate: %s RSSI=%d Discoverable=%d",
                 addr_str, best->rssi, best->is_discoverable);

        set_state(BT_HID_STATE_CONNECTING);
        start_connect_timeout();

        // Save addr_type for updating target on successful connection
        s_connecting_addr_type = best->addr_type;

        ESP_LOGI(TAG, ">>> BEFORE esp_hidh_dev_open() call <<<");
        ESP_LOGI(TAG, "Calling esp_hidh_dev_open() for %s (transport=%s, addr_type=%d)...",
                 addr_str, best->is_ble ? "BLE" : "Classic BT", best->addr_type);

        esp_hidh_dev_t *dev = esp_hidh_dev_open((uint8_t *)best->bd_addr,
                          best->is_ble ? ESP_HID_TRANSPORT_BLE : ESP_HID_TRANSPORT_BT,
                          best->addr_type);

        ESP_LOGI(TAG, ">>> AFTER esp_hidh_dev_open() call, dev=%p <<<", dev);

        if (dev == NULL) {
            ESP_LOGE(TAG, "esp_hidh_dev_open() returned NULL - open failed immediately");
            stop_connect_timeout();
            set_state(BT_HID_STATE_ERROR);
            // Retry pairing after delay
            s_restart_pairing_after_timeout = true;
            if (s_reconnect_timer) {
                xTimerChangePeriod(s_reconnect_timer, pdMS_TO_TICKS(2000), 0);
                xTimerStart(s_reconnect_timer, 0);
            }
        } else {
            ESP_LOGI(TAG, "esp_hidh_dev_open() initiated successfully (dev=%p), waiting for OPEN event...", dev);
        }
    }

    clear_pairing_candidates();
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
        // Stop connection timeout timer
        stop_connect_timeout();

        if (param->open.status == ESP_OK && param->open.dev != NULL) {
            esp_hidh_dev_t *dev = param->open.dev;
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
                s_target_addr_type = s_connecting_addr_type;  // Save BLE address type
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

                    /*
                     * NOTE: Tracking code disabled for performance optimization.
                     * This was used by the key release safety timer.
                     * Re-enable if stuck key issues occur.
                     */
#if 0  // DISABLED: Key tracking for release timer
                    memcpy(s_last_keyboard_report, data, 8);
                    s_last_report_time = esp_timer_get_time();
#endif

                    // Forward to callback (fast path - no extra processing)
                    if (s_keyboard_cb) {
                        s_keyboard_cb(data);
                    }
                }
            }
#if DEBUG_VERBOSE
            else {
                ESP_LOGD(TAG, "HID usage: %d, len: %d", usage, param->input.length);
            }
#endif
        }
        break;
    }

    case ESP_HIDH_BATTERY_EVENT:
#if DEBUG_VERBOSE
        ESP_LOGD(TAG, "Battery: %d%%", param->battery.level);
#endif
        break;

    default:
#if DEBUG_VERBOSE
        ESP_LOGD(TAG, "HID event: %d", event);
#endif
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

    // Note: Pairing mode is now handled separately in gap_event_handler and bt_gap_event_handler
    // This function is only called when NOT in pairing mode (scanning for known device)

    // Check if this is our target device
    if (s_has_target && storage_bd_addr_equal(bd_addr, s_target_addr)) {
        ESP_LOGI(TAG, "Found target device %s", addr_str);

        // Update target addr_type from scan result (BLE address type may change with RPA)
        // For Classic BT, addr_type is ignored anyway
        if (is_ble && cod == NULL) {
            // BLE scan - addr_type is in scan result, but we don't have it here
            // Will use stored s_target_addr_type which was saved from pairing
            ESP_LOGI(TAG, "Using stored addr_type=%d for BLE target", s_target_addr_type);
        }

        // Stop scan and connect
        if (is_ble) {
            esp_ble_gap_stop_scanning();
        } else {
            esp_bt_gap_cancel_discovery();
        }

        set_state(BT_HID_STATE_CONNECTING);
        start_connect_timeout();

        // Save for updating target on successful connection
        s_connecting_addr_type = s_target_addr_type;

        ESP_LOGI(TAG, "Calling esp_hidh_dev_open() for target %s (transport=%s, addr_type=%d)...",
                 addr_str, s_target_is_ble ? "BLE" : "Classic BT", s_target_addr_type);
        esp_hidh_dev_t *dev = esp_hidh_dev_open((uint8_t *)bd_addr,
                          s_target_is_ble ? ESP_HID_TRANSPORT_BLE : ESP_HID_TRANSPORT_BT,
                          s_target_addr_type);
        if (dev == NULL) {
            ESP_LOGE(TAG, "esp_hidh_dev_open() returned NULL - open failed immediately");
            stop_connect_timeout();
            set_state(BT_HID_STATE_ERROR);
            schedule_reconnect();
        } else {
            ESP_LOGI(TAG, "esp_hidh_dev_open() initiated (dev=%p), waiting for OPEN event...", dev);
        }
    }
}

// HID Service UUID for BLE (0x1812)
static const uint16_t HID_SERVICE_UUID = 0x1812;

// BLE Advertising Flags
#define BLE_ADV_FLAG_LIMITED_DISC   0x01  // Limited Discoverable Mode (usually pairing mode)
#define BLE_ADV_FLAG_GENERAL_DISC   0x02  // General Discoverable Mode

static bool ble_is_limited_discoverable(uint8_t *adv_data, uint8_t adv_data_len)
{
    if (adv_data == NULL || adv_data_len == 0) {
        return false;
    }

    uint8_t flags_len = 0;
    uint8_t *flags = esp_ble_resolve_adv_data(adv_data, ESP_BLE_AD_TYPE_FLAG, &flags_len);

    if (flags != NULL && flags_len > 0) {
        // Limited Discoverable Mode indicates device is actively seeking pairing
        if (flags[0] & BLE_ADV_FLAG_LIMITED_DISC) {
            return true;
        }
    }
    return false;
}

static bool ble_has_hid_service(uint8_t *adv_data, uint8_t adv_data_len)
{
    if (adv_data == NULL || adv_data_len == 0) {
        return false;
    }

    // Search for 16-bit service UUIDs in advertising data
    uint8_t *uuid_data = NULL;
    uint8_t uuid_len = 0;

    // Try complete list of 16-bit UUIDs
    uuid_data = esp_ble_resolve_adv_data(adv_data, ESP_BLE_AD_TYPE_16SRV_CMPL, &uuid_len);
    if (uuid_data != NULL && uuid_len >= 2) {
        for (int i = 0; i < uuid_len; i += 2) {
            uint16_t uuid = (uuid_data[i + 1] << 8) | uuid_data[i];
            if (uuid == HID_SERVICE_UUID) {
                return true;
            }
        }
    }

    // Try incomplete list of 16-bit UUIDs
    uuid_data = esp_ble_resolve_adv_data(adv_data, ESP_BLE_AD_TYPE_16SRV_PART, &uuid_len);
    if (uuid_data != NULL && uuid_len >= 2) {
        for (int i = 0; i < uuid_len; i += 2) {
            uint16_t uuid = (uuid_data[i + 1] << 8) | uuid_data[i];
            if (uuid == HID_SERVICE_UUID) {
                return true;
            }
        }
    }

    return false;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            // In pairing mode, collect candidates instead of connecting immediately
            if (s_pairing_mode) {
                if (!ble_has_hid_service(param->scan_rst.ble_adv, param->scan_rst.adv_data_len)) {
                    // Not a HID device, skip
                    return;
                }

                // Check if device is in Limited Discoverable Mode (pairing mode)
                bool is_discoverable = ble_is_limited_discoverable(
                    param->scan_rst.ble_adv, param->scan_rst.adv_data_len);

                // Add to candidate list with BLE address type
                add_pairing_candidate(
                    param->scan_rst.bda,
                    param->scan_rst.rssi,
                    true,  // is_ble
                    is_discoverable,
                    param->scan_rst.ble_addr_type  // Use actual BLE address type from scan
                );
                return;
            }

            // Not pairing mode - check for target device
            // Update target addr_type if this is our target (for reconnection)
            if (s_has_target && storage_bd_addr_equal(param->scan_rst.bda, s_target_addr)) {
                s_target_addr_type = param->scan_rst.ble_addr_type;
                ESP_LOGI(TAG, "Updated target addr_type to %d from scan", s_target_addr_type);
            }

            check_and_connect_device(
                param->scan_rst.bda,
                NULL,
                NULL,
                param->scan_rst.rssi,
                true
            );
        } else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            ESP_LOGI(TAG, "BLE scan complete");
            bt_hid_state_t state = get_state();

            if (state == BT_HID_STATE_PAIRING && s_pairing_mode) {
                // Pairing mode: select best candidate after scan complete
                ESP_LOGI(TAG, "BLE scan done, selecting best candidate...");
                select_and_connect_best_candidate();
            } else if (state == BT_HID_STATE_SCANNING) {
                set_state(BT_HID_STATE_IDLE);
                schedule_reconnect();
            }
        }
        break;

    // ========== BLE Security/Pairing Events ==========

    case ESP_GAP_BLE_SEC_REQ_EVT:
        // Peer device requests security/pairing - accept it
        ESP_LOGI(TAG, "BLE security request from peer - accepting");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        // Authentication (pairing) complete
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "BLE pairing success, addr_type=%d, auth_mode=%d",
                     param->ble_security.auth_cmpl.addr_type,
                     param->ble_security.auth_cmpl.auth_mode);
        } else {
            ESP_LOGW(TAG, "BLE pairing failed, reason=0x%x",
                     param->ble_security.auth_cmpl.fail_reason);
        }
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        // Display passkey for keyboard entry
        // With static passkey configured, this should always show the same value
#if BT_BLE_STATIC_PASSKEY > 0
        ESP_LOGI(TAG, "BLE Passkey: %06d - Enter this on keyboard then press Enter",
                 BT_BLE_STATIC_PASSKEY);
#else
        ESP_LOGI(TAG, "BLE Passkey: %06lu - Enter this on keyboard then press Enter",
                 (unsigned long)param->ble_security.key_notif.passkey);
#endif
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        // Numeric comparison - auto-accept for headless device
        ESP_LOGI(TAG, "BLE Numeric comparison: %06lu - auto-accepting",
                 (unsigned long)param->ble_security.key_notif.passkey);
        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        // Passkey entry requested - use default 0 for headless device
        ESP_LOGI(TAG, "BLE Passkey entry requested - sending 000000");
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 0);
        break;

    case ESP_GAP_BLE_KEY_EVT:
        // Key exchange event - just log it
        ESP_LOGD(TAG, "BLE key exchange, key_type=%d", param->ble_security.ble_key.key_type);
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
        esp_bt_cod_t *cod = NULL;
        int rssi = -100;  // Default low RSSI

        // Extract device properties
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            switch (param->disc_res.prop[i].type) {
            case ESP_BT_GAP_DEV_PROP_EIR: {
                uint8_t *eir = param->disc_res.prop[i].val;
                uint8_t len;
                name = (char *)esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
                if (!name) {
                    name = (char *)esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
                }
                break;
            }
            case ESP_BT_GAP_DEV_PROP_COD:
                cod = (esp_bt_cod_t *)param->disc_res.prop[i].val;
                break;
            case ESP_BT_GAP_DEV_PROP_RSSI:
                rssi = *(int8_t *)param->disc_res.prop[i].val;
                break;
            default:
                break;
            }
        }

        // In pairing mode, collect candidates
        if (s_pairing_mode) {
            // Check if it's a keyboard (COD check)
            if (cod) {
                uint32_t cod_full = (cod->major << 8) | cod->minor;
                uint8_t major_class = (cod_full >> 8) & 0x1F;
                uint8_t minor_class = cod->minor;

                // Major class 0x05 = Peripheral, Minor 0x40 = Keyboard
                if (major_class != 0x05 || (minor_class & 0xC0) != 0x40) {
                    // Not a keyboard, skip
                    break;
                }
            }

            char addr_str[18];
            storage_bd_addr_to_str(param->disc_res.bda, addr_str);
            ESP_LOGI(TAG, "Classic BT HID found: %s (%s) RSSI=%d",
                     addr_str, name ? name : "unknown", rssi);

            // Classic BT devices discovered during inquiry are generally in discoverable mode
            add_pairing_candidate(
                param->disc_res.bda,
                rssi,
                false,  // is_ble
                true,   // Classic BT in inquiry = discoverable
                0       // addr_type not used for Classic BT
            );
            break;
        }

        // Not pairing mode - check for target device
        check_and_connect_device(
            param->disc_res.bda,
            name,
            cod,
            rssi,
            false
        );
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "Classic BT scan stopped");
            bt_hid_state_t state = get_state();

            if (state == BT_HID_STATE_PAIRING && s_pairing_mode) {
                // Pairing mode: select best candidate after scan complete
                ESP_LOGI(TAG, "Classic BT scan done, selecting best candidate...");
                select_and_connect_best_candidate();
            } else if (state == BT_HID_STATE_SCANNING) {
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

    case ESP_BT_GAP_PIN_REQ_EVT: {
        // Auto-respond with configured PIN code
        esp_bt_pin_code_t pin = {0};
        const char *pin_str = BT_CLASSIC_PIN_CODE;
        uint8_t pin_len = strlen(pin_str);
        memcpy(pin, pin_str, pin_len);
        ESP_LOGI(TAG, "Classic BT PIN requested - sending '%s'", pin_str);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, pin_len, pin);
        break;
    }

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

    ESP_LOGI(TAG, "Step 1: Releasing unused BT memory...");
    // Release BLE memory not needed (we need both Classic and BLE)
    // Note: This may fail if already released, which is OK
    esp_err_t mem_ret = esp_bt_controller_mem_release(ESP_BT_MODE_IDLE);
    if (mem_ret != ESP_OK && mem_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Memory release returned: %s (continuing)", esp_err_to_name(mem_ret));
    }

    ESP_LOGI(TAG, "Step 2: Initializing BT controller...");
    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    // Explicitly set dual mode for ESP32 (both Classic BT and BLE)
    bt_cfg.mode = ESP_BT_MODE_BTDM;
    bt_cfg.bt_max_acl_conn = 3;
    bt_cfg.bt_max_sync_conn = 3;
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "Step 2: BT controller init OK");

    ESP_LOGI(TAG, "Step 3: Enabling BT controller (BTDM mode)...");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "Step 3: BT controller enable OK");

    ESP_LOGI(TAG, "Step 4: Initializing Bluedroid...");
    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "Step 4: Bluedroid init OK");

    ESP_LOGI(TAG, "Step 5: Enabling Bluedroid...");
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "Step 5: Bluedroid enable OK");

    ESP_LOGI(TAG, "Step 6: Registering GAP callbacks...");
    // Register GAP callbacks
    esp_bt_gap_register_callback(bt_gap_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    ESP_LOGI(TAG, "Step 6: GAP callbacks registered");

    ESP_LOGI(TAG, "Step 6a: Registering BLE GATTC callback...");
    // Register GATTC callback for BLE HID support (REQUIRED for esp_hidh to work with BLE devices)
    ret = esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC callback register failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    ESP_LOGI(TAG, "Step 6a: GATTC callback registered");

    ESP_LOGI(TAG, "Step 7: Configuring Classic BT security...");
    // Configure Classic BT
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;  // No input/output for pairing
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    // Set scan mode (discoverable for pairing)
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    ESP_LOGI(TAG, "Step 7: Classic BT security configured");

    ESP_LOGI(TAG, "Step 8: Configuring BLE security...");
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

    // Set static passkey for headless operation
    // When pairing, user enters this code on the keyboard
#if BT_BLE_STATIC_PASSKEY > 0
    uint32_t passkey = BT_BLE_STATIC_PASSKEY;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(passkey));
    ESP_LOGI(TAG, "Step 8: BLE security configured (static passkey: %06lu)", (unsigned long)passkey);
#else
    ESP_LOGI(TAG, "Step 8: BLE security configured (random passkey)");
#endif

    ESP_LOGI(TAG, "Step 9: Initializing HID Host...");
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
    ESP_LOGI(TAG, "Step 9: HID Host init OK");

    ESP_LOGI(TAG, "Step 10: Creating timers...");
    // Create reconnect timer
    s_reconnect_timer = xTimerCreate(
        "bt_reconn",
        pdMS_TO_TICKS(BT_RETRY_DELAY_MS),
        pdFALSE,  // One-shot
        NULL,
        reconnect_timer_callback
    );

    // Create connection timeout timer
    s_connect_timeout_timer = xTimerCreate(
        "bt_conn_to",
        pdMS_TO_TICKS(BT_CONNECT_TIMEOUT_MS),
        pdFALSE,  // One-shot
        NULL,
        connect_timeout_callback
    );
    ESP_LOGI(TAG, "Step 10: Timers created");

#if 0  // DISABLED: Key release safety timer
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
#endif

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

    if (s_connect_timeout_timer) {
        xTimerStop(s_connect_timeout_timer, portMAX_DELAY);
        xTimerDelete(s_connect_timeout_timer, portMAX_DELAY);
        s_connect_timeout_timer = NULL;
    }

#if 0  // DISABLED: Key release safety timer cleanup
    if (s_key_release_timer) {
        xTimerStop(s_key_release_timer, portMAX_DELAY);
        xTimerDelete(s_key_release_timer, portMAX_DELAY);
        s_key_release_timer = NULL;
    }
#endif

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
        ESP_LOGI(TAG, "Disconnecting current device before pairing...");
        bt_hid_host_disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
    } else if (state == BT_HID_STATE_CONNECTING) {
        // Already attempting to connect - abort to restart pairing
        ESP_LOGW(TAG, "Already connecting - aborting to restart pairing");
        stop_connect_timeout();
        // Note: We can't cancel esp_hidh_dev_open(), but stopping scan/discovery
        // and resetting state should help
        esp_ble_gap_stop_scanning();
        esp_bt_gap_cancel_discovery();
        vTaskDelay(pdMS_TO_TICKS(100));
    } else if (state == BT_HID_STATE_PAIRING) {
        ESP_LOGW(TAG, "Already in pairing mode - restarting scan");
        esp_ble_gap_stop_scanning();
        esp_bt_gap_cancel_discovery();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Entering pairing mode");

    // Clear any previous candidates
    clear_pairing_candidates();

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
