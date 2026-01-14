/**
 * @file bt_hid_host.h
 * @brief Bluetooth HID Host for keyboard input
 *
 * Supports both Classic Bluetooth and BLE HID devices.
 * Based on ESP-IDF esp_hid_host example.
 *
 * Reference:
 * - ESP-IDF HID Host API: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/bluetooth/esp_hidh.html
 * - ESP-IDF HID Host Example: https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/esp_hid_host
 */

#ifndef BT_HID_HOST_H
#define BT_HID_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_bt_defs.h"

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief Connection state
 */
typedef enum {
    BT_HID_STATE_IDLE = 0,
    BT_HID_STATE_SCANNING,
    BT_HID_STATE_CONNECTING,
    BT_HID_STATE_CONNECTED,
    BT_HID_STATE_PAIRING,
    BT_HID_STATE_ERROR,
} bt_hid_state_t;

/**
 * @brief Keyboard input callback
 * @param data Pointer to 8-byte HID keyboard report
 */
typedef void (*bt_hid_keyboard_cb_t)(const uint8_t *data);

/**
 * @brief State change callback
 * @param state New state
 */
typedef void (*bt_hid_state_cb_t)(bt_hid_state_t state);

/**
 * @brief Device found callback for pairing mode
 * @param bd_addr Device address
 * @param name Device name (may be NULL)
 * @param is_ble true if BLE device
 */
typedef void (*bt_hid_device_found_cb_t)(const esp_bd_addr_t bd_addr, const char *name, bool is_ble);

/* ============================================================================
 * Function Declarations
 * ============================================================================ */

/**
 * @brief Initialize Bluetooth HID Host
 * @param keyboard_cb Callback for keyboard input (required)
 * @param state_cb Callback for state changes (optional, can be NULL)
 * @return ESP_OK on success
 */
esp_err_t bt_hid_host_init(bt_hid_keyboard_cb_t keyboard_cb, bt_hid_state_cb_t state_cb);

/**
 * @brief De-initialize Bluetooth HID Host
 */
void bt_hid_host_deinit(void);

/**
 * @brief Start scanning for saved device
 * @param target_addr Address of device to find (NULL to scan for any HID device)
 * @param is_ble true if target is BLE device
 * @return ESP_OK if scan started
 */
esp_err_t bt_hid_host_scan_for_device(const esp_bd_addr_t target_addr, bool is_ble);

/**
 * @brief Start pairing mode - scan for any HID keyboard
 * @return ESP_OK if scan started
 */
esp_err_t bt_hid_host_start_pairing(void);

/**
 * @brief Stop current operation (scan/connect)
 */
void bt_hid_host_stop(void);

/**
 * @brief Disconnect current device
 */
void bt_hid_host_disconnect(void);

/**
 * @brief Get current connection state
 * @return Current state
 */
bt_hid_state_t bt_hid_host_get_state(void);

/**
 * @brief Check if a device is connected
 * @return true if connected
 */
bool bt_hid_host_is_connected(void);

/**
 * @brief Get connected device address
 * @param bd_addr Output buffer for address
 * @param is_ble Output: true if BLE device
 * @return ESP_OK if connected, ESP_ERR_INVALID_STATE otherwise
 */
esp_err_t bt_hid_host_get_connected_device(esp_bd_addr_t bd_addr, bool *is_ble);

#endif /* BT_HID_HOST_H */
