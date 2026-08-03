/**
 * @file ble_hid_host.h
 * @brief BLE HID (HOGP) Host based on NimBLE
 *
 * Connects to a bonded BLE keyboard, parses its HID report map, subscribes
 * only to the keyboard input report, and delivers translated 8-byte boot
 * reports via callback. Supports LED (Caps/Num/Scroll) output forwarding.
 */

#ifndef BLE_HID_HOST_H
#define BLE_HID_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BLE_HID_STATE_IDLE = 0,     // No bonded device / waiting
    BLE_HID_STATE_SCANNING,     // Scanning for the bonded keyboard
    BLE_HID_STATE_CONNECTING,   // Connecting / encrypting / discovering
    BLE_HID_STATE_CONNECTED,    // Subscribed, relaying input
    BLE_HID_STATE_PAIRING,      // Pairing mode: scanning for a new keyboard
    BLE_HID_STATE_ERROR,        // Repeated failures; still retrying
} ble_hid_state_t;

/**
 * @brief Keyboard state callback.
 *
 * Reports the modifiers and every keycode currently held on one source
 * device - not a boot report, and not truncated to six keys. Merging the
 * devices and capping to the boot report's six slots happens in key_state.
 *
 * @param source  Index identifying which subscribed report this came from.
 *                A keyboard exposing both a 6KRO and an NKRO report produces
 *                two sources, and they must be tracked separately: whichever
 *                one the keyboard is not currently using reads as empty.
 * @param modifiers Modifier byte from that report
 * @param keys      Keycodes currently held in that report
 * @param count     Number of entries in @p keys
 *
 * Called from the NimBLE host task; must not block.
 */
typedef void (*ble_hid_keyboard_cb_t)(int source, uint8_t modifiers,
                                      const uint8_t *keys, int count);

typedef void (*ble_hid_state_cb_t)(ble_hid_state_t state);

/**
 * @brief Non-keyboard usage report callback (media / system control keys).
 *
 * @param is_system true for system control (power/sleep/wake), false for
 *                  consumer page (media, browser, application keys)
 * @param usages    currently pressed usage codes (empty = all released)
 * @param count     number of usages
 *
 * Called from the NimBLE host task; must not block.
 */
typedef void (*ble_hid_ext_cb_t)(bool is_system, const uint16_t *usages, int count);

/**
 * @brief Register the media / system control key callback (optional).
 *
 * Must be called before ble_hid_host_init().
 */
void ble_hid_host_set_ext_cb(ble_hid_ext_cb_t cb);

/**
 * @brief Initialize the BLE stack and start the connection manager task.
 *
 * If a bonded keyboard is stored in NVS, reconnection starts automatically.
 * Otherwise the module stays idle until ble_hid_host_start_pairing().
 */
esp_err_t ble_hid_host_init(ble_hid_keyboard_cb_t keyboard_cb, ble_hid_state_cb_t state_cb);

/**
 * @brief Enter pairing mode: scan, connect to the strongest keyboard, bond,
 *        and replace any previously stored pairing.
 *
 * Non-blocking; safe to call from timer context (posts a command).
 */
esp_err_t ble_hid_host_start_pairing(void);

/**
 * @brief Forward host LED state (Bit0=Num, Bit1=Caps, Bit2=Scroll) to the keyboard.
 */
esp_err_t ble_hid_host_send_led_status(uint8_t led_status);

ble_hid_state_t ble_hid_host_get_state(void);
bool ble_hid_host_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_HID_HOST_H */
