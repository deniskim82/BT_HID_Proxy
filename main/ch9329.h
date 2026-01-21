/**
 * @file ch9329.h
 * @brief CH9329 UART to USB HID controller driver
 *
 * Protocol Reference:
 * - WCH Official: https://www.wch-ic.com/products/CH9329.html
 * - Protocol implementation based on: https://github.com/Blue-Beaker/9329KeyboardRemote
 *
 * Frame Format:
 * [0x57][0xAB][ADDR][CMD][LEN][DATA...][CHECKSUM]
 */

#ifndef CH9329_H
#define CH9329_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ============================================================================
 * CH9329 Protocol Constants
 * ============================================================================ */

// Frame header
#define CH9329_HEADER_1         0x57
#define CH9329_HEADER_2         0xAB

// Default address
#define CH9329_ADDR_DEFAULT     0x00

// Commands
#define CH9329_CMD_GET_INFO     0x01    // Get chip info
#define CH9329_CMD_KEYBOARD     0x02    // Send keyboard report
#define CH9329_CMD_MEDIA_KEY    0x03    // Send media key
#define CH9329_CMD_MOUSE_ABS    0x04    // Absolute mouse position
#define CH9329_CMD_MOUSE_REL    0x05    // Relative mouse movement
#define CH9329_CMD_READ_CFG     0x08    // Read configuration
#define CH9329_CMD_WRITE_CFG    0x09    // Write configuration
#define CH9329_CMD_RESPONSE     0x81    // Response from chip

// Keyboard report size (standard USB HID)
#define CH9329_KB_REPORT_SIZE   8

// Modifier key bit positions (same as USB HID)
#define CH9329_MOD_LCTRL        0x01
#define CH9329_MOD_LSHIFT       0x02
#define CH9329_MOD_LALT         0x04
#define CH9329_MOD_LGUI         0x08
#define CH9329_MOD_RCTRL        0x10
#define CH9329_MOD_RSHIFT       0x20
#define CH9329_MOD_RALT         0x40
#define CH9329_MOD_RGUI         0x80

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Keyboard report structure (USB HID standard)
 */
typedef struct {
    uint8_t modifier;       // Modifier keys (Ctrl, Shift, Alt, GUI)
    uint8_t reserved;       // Reserved (always 0x00)
    uint8_t keys[6];        // Up to 6 simultaneous key codes
} ch9329_keyboard_report_t;

/* ============================================================================
 * Function Declarations
 * ============================================================================ */

/**
 * @brief Initialize CH9329 UART communication
 * @return ESP_OK on success
 */
esp_err_t ch9329_init(void);

/**
 * @brief De-initialize CH9329
 */
void ch9329_deinit(void);

/**
 * @brief Send keyboard report to CH9329
 * @param report Pointer to keyboard report
 * @return ESP_OK on success
 */
esp_err_t ch9329_send_keyboard_report(const ch9329_keyboard_report_t *report);

/**
 * @brief Send raw keyboard data (8 bytes) to CH9329
 * @param data Pointer to 8-byte keyboard data
 * @return ESP_OK on success
 */
esp_err_t ch9329_send_keyboard_raw(const uint8_t *data);

/**
 * @brief Release all keys (send empty report)
 * @return ESP_OK on success
 */
esp_err_t ch9329_release_all_keys(void);

/**
 * @brief Check if CH9329 is ready
 * @return true if initialized and ready
 */
bool ch9329_is_ready(void);

/**
 * @brief LED status callback type
 * @param led_status LED status byte (Bit0=NumLock, Bit1=CapsLock, Bit2=ScrollLock)
 */
typedef void (*ch9329_led_cb_t)(uint8_t led_status);

/**
 * @brief Set LED status callback
 * @param cb Callback function to receive LED status updates
 */
void ch9329_set_led_callback(ch9329_led_cb_t cb);

/**
 * @brief Start UART RX task for LED status monitoring
 * @return ESP_OK on success
 */
esp_err_t ch9329_start_rx_task(void);

/**
 * @brief Stop UART RX task
 */
void ch9329_stop_rx_task(void);

#endif /* CH9329_H */
