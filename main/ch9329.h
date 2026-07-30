/**
 * @file ch9329.h
 * @brief CH9329 UART to USB HID controller driver
 *
 * Design:
 *  - All UART TX goes through a single dedicated task fed by a queue, so
 *    frames can never interleave regardless of which task requests a send.
 *  - A dedicated RX task parses status frames (keyboard ACK, LED state).
 *  - The hot path (keyboard report) is a single non-blocking queue post.
 */

#ifndef CH9329_H
#define CH9329_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CH9329 protocol constants */
#define CH9329_HEADER_1         0x57
#define CH9329_HEADER_2         0xAB
#define CH9329_ADDR_DEFAULT     0x00
#define CH9329_CMD_KEYBOARD     0x02
#define CH9329_CMD_GET_LED      0x0D
#define CH9329_CMD_KB_ACK       0x82
#define CH9329_CMD_LED_RESPONSE 0x8D
#define CH9329_KB_REPORT_SIZE   0x08

/**
 * @brief LED status callback
 * @param led_status Bit0=NumLock, Bit1=CapsLock, Bit2=ScrollLock
 */
typedef void (*ch9329_led_cb_t)(uint8_t led_status);

/**
 * @brief Initialize UART driver and start TX/RX tasks
 */
esp_err_t ch9329_init(void);

/**
 * @brief Stop tasks and delete UART driver
 */
void ch9329_deinit(void);

/**
 * @brief Queue an 8-byte boot keyboard report [mod, 0, k1..k6] for sending.
 *
 * Non-blocking; duplicate reports are suppressed by the TX task.
 * Safe to call from any task.
 */
esp_err_t ch9329_send_keyboard_report(const uint8_t data[8]);

/**
 * @brief Queue an all-zero report and reset duplicate suppression.
 *
 * Guarantees a release frame actually goes out on the wire even if the last
 * sent report was already empty.
 */
esp_err_t ch9329_release_all_keys(void);

/**
 * @brief Queue a GET_LED_STATUS request; answer arrives via the LED callback.
 */
esp_err_t ch9329_request_led_status(void);

/**
 * @brief Register callback invoked when the host PC's LED state changes.
 */
void ch9329_set_led_callback(ch9329_led_cb_t cb);

bool ch9329_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* CH9329_H */
