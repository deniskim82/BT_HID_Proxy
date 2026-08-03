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
#define CH9329_CMD_GET_INFO     0x01
#define CH9329_CMD_KEYBOARD     0x02
#define CH9329_CMD_MEDIA        0x03
#define CH9329_CMD_GET_PARA_CFG 0x08
/* NOTE: 0x0D is CMD_RESET on this chip, NOT a "get LED" command. The CH9329
 * has no dedicated LED query - the host's Caps/Num/Scroll state is carried in
 * the GET_INFO (0x01) reply. Sending 0x0D resets the chip and drops its USB
 * enumeration, which is exactly what an earlier version of this driver did on
 * every connect and every lock-key press. */
#define CH9329_CMD_RESET        0x0D
#define CH9329_CMD_INFO_RESPONSE 0x81
#define CH9329_CMD_PARA_CFG_RESPONSE 0x88
#define CH9329_CMD_KB_ACK       0x82
#define CH9329_CMD_RESET_RESPONSE 0x8D
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
 * @brief Forward the currently pressed consumer-page usages (media, browser
 *        and application keys).
 *
 * The CH9329 does not take usage codes: it defines a fixed bitmap, so the
 * usages are translated here. Pass count 0 to release everything. Usages with
 * no CH9329 equivalent are ignored.
 */
esp_err_t ch9329_send_consumer_usages(const uint16_t *usages, int count);

/**
 * @brief Forward system control usages (Power 0x81, Sleep 0x82, Wake 0x83).
 *
 * Pass count 0 to release.
 */
esp_err_t ch9329_send_system_usages(const uint16_t *usages, int count);

/**
 * @brief Queue a request for the host's LED state.
 *
 * Implemented as GET_INFO: the reply carries the Caps/Num/Scroll state, and
 * the LED callback is invoked when it changes.
 */
esp_err_t ch9329_request_led_status(void);

/**
 * @brief Queue a GET_INFO request; the reply is logged.
 *
 * The reply carries the chip version and, importantly, whether the CH9329
 * believes it is enumerated on the USB host. That separates "chip is dead or
 * not listening on UART" from "chip is fine but its USB side never came up".
 */
esp_err_t ch9329_request_info(void);

/**
 * @brief Queue a GET_PARA_CFG request; the stored configuration is logged.
 *
 * Reads back what the chip actually has in its flash - USB working mode,
 * serial mode, chip address, baud rate, VID/PID - so a configuration that
 * breaks USB enumeration can be identified without the vendor tool.
 */
esp_err_t ch9329_request_para_cfg(void);

/**
 * @brief Baud rate the driver settled on (after auto-detection).
 */
int ch9329_get_baud_rate(void);

/**
 * @brief Register callback invoked when the host PC's LED state changes.
 */
void ch9329_set_led_callback(ch9329_led_cb_t cb);

bool ch9329_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* CH9329_H */
