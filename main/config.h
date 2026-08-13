/**
 * @file config.h
 * @brief Global configuration for BT HID Proxy (BLE-only rewrite)
 *
 * Hardware: M5Stamp Pico (ESP32-PICO-D4)
 * All pin mappings and UART settings are unchanged from the original design.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/uart.h"

/* ============================================================================
 * Hardware Pin Configuration (M5Stamp Pico)
 * ============================================================================ */

// UART for CH9329 communication
#define CH9329_UART_NUM         UART_NUM_1
#define CH9329_UART_TX_PIN      GPIO_NUM_32
#define CH9329_UART_RX_PIN      GPIO_NUM_33
#define CH9329_UART_BAUD_RATE   115200

// Built-in button on M5Stamp Pico
#define BUTTON_GPIO             GPIO_NUM_39

// RGB LED (SK6812)
#define RGB_LED_GPIO            GPIO_NUM_27

/* ============================================================================
 * Timing Configuration
 * ============================================================================ */

// Button long press threshold for pairing mode (milliseconds)
#define BUTTON_LONG_PRESS_MS    5000

// Pairing scan window (seconds) - collect candidates, then connect to best RSSI
#define BLE_PAIRING_SCAN_SEC    6

// Reconnect: scan-for-advertisement window (seconds)
#define BLE_RECONNECT_SCAN_SEC  8

// Reconnect: direct connection attempt timeout (milliseconds)
#define BLE_DIRECT_CONNECT_TIMEOUT_MS  4000

// Delay between reconnect cycles (milliseconds)
#define BLE_RECONNECT_DELAY_MS  500

// Delay after connecting before polling initial LED state (milliseconds)
#define LED_POLL_AFTER_CONNECT_MS   300

// Delay after a Caps/Num/Scroll key press before polling LED state (milliseconds)
#define LED_POLL_AFTER_KEY_MS       100

/* ============================================================================
 * Bluetooth Configuration
 * ============================================================================ */

// Pairing is always passkey-free (Just Works) - see ble_hid_host_init().
//
// There used to be a BLE_STATIC_PASSKEY here. It bought nothing: a passkey
// only protects against an active MITM as long as the attacker cannot guess
// it, and a value compiled into open-source firmware is not a secret. The
// board has a single RGB LED, so it cannot display a random per-pairing
// passkey either - which is why it now declares NO_IO and never asks for,
// nor accepts, Passkey Entry. The link is still encrypted (LE Secure
// Connections / ECDH), so passive eavesdropping remains defeated.

// Minimum RSSI to accept a keyboard in pairing mode (dBm)
#define BLE_PAIRING_RSSI_MIN    (-80)

/* ============================================================================
 * Diagnostics
 * ============================================================================ */

// Health line cadence (milliseconds) and how often it is also written to the
// persistent log. Console every minute, flash every fifth minute: the ring
// then spans days instead of hours, which is the timescale of the fault it
// exists to catch.
#define DIAG_HEALTH_TICK_MS         60000
#define DIAG_HEALTH_PERSIST_EVERY   5

// How long a connected link may go without input before the spell is recorded
// (milliseconds). Long enough that ordinary pauses at a desk are ignored.
#define DIAG_SILENT_LINK_MS         (15 * 60 * 1000)

/* ============================================================================
 * CH9329 Configuration
 * ============================================================================ */

// ESP32 UART buffer sizes for CH9329 communication
// Note: RX buffer must be > SOC_UART_FIFO_LEN (128 bytes on ESP32)
#define CH9329_UART_TX_BUF_SIZE 512
#define CH9329_UART_RX_BUF_SIZE 256

// Depth of the TX message queue (keyboard reports + control commands)
#define CH9329_TX_QUEUE_LEN     32

// Set to 0 to stop sending media/system key frames (CH9329 command 0x03)
// entirely. Useful for bisecting: if a module misbehaves only once media keys
// are in play, this isolates that command without touching anything else.
#define CH9329_ENABLE_MEDIA_KEYS 1

#endif /* CONFIG_H */
