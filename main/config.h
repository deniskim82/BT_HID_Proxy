/**
 * @file config.h
 * @brief Global configuration for BT HID Proxy
 *
 * Hardware: M5Stamp Pico (ESP32-PICO-D4)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/uart.h"

/* ============================================================================
 * Debug Configuration
 * ============================================================================ */

// Set to 1 to enable verbose debug logging (affects performance)
#define DEBUG_VERBOSE           0

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

// RGB LED (SK6812) - Reserved for future use
#define RGB_LED_GPIO            GPIO_NUM_27

/* ============================================================================
 * Timing Configuration
 * ============================================================================ */

// Button long press threshold for pairing mode (milliseconds)
#define BUTTON_LONG_PRESS_MS    5000

// BT scan timeout (seconds)
#define BT_SCAN_TIMEOUT_SEC     10

// Connection retry delay (milliseconds)
#define BT_RETRY_DELAY_MS       3000

// Key release timeout - auto-release all keys if no input (milliseconds)
#define KEY_RELEASE_TIMEOUT_MS  100

/* ============================================================================
 * Bluetooth Configuration
 * ============================================================================ */

// Device name for BT discovery
#define BT_DEVICE_NAME          "BT_HID_Proxy"

// Maximum stored paired devices
#define MAX_PAIRED_DEVICES      1

/* ============================================================================
 * CH9329 Configuration
 * ============================================================================ */

// Frame buffer size
#define CH9329_FRAME_BUF_SIZE   64

// UART buffer sizes (TX larger for safety margin)
#define CH9329_UART_TX_BUF_SIZE 512
#define CH9329_UART_RX_BUF_SIZE 64

// Pre-computed checksum base for keyboard frame header
// Header: 0x57 + 0xAB + 0x00 + 0x02 + 0x08 = 0x0C
#define CH9329_KB_HEADER_CHECKSUM_BASE  0x0C

#endif /* CONFIG_H */
