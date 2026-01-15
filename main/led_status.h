/**
 * @file led_status.h
 * @brief RGB LED status indicator for BT HID Proxy
 *
 * Uses SK6812 (or WS2812) addressable RGB LED on M5Stamp Pico.
 */

#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>
#include "esp_err.h"

/* ============================================================================
 * LED Status Modes
 * ============================================================================ */

typedef enum {
    LED_STATUS_OFF = 0,
    LED_STATUS_IDLE,        // Slow red blink
    LED_STATUS_SCANNING,    // Fast blue blink
    LED_STATUS_PAIRING,     // Blue/green alternating
    LED_STATUS_CONNECTING,  // Slow green blink
    LED_STATUS_CONNECTED,   // Solid green
    LED_STATUS_ERROR,       // Fast red blink
} led_status_mode_t;

/* ============================================================================
 * Function Declarations
 * ============================================================================ */

/**
 * @brief Initialize LED status indicator
 * @return ESP_OK on success
 */
esp_err_t led_status_init(void);

/**
 * @brief De-initialize LED status indicator
 */
void led_status_deinit(void);

/**
 * @brief Set LED status mode
 * @param mode The status mode to display
 */
void led_status_set(led_status_mode_t mode);

/**
 * @brief Get current LED status mode
 * @return Current mode
 */
led_status_mode_t led_status_get(void);

/**
 * @brief Set custom RGB color (overrides mode)
 * @param r Red (0-255)
 * @param g Green (0-255)
 * @param b Blue (0-255)
 */
void led_status_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Turn LED off
 */
void led_status_off(void);

#endif /* LED_STATUS_H */
