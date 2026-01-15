/**
 * @file button.h
 * @brief Button input handler for pairing mode trigger
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include "esp_err.h"

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

/**
 * @brief Button event types
 */
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,   // Normal click
    BUTTON_EVENT_LONG_PRESS,    // Long press (>= threshold)
} button_event_t;

/**
 * @brief Callback function type for button events
 * @param event The button event type
 */
typedef void (*button_callback_t)(button_event_t event);

/* ============================================================================
 * Function Declarations
 * ============================================================================ */

/**
 * @brief Initialize button handler
 * @param callback Function to call on button events (NULL to disable)
 * @return ESP_OK on success
 */
esp_err_t button_init(button_callback_t callback);

/**
 * @brief De-initialize button handler
 */
void button_deinit(void);

/**
 * @brief Check if button is currently pressed
 * @return true if pressed
 */
bool button_is_pressed(void);

/**
 * @brief Set callback for button events
 * @param callback Function to call (NULL to disable)
 */
void button_set_callback(button_callback_t callback);

#endif /* BUTTON_H */
