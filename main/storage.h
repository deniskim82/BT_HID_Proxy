/**
 * @file storage.h
 * @brief NVS storage for Bluetooth pairing information
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_bt_defs.h"

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Paired device information
 */
typedef struct {
    esp_bd_addr_t bd_addr;      // Bluetooth device address (6 bytes)
    uint8_t addr_type;          // Address type (public/random for BLE)
    bool is_ble;                // true if BLE, false if Classic BT
    bool valid;                 // true if entry contains valid data
} paired_device_t;

/* ============================================================================
 * Function Declarations
 * ============================================================================ */

/**
 * @brief Initialize NVS storage
 * @return ESP_OK on success
 */
esp_err_t storage_init(void);

/**
 * @brief Save paired device information
 * @param device Pointer to device info to save
 * @return ESP_OK on success
 */
esp_err_t storage_save_paired_device(const paired_device_t *device);

/**
 * @brief Load paired device information
 * @param device Pointer to receive device info
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if no device saved
 */
esp_err_t storage_load_paired_device(paired_device_t *device);

/**
 * @brief Clear saved paired device
 * @return ESP_OK on success
 */
esp_err_t storage_clear_paired_device(void);

/**
 * @brief Check if paired device exists in storage
 * @return true if valid paired device exists
 */
bool storage_has_paired_device(void);

/**
 * @brief Get BD address as string (for logging)
 * @param bd_addr Bluetooth device address
 * @param str Output buffer (minimum 18 bytes for "XX:XX:XX:XX:XX:XX\0")
 */
void storage_bd_addr_to_str(const esp_bd_addr_t bd_addr, char *str);

/**
 * @brief Compare two BD addresses
 * @param addr1 First address
 * @param addr2 Second address
 * @return true if equal
 */
bool storage_bd_addr_equal(const esp_bd_addr_t addr1, const esp_bd_addr_t addr2);

#endif /* STORAGE_H */
