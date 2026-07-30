/**
 * @file storage.h
 * @brief NVS storage for BLE pairing information
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stored peer identity
 *
 * addr/addr_type follow NimBLE conventions (ble_addr_t equivalent):
 * addr_type 0 = public, 1 = static random.
 */
typedef struct {
    uint8_t addr[6];        // Little-endian, as used by NimBLE
    uint8_t addr_type;
    bool valid;
    char name[32];
} paired_device_t;

esp_err_t storage_init(void);

esp_err_t storage_save_paired_device(const paired_device_t *device);
esp_err_t storage_load_paired_device(paired_device_t *device);
esp_err_t storage_clear_paired_device(void);

/**
 * @brief Format a NimBLE little-endian address as "AA:BB:CC:DD:EE:FF"
 * @param str Output buffer, at least 18 bytes
 */
void storage_addr_to_str(const uint8_t addr[6], char *str);

#ifdef __cplusplus
}
#endif

#endif /* STORAGE_H */
