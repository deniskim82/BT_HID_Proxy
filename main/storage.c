/**
 * @file storage.c
 * @brief NVS storage for BLE pairing information
 */

#include "storage.h"
#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "STORAGE";

#define NVS_NAMESPACE       "bt_hid_proxy"
// New key: the blob layout changed in the BLE-only rewrite, so a stale blob
// from the old firmware must not be picked up.
#define NVS_KEY_PAIRED_BLE  "paired_ble"

static bool s_initialized = false;

esp_err_t storage_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t storage_save_paired_device(const paired_device_t *device)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_blob(handle, NVS_KEY_PAIRED_BLE, device, sizeof(paired_device_t));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    if (ret == ESP_OK) {
        char addr_str[18];
        storage_addr_to_str(device->addr, addr_str);
        ESP_LOGI(TAG, "Saved paired device: %s (type=%d) '%s'",
                 addr_str, device->addr_type, device->name);
    } else {
        ESP_LOGE(TAG, "Failed to save paired device: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t storage_load_paired_device(paired_device_t *device)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(device, 0, sizeof(paired_device_t));

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    size_t size = sizeof(paired_device_t);
    ret = nvs_get_blob(handle, NVS_KEY_PAIRED_BLE, device, &size);
    nvs_close(handle);

    if (ret != ESP_OK || size != sizeof(paired_device_t) || !device->valid) {
        memset(device, 0, sizeof(paired_device_t));
        return ESP_ERR_NOT_FOUND;
    }

    char addr_str[18];
    storage_addr_to_str(device->addr, addr_str);
    ESP_LOGI(TAG, "Loaded paired device: %s (type=%d) '%s'",
             addr_str, device->addr_type, device->name);

    return ESP_OK;
}

esp_err_t storage_clear_paired_device(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_erase_key(handle, NVS_KEY_PAIRED_BLE);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Paired device cleared");
    return ret;
}

void storage_addr_to_str(const uint8_t addr[6], char *str)
{
    if (str == NULL) {
        return;
    }
    // NimBLE addresses are little-endian; print MSB first
    snprintf(str, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}
