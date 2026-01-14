/**
 * @file storage.c
 * @brief NVS storage implementation for Bluetooth pairing information
 */

#include "storage.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "STORAGE";

/* ============================================================================
 * Private Constants
 * ============================================================================ */

#define NVS_NAMESPACE           "bt_hid_proxy"
#define NVS_KEY_PAIRED_DEVICE   "paired_dev"

/* ============================================================================
 * Private Variables
 * ============================================================================ */

static bool s_initialized = false;

/* ============================================================================
 * Public Functions
 * ============================================================================ */

esp_err_t storage_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "NVS storage initialized");

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
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save device as blob
    ret = nvs_set_blob(handle, NVS_KEY_PAIRED_DEVICE, device, sizeof(paired_device_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write paired device: %s", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    nvs_close(handle);

    if (ret == ESP_OK) {
        char addr_str[18];
        storage_bd_addr_to_str(device->bd_addr, addr_str);
        ESP_LOGI(TAG, "Saved paired device: %s (BLE=%d)", addr_str, device->is_ble);
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

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No paired device stored");
        } else {
            ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        }
        memset(device, 0, sizeof(paired_device_t));
        device->valid = false;
        return ESP_ERR_NOT_FOUND;
    }

    size_t required_size = sizeof(paired_device_t);
    ret = nvs_get_blob(handle, NVS_KEY_PAIRED_DEVICE, device, &required_size);
    nvs_close(handle);

    if (ret != ESP_OK || required_size != sizeof(paired_device_t)) {
        ESP_LOGW(TAG, "Failed to load paired device: %s", esp_err_to_name(ret));
        memset(device, 0, sizeof(paired_device_t));
        device->valid = false;
        return ESP_ERR_NOT_FOUND;
    }

    if (!device->valid) {
        ESP_LOGI(TAG, "Stored device marked as invalid");
        return ESP_ERR_NOT_FOUND;
    }

    char addr_str[18];
    storage_bd_addr_to_str(device->bd_addr, addr_str);
    ESP_LOGI(TAG, "Loaded paired device: %s (BLE=%d)", addr_str, device->is_ble);

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

    ret = nvs_erase_key(handle, NVS_KEY_PAIRED_DEVICE);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ret = ESP_OK;  // Already cleared
    }

    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

    nvs_close(handle);

    ESP_LOGI(TAG, "Paired device cleared");
    return ret;
}

bool storage_has_paired_device(void)
{
    paired_device_t device;
    return (storage_load_paired_device(&device) == ESP_OK && device.valid);
}

void storage_bd_addr_to_str(const esp_bd_addr_t bd_addr, char *str)
{
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            bd_addr[0], bd_addr[1], bd_addr[2],
            bd_addr[3], bd_addr[4], bd_addr[5]);
}

bool storage_bd_addr_equal(const esp_bd_addr_t addr1, const esp_bd_addr_t addr2)
{
    return (memcmp(addr1, addr2, sizeof(esp_bd_addr_t)) == 0);
}
