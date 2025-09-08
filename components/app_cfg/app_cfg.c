#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "app_cfg.h"

static char s_device_id[16] = {0};

esp_err_t app_cfg_init(void)
{
    // Build "IAQ-XXXX" from base MAC last two bytes
    uint8_t mac[6] = {0};
    // Prefer base MAC from eFUSE (stable)
    esp_err_t err = esp_efuse_mac_get_default(mac);
    if (err != ESP_OK) {
        // Fallback to station MAC if needed
        err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
        if (err != ESP_OK) {
            // Last resort: a fixed id to avoid NULLs
            strcpy(s_device_id, "IAQ-0000");
            return err;
        }
    }
    // last two bytes hex, uppercase, zero-padded
    snprintf(s_device_id, sizeof(s_device_id), "IAQ-%02X%02X", mac[4], mac[5]);
    return ESP_OK;
}

const char* app_cfg_get_device_id(void)
{
    // If init wasn’t called yet, compute once lazily.
    if (s_device_id[0] == 0) {
        (void)app_cfg_init();
    }
    return s_device_id;
}
