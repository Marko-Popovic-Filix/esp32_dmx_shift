#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "wifi_setup.h"
#include "net_post.h"
#include "dmx_ctrl.h"

static const char *TAG = "MAIN";

static volatile bool s_post_started = false;

// JSON -> DMX
static void on_cmd(const char *json, int len) {
    ESP_LOGI("CMD", "payload (%d): %.*s", len, len, json);
    (void) dmx_ctrl_apply_json(json, len);
}

// Start polling only after Wi-Fi STA has an IP
static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (s_post_started) return;
    net_post_set_callback(on_cmd);
    esp_err_t err = net_post_start_on_core(0);
    if (err == ESP_OK) {
        s_post_started = true;
        ESP_LOGI(TAG, "net_post started (Wi-Fi connected, IP acquired)");
    } else {
        ESP_LOGE(TAG, "net_post_start_on_core failed: 0x%x", err);
    }
}

static void on_wifi_disconnected(void *arg, esp_event_base_t base, int32_t id, void *data) {
    ESP_LOGW(TAG, "Wi-Fi disconnected; polling stays stopped until reconnection");
    // If net_post exposes a stop API, you can stop it here:
    // if (s_post_started) { net_post_stop(); s_post_started = false; }
}

void app_main(void) {
    // NVS for Wi-Fi creds etc.
    ESP_ERROR_CHECK(nvs_flash_init());

    // Bring up netif + default event loop BEFORE any networking
    esp_err_t err;
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);

    // Gate polling by connectivity
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnected, NULL, NULL));

    // Start DMX worker on Core 1 (continuous DMX stream)
    ESP_ERROR_CHECK(dmx_ctrl_start_on_core(1));

    // Start the Wi-Fi portal AP on Core 0 so you can see the SSID and enter creds
    // (keeps your component’s behavior; if you later long-press GPIO0 it will wipe creds & reboot)
    wifi_setup_start_on_core(0);

    // Keep the long-press monitor on Core 0
    wifi_setup_start_button_monitor_on_core(0);

    ESP_LOGI(TAG, "booted: Wi-Fi portal@C0, BTN@C0, DMX@C1; polling waits for IP");
}
