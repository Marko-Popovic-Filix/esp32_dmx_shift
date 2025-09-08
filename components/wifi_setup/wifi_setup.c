#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_mac.h"      // esp_read_mac
#include "wifi_setup.h"

#define TAG "wifi_setup"

/* ========= Config ========= */
#ifndef RESET_BTN_GPIO
#define RESET_BTN_GPIO       0      // default: BOOT button on many dev boards
#endif
#define BTN_SAMPLE_MS        20
#define BTN_HOLD_MS          3000

#define NVS_NS               "wm"
#define KEY_SSID             "ssid"
#define KEY_PASS             "pass"

/* ========= AP defaults (suffix is added from MAC) ========= */
#define AP_SSID_BASE         "ESP32_Setup"
#define AP_PASS              ""
#define AP_CHAN              1
#define AP_MAXCON            4

/* ========= Module state ========= */
static httpd_handle_t s_httpd    = NULL;
static esp_netif_t   *s_ap_netif = NULL;
static esp_netif_t   *s_sta_netif= NULL;

/* ========= NVS helpers ========= */
static esp_err_t nvs_set_str2(const char *key, const char *val)
{
    nvs_handle_t h = 0;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &h), TAG, "nvs_open");
    esp_err_t err;
    if (val && val[0]) err = nvs_set_str(h, key, val);
    else               err = nvs_erase_key(h, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static esp_err_t nvs_get_str2(const char *key, char *out, size_t outlen)
{
    out[0] = 0;
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    size_t sz = outlen;
    err = nvs_get_str(h, key, out, &sz);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) { out[0] = 0; return ESP_OK; }
    return err;
}

static void nvs_wipe_and_reboot(void)
{
    nvs_handle_t h = 0;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

/* ========= Small helpers ========= */
static void url_decode(char *s)
{
    char *o = s;
    for (char *p = s; *p; ++p) {
        if (*p == '+') { *o++ = ' '; }
        else if (*p == '%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])) {
            int hi = isdigit((unsigned char)p[1]) ? p[1]-'0' : 10 + (tolower((unsigned char)p[1]) - 'a');
            int lo = isdigit((unsigned char)p[2]) ? p[2]-'0' : 10 + (tolower((unsigned char)p[2]) - 'a');
            *o++ = (char)((hi<<4)|lo);
            p += 2;
        } else *o++ = *p;
    }
    *o = 0;
}

static bool form_get(const char *form, const char *key, char *out, size_t outlen)
{
    out[0] = 0;
    size_t kl = strlen(key);
    const char *p = form;
    while (p && *p) {
        const char *eq = strchr(p, '=');
        if (!eq) break;
        const char *amp = strchr(eq, '&');
        size_t klen = (size_t)(eq - p);
        if (klen == kl && strncmp(p, key, kl) == 0) {
            size_t vlen = amp ? (size_t)(amp - (eq + 1)) : strlen(eq + 1);
            if (vlen >= outlen) vlen = outlen - 1;
            memcpy(out, eq + 1, vlen);
            out[vlen] = 0;
            url_decode(out);
            return true;
        }
        p = amp ? amp + 1 : NULL;
    }
    return false;
}

/* ========= HTTP server ========= */
static esp_err_t index_get_handler(httpd_req_t *req)
{
    static const char *page =
        "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32 Wi-Fi Setup</title></head><body>"
        "<h2>Wi-Fi Setup</h2>"
        "<form method='POST' action='/save'>"
        "SSID:<br><input name='ssid' maxlength='32'><br><br>"
        "Password:<br><input name='pass' type='password' maxlength='64'><br><br>"
        "<button type='submit'>Save & Reboot</button>"
        "</form>"
        "<p>Hold BOOT (GPIO0) for 3s to clear creds.</p>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t save_post_handler(httpd_req_t *req)
{
    char buf[1024];
    int total = 0;
    while (1) {
        int n = httpd_req_recv(req, buf + total, sizeof(buf) - 1 - total);
        if (n <= 0) break;
        total += n;
        if (total >= (int)sizeof(buf) - 1) break;
    }
    buf[total] = 0;

    char ssid[33] = {0};
    char pass[65] = {0};
    (void)form_get(buf, "ssid", ssid, sizeof(ssid));
    (void)form_get(buf, "pass", pass, sizeof(pass));

    if (ssid[0] == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_OK;
    }

    nvs_set_str2(KEY_SSID, ssid);
    nvs_set_str2(KEY_PASS, pass);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<html><body><h3>Saved. Rebooting…</h3></body></html>");

    ESP_LOGW(TAG, "Rebooting to apply credentials...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

static httpd_handle_t httpd_start_simple(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_open_sockets = 4;
    httpd_handle_t h = NULL;
    if (httpd_start(&h, &cfg) == ESP_OK) {
        httpd_uri_t u1 = {.uri="/",     .method=HTTP_GET,  .handler=index_get_handler, .user_ctx=NULL};
        httpd_uri_t u2 = {.uri="/save", .method=HTTP_POST, .handler=save_post_handler, .user_ctx=NULL};
        httpd_register_uri_handler(h, &u1);
        httpd_register_uri_handler(h, &u2);
        ESP_LOGI(TAG, "HTTPD started");
    } else {
        ESP_LOGE(TAG, "httpd_start failed");
    }
    return h;
}

/* ========= AP SSID with MAC suffix ========= */
static void make_ap_ssid(char out[33])
{
    uint8_t mac[6] = {0};
    // Use the STA MAC to derive a stable suffix; SOFTAP MAC is also OK.
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, 33, AP_SSID_BASE"-%02X%02X", mac[4], mac[5]);
}

/* ========= Wi-Fi bring-up ========= */
static void start_ap_portal(void)
{
    ESP_LOGI(TAG, "Starting AP+HTTP portal");

    if (!s_ap_netif) s_ap_netif = esp_netif_create_default_wifi_ap();

    char ssid_dyn[33];
    make_ap_ssid(ssid_dyn);
    ESP_LOGI(TAG, "AP SSID: %s", ssid_dyn);

    wifi_config_t ap = { 0 };
    strlcpy((char*)ap.ap.ssid, ssid_dyn, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(ssid_dyn);
    ap.ap.channel = AP_CHAN;
    ap.ap.max_connection = AP_MAXCON;
    ap.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (!s_httpd) s_httpd = httpd_start_simple();
}

static void connect_sta(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG, "Connecting STA to '%s'%s", ssid, (pass && pass[0]) ? "" : " (open)");

    if (!s_sta_netif) s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_config_t sta = { 0 };
    strlcpy((char*)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    if (pass && pass[0]) strlcpy((char*)sta.sta.password, pass, sizeof(sta.sta.password));
    sta.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

/* Stop AP/HTTP once STA has IP */
static void evt_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP on STA");
        if (s_httpd) { httpd_stop(s_httpd); s_httpd = NULL; }
        if (s_ap_netif) {
            ESP_LOGI(TAG, "Switching to STA-only mode");
            esp_wifi_set_mode(WIFI_MODE_STA);
        }
    }
}

/* ========= Button task (long-press to clear) ========= */
static void btn_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RESET_BTN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,     // BOOT is usually pulled up, press to GND
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    ESP_LOGI(TAG, "Reset button on GPIO%d (hold >= %d ms to clear Wi-Fi)", RESET_BTN_GPIO, BTN_HOLD_MS);

    bool prev = gpio_get_level(RESET_BTN_GPIO);
    int64_t t_down_ms = 0;

    for (;;) {
        bool level = gpio_get_level(RESET_BTN_GPIO); // 1 = released, 0 = pressed
        if (!level) {
            if (prev) {
                t_down_ms = esp_timer_get_time() / 1000;
                ESP_LOGD(TAG, "Button pressed (level=0)");
            } else {
                int64_t held = (esp_timer_get_time() / 1000) - t_down_ms;
                if (held >= BTN_HOLD_MS) {
                    ESP_LOGW(TAG, "Long-press detected → wipe creds + reboot");
                    nvs_wipe_and_reboot();
                }
            }
            prev = false;
        } else {
            if (!prev) ESP_LOGD(TAG, "Button released (level=1)");
            prev = true;
        }
        vTaskDelay(pdMS_TO_TICKS(BTN_SAMPLE_MS));
    }
}

/* ========= Public API ========= */
static void wifi_setup_task(void *arg)
{
   {
    esp_err_t e = esp_netif_init();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) { ESP_ERROR_CHECK(e); }
}
{
    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) { ESP_ERROR_CHECK(e); }
}


    wifi_init_config_t wicfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wicfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Events (to stop AP+HTTP when STA connects)
    esp_event_handler_instance_t h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, evt_handler, NULL, &h_ip));

    // Try stored creds -> STA; else portal
    char ssid[33] = {0};
    char pass[65] = {0};
    (void)nvs_get_str2(KEY_SSID, ssid, sizeof(ssid));
    (void)nvs_get_str2(KEY_PASS, pass, sizeof(pass));

    if (ssid[0]) connect_sta(ssid, pass);
    else         start_ap_portal();

    vTaskDelete(NULL);
}

esp_err_t wifi_setup_start(void)
{
    xTaskCreate(wifi_setup_task, "wifi_setup", 4096, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t wifi_setup_start_on_core(BaseType_t core_id)
{
    xTaskCreatePinnedToCore(wifi_setup_task, "wifi_setup", 4096, NULL, 5, NULL, core_id);
    return ESP_OK;
}

void wifi_setup_start_button_monitor_on_core(BaseType_t core_id)
{
    xTaskCreatePinnedToCore(btn_task, "wm_btn", 2048, NULL, 10, NULL, core_id);
}
