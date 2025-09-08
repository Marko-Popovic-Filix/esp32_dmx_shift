// components/net_post/net_post.c
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "net_post.h"

// Kconfig options (kept same names but semantics updated to GET)
#ifndef CONFIG_APP_POST_URL
#define CONFIG_APP_POST_URL "https://example-bucket.s3.amazonaws.com/latest-command.json"
#endif

#ifndef CONFIG_APP_POST_INTERVAL_SEC
#define CONFIG_APP_POST_INTERVAL_SEC 3
#endif

static const char *TAG = "net_post(get)";

// State
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_lock;  // protects url/period/callback
static char *s_url = NULL;
static unsigned s_period_sec = CONFIG_APP_POST_INTERVAL_SEC;
static net_post_cb_t s_cb = NULL;
static uint32_t s_last_crc = 0;

// Simple CRC32 (same polynomial as IDF's wear_levelling)
static uint32_t crc32_calc(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

typedef struct { char *buf; int len; } dl_ctx_t;

static esp_err_t http_evt_hdl(esp_http_client_event_t *evt) {
    dl_ctx_t *ctx = (dl_ctx_t*)evt->user_data;
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA: {
            if (evt->data_len <= 0) break;
            char *p = realloc(ctx->buf, ctx->len + evt->data_len + 1);
            if (!p) return ESP_FAIL;
            ctx->buf = p;
            memcpy(ctx->buf + ctx->len, evt->data, evt->data_len);
            ctx->len += evt->data_len;
            ctx->buf[ctx->len] = 0;
            break;
        }
        default: break;
    }
    return ESP_OK;
}

static esp_err_t s3_fetch_once(const char *url, char **out_buf, int *out_len) {
    *out_buf = NULL; *out_len = 0;
    if (!url) return ESP_ERR_INVALID_ARG;

    dl_ctx_t ctx = {0};
    esp_http_client_config_t cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_evt_hdl,
        .user_data = &ctx,
        .timeout_ms = 7000,
        .method = HTTP_METHOD_GET,
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) return ESP_FAIL;

    esp_err_t err = esp_http_client_perform(cli);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(cli);
        if (status == 200 && ctx.len > 0) {
            *out_buf = ctx.buf;
            *out_len = ctx.len;
            ctx.buf = NULL; ctx.len = 0;
        } else {
            ESP_LOGW(TAG, "HTTP %d len=%d", status, ctx.len);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "GET failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(cli);
    free(ctx.buf);
    return err;
}

static void post_task(void *arg) {
    ESP_LOGI(TAG, "poll task core=%d", xPortGetCoreID());
    while (1) {
        // Copy config under lock
        char *url_local = NULL;
        unsigned period_local = 0;
        net_post_cb_t cb_local = NULL;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        const char *u = s_url ? s_url : CONFIG_APP_POST_URL;
        url_local = strdup(u);
        period_local = s_period_sec > 0 ? s_period_sec : CONFIG_APP_POST_INTERVAL_SEC;
        cb_local = s_cb;
        xSemaphoreGive(s_lock);

        if (!url_local) {
            ESP_LOGE(TAG, "No URL set and strdup failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        char *payload = NULL; int len = 0;
        esp_err_t err = s3_fetch_once(url_local, &payload, &len);
        if (err == ESP_OK && payload && len > 0) {
            uint32_t crc = crc32_calc((const uint8_t*)payload, len);
            if (crc != s_last_crc) {
                s_last_crc = crc;
                ESP_LOGI(TAG, "New JSON (%d bytes, crc=%08x)", len, crc);
                if (cb_local) cb_local(payload, len);
            } else {
                ESP_LOGD(TAG, "Unchanged (crc=%08x)", crc);
            }
        }
        free(payload);
        free(url_local);
        vTaskDelay(pdMS_TO_TICKS(period_local * 1000));
    }
}

esp_err_t net_post_start_on_core(BaseType_t core_id) {
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreatePinnedToCore(post_task, "S3Poll", 8192, NULL, 5, &s_task, core_id);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void net_post_stop(void) {
    if (s_task) {
        TaskHandle_t t = s_task;
        s_task = NULL;
        vTaskDelete(t);
        ESP_LOGI(TAG, "poll task deleted");
    }
}

esp_err_t net_post_set_api(const char* url, const char* /*unused_api_key*/) {
    if (!url || !*url) return ESP_ERR_INVALID_ARG;
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    free(s_url);
    s_url = strdup(url);
    xSemaphoreGive(s_lock);
    return s_url ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t net_post_set_period(unsigned period_sec) {
    if (period_sec < 1) period_sec = 1;
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_period_sec = period_sec;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t net_post_set_callback(net_post_cb_t cb) {
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cb = cb;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
