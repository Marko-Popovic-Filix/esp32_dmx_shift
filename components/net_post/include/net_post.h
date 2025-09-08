#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Type of callback invoked when a fresh JSON payload is fetched.
 *  The buffer is valid only for the duration of the callback.
 */
typedef void (*net_post_cb_t)(const char *json, int len);

/** Start the S3 GET polling task pinned to the requested core. */
esp_err_t net_post_start_on_core(BaseType_t core_id);

/** Stop the polling task (if running). */
void net_post_stop(void);

/** Set/override the S3 URL at runtime (makes an owned copy). */
esp_err_t net_post_set_api(const char* url, const char* unused_api_key /* kept for ABI compatibility, ignored */);

/** Set/override polling period (seconds), min 1s. */
esp_err_t net_post_set_period(unsigned period_sec);

/** Register a callback to receive NEW payloads (deduped by CRC). */
esp_err_t net_post_set_callback(net_post_cb_t cb);

#ifdef __cplusplus
}
#endif
