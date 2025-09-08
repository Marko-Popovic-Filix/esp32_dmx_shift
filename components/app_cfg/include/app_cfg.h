#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize app configuration (compute device_id).
 */
esp_err_t app_cfg_init(void);

/**
 * Returns a stable device id for this unit: "IAQ-XXXX" (MAC last 2 bytes).
 * The pointer remains valid for the lifetime of the app.
 */
const char* app_cfg_get_device_id(void);

#ifdef __cplusplus
}
#endif
