#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t dmx_ctrl_start_on_core(BaseType_t core_id);
void dmx_ctrl_stop(void);
esp_err_t dmx_ctrl_apply_json(const char *json, int len);

#ifdef __cplusplus
}
#endif
