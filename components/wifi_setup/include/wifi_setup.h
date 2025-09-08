// components/wifi_setup/include/wifi_setup.h
#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t wifi_setup_start(void);
esp_err_t wifi_setup_start_on_core(BaseType_t core_id);
void wifi_setup_start_button_monitor_on_core(BaseType_t core_id);
