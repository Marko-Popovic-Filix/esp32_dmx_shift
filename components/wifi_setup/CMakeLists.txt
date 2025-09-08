# components/wifi_setup/CMakeLists.txt
idf_component_register(
    SRCS "wifi_setup.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif esp_http_server nvs_flash
    PRIV_REQUIRES esp_timer esp_driver_gpio
)
