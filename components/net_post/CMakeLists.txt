idf_component_register(
    SRCS "net_post.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_http_client
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        app_cfg
        esp_timer
        esp_driver_gpio
    PRIV_REQUIRES
        mbedtls          # <-- needed for esp_crt_bundle.h
)