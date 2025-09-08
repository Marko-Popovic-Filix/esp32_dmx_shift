idf_component_register(
    SRCS "main.c"
    REQUIRES
        wifi_setup
        net_post
        dmx_ctrl
        app_cfg
        utils
        driver
        esp_netif
        esp_event
        esp_wifi
        esp_http_server
)