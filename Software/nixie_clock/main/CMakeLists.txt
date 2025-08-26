idf_component_register(SRCS "ntp_sync.c" "wifi_manager.c" "web_server.c" "main.c" "nixie_ctr.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server json nvs_flash driver esp_adc esp_timer esp_wifi esp_netif
                    EMBED_FILES "index.html")
