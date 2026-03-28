idf_component_register(
    SRCS "main.c" "bno055.c" "wifi_config.c" "ota.c" "discovery.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash esp_wifi esp_netif esp_http_server esp_app_format app_update
)

# Module type for discovery mDNS advertisement
target_compile_definitions(${COMPONENT_LIB} PRIVATE MODULE_TYPE="plateau")
