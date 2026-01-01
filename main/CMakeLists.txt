idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver mqtt esp_wifi nvs_flash esp32-camera esp_timer esp_adc )