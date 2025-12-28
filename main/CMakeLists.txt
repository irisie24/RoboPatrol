idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt driver nvs_flash esp32-camera esp_timer esp_adc )