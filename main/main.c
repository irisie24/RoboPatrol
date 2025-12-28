#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "esp_camera.h"
#include "driver/adc.h"

#define TAG "ROVER"

// Sensor Pins
#define TRIG_PIN GPIO_NUM_12
#define ECHO_PIN GPIO_NUM_13
#define PIR_PIN GPIO_NUM_14
#define MQ2_PIN ADC1_CHANNEL_0  // GPIO36

// BLE UUIDs (Nordic UART Service)
#define NORDIC_UART_SERVICE_UUID 0x6E400001
#define NORDIC_TX_CHAR_UUID 0x6E400003
#define NORDIC_RX_CHAR_UUID 0x6E400002

// Sensor data structure
typedef struct
{
    float distance_cm;
    uint8_t motion_detected;
    uint16_t gas_level;
    uint8_t camera_status;
} sensor_data_t;

static uint16_t conn_id = 0;
static uint16_t char_handle = 0;
static bool connected = false;

// Ultrasonic sensor task
void ultrasonic_task(void *param)
{
    sensor_data_t *data = (sensor_data_t *)param;
    
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    
    while(1)
    {
        gpio_set_level(TRIG_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(2));
        gpio_set_level(TRIG_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(TRIG_PIN, 0);
        
        int64_t start = esp_timer_get_time();
        while(gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - start) < 30000);
        start = esp_timer_get_time();
        while(gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - start) < 30000);
        int64_t duration = esp_timer_get_time() - start;
        
        data->distance_cm = duration * 0.034 / 2.0;
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// PIR sensor task
void pir_task(void *param)
{
    sensor_data_t *data = (sensor_data_t *)param;
    
    gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);
    
    while(1)
    {
        data->motion_detected = gpio_get_level(PIR_PIN);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// MQ2 gas sensor task
void mq2_task(void *param)
{
    sensor_data_t *data = (sensor_data_t *)param;
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ2_PIN, ADC_ATTEN_DB_11);
    
    while(1)
    {
        data->gas_level = adc1_get_raw(MQ2_PIN);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Camera initialization
void camera_init()
{
    camera_config_t config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 15,
        .pin_sscb_sda = 4,
        .pin_sscb_scl = 5,
        .pin_d7 = 16,
        .pin_d6 = 17,
        .pin_d5 = 18,
        .pin_d4 = 19,
        .pin_d3 = 20,
        .pin_d2 = 21,
        .pin_d1 = 22,
        .pin_d0 = 23,
        .pin_vsync = 25,
        .pin_href = 26,
        .pin_pclk = 27,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 1
    };
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed");
    }
}

// BLE send data task
void ble_send_task(void *param)
{
    sensor_data_t *data = (sensor_data_t *)param;
    char json_buffer[256];
    
    while(1)
    {
        if(connected)
        {
            snprintf(json_buffer, sizeof(json_buffer),
                "{\"distance\":%.2f,\"motion\":%d,\"gas\":%d,\"camera\":%d}",
                data->distance_cm, data->motion_detected, data->gas_level, data->camera_status);
            
            esp_ble_gatts_send_indicate(0, conn_id, char_handle, 
                strlen(json_buffer), (uint8_t*)json_buffer, false);
            
            ESP_LOGI(TAG, "Sent: %s", json_buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// BLE event handlers
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    // Handle GAP events
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event)
    {
        case ESP_GATTS_CONNECT_EVT:
            conn_id = param->connect.conn_id;
            connected = true;
            ESP_LOGI(TAG, "BLE Connected");
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            connected = false;
            ESP_LOGI(TAG, "BLE Disconnected");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void ble_init()
{
    esp_err_t ret;
    
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    
    esp_ble_gap_set_device_name("ROBO_PATROL");
    esp_ble_gap_start_advertising(&adv_params);
}

void app_main(void)
{
    static sensor_data_t sensor_data = {0};
    
    ESP_LOGI(TAG, "Rover Control Starting");
    
    camera_init();
    sensor_data.camera_status = 1;
    
    ble_init();
    
    xTaskCreate(ultrasonic_task, "ultrasonic", 4096, &sensor_data, 5, NULL);
    xTaskCreate(pir_task, "pir", 2048, &sensor_data, 5, NULL);
    xTaskCreate(mq2_task, "mq2", 2048, &sensor_data, 5, NULL);
    xTaskCreate(ble_send_task, "ble_send", 4096, &sensor_data, 5, NULL);
    
    ESP_LOGI(TAG, "All tasks created");
}
