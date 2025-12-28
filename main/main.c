#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_camera.h"

/* NimBLE Stack */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#define TAG "ROVER_S3"

// --- Hardware Pins ---
#define TRIG_PIN       GPIO_NUM_12
#define ECHO_PIN       GPIO_NUM_13
#define PIR_PIN        GPIO_NUM_14
#define MQ2_CHAN       ADC1_CHANNEL_0 

// Nordic UART Service UUIDs
static const ble_uuid128_t NUS_SVC_UUID = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

typedef struct 
{
    float distance;
    int motion;
    int gas;
    int camera_ok;
} rover_sensors_t;

static rover_sensors_t g_sensors;
static uint16_t g_conn_handle = 0;

// --- Sensor Polling ---
void sensor_poll_task(void *pv)
{
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ2_CHAN, ADC_ATTEN_DB_11);

    while (1) 
    {
        // 1. Ultrasonic
        gpio_set_level(TRIG_PIN, 0);
        esp_rom_delay_us(2);
        gpio_set_level(TRIG_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);
        
        int64_t t1 = esp_timer_get_time();
        while(gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - t1) < 20000);
        int64_t t2 = esp_timer_get_time();
        while(gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - t2) < 20000);
        g_sensors.distance = (float)(esp_timer_get_time() - t2) * 0.034 / 2;

        // 2. PIR & Gas
        g_sensors.motion = gpio_get_level(PIR_PIN);
        g_sensors.gas = adc1_get_raw(MQ2_CHAN);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- BLE Client Logic ---
static void ble_app_scan(void);

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    switch (event->type) 
    {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            for (int i = 0; i < fields.num_uuids128; i++) 
            {
                if (ble_uuid_cmp(&fields.uuids128[i].u, (ble_uuid_t *)&NUS_SVC_UUID) == 0) 
                {
                    ble_gap_disc_cancel();
                    ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
                }
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) 
            {
                g_conn_handle = event->connect.conn_handle;
            }
            else 
            {
                ble_app_scan();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            g_conn_handle = 0;
            ble_app_scan();
            break;
    }
    return 0;
}

static void ble_app_scan(void)
{
    struct ble_gap_disc_params dp = { .filter_duplicates = 1 };
    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &dp, ble_gap_event, NULL);
}

void ble_tx_task(void *pv)
{
    char json[128];
    while (1) 
    {
        if (g_conn_handle != 0) 
        {
            snprintf(json, sizeof(json), "{\"d\":%.1f,\"m\":%d,\"g\":%d,\"c\":%d}", 
                     g_sensors.distance, g_sensors.motion, g_sensors.gas, g_sensors.camera_ok);
            // Handle 0x0002 is typical for NUS RX on the gateway side
            ble_gattc_write_flat(g_conn_handle, 0x0002, json, strlen(json), NULL, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// FIX: Standard C function for NimBLE host task
void ble_host_task(void *param) 
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    nvs_flash_init();
    
    camera_config_t cam_cfg = {
        .pin_pwdn = -1, .pin_reset = -1, .pin_xclk = 15, .pin_sscb_sda = 4, .pin_sscb_scl = 5,
        .pin_d7 = 16, .pin_d6 = 17, .pin_d5 = 18, .pin_d4 = 19, .pin_d3 = 20, .pin_d2 = 21, .pin_d1 = 22, .pin_d0 = 23,
        .pin_vsync = 25, .pin_href = 26, .pin_pclk = 27, .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA, .jpeg_quality = 12, .fb_count = 1
    };
    g_sensors.camera_ok = (esp_camera_init(&cam_cfg) == ESP_OK);

    xTaskCreate(sensor_poll_task, "sensors", 4096, NULL, 5, NULL);
    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 5, NULL);

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_scan;
    
    // FIX: Passing the named function instead of a lambda
    nimble_port_freertos_init(ble_host_task);
}
