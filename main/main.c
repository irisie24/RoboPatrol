/*
 * Robo Patrol Control Logic - Central Mode (ESP32-S3)
 * Optimized for high-performance BLE discovery and hardware safety.
 */

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
#define TRIG_PIN       GPIO_NUM_13
#define ECHO_PIN       GPIO_NUM_17
#define PIR_PIN        GPIO_NUM_14
#define MQ2_CHAN       ADC1_CHANNEL_0 

// Nordic UART Service (NUS) UUIDs
static const ble_uuid128_t NUS_SVC_UUID = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t NUS_CHR_RX_UUID = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

typedef struct 
{
    float distance;
    int motion;
    int gas;
    int camera_ok;
} rover_sensors_t;

static rover_sensors_t g_sensors;
static uint16_t g_conn_handle = 0;
static uint16_t g_rx_char_handle = 0; 

// --- Sensor Polling ---
/*
void sensor_poll_task(void *pv)
{
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ2_CHAN, ADC_ATTEN_DB_11);

    while (1) 
    {
        // Ultrasonic calculation
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

        g_sensors.motion = gpio_get_level(PIR_PIN);

        //ESP_LOGI(TAG, "PIR sensor: %d", g_sensors.motion);
        ESP_LOGI(TAG, "Distance : %1f", g_sensors.distance);

        g_sensors.gas = adc1_get_raw(MQ2_CHAN);     

        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}
*/

void sensor_poll_task(void *pv)
{
    // Hardware Setup
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ECHO_PIN, GPIO_PULLDOWN_ONLY); // Hardware atom: ensure clean 0V

    // ADC setup for MQ2 (even if disconnected, keep config stable)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ2_CHAN, ADC_ATTEN_DB_11);

    while (1) 
    {
        // 1. Clean the line
        gpio_set_level(TRIG_PIN, 0);
        esp_rom_delay_us(5);

        // 2. Send the Trigger Pulse
        gpio_set_level(TRIG_PIN, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_PIN, 0);

        // 3. Wait for the Rising Edge (Pulse Start)
        int64_t start_limit = esp_timer_get_time();
        while(gpio_get_level(ECHO_PIN) == 0) {
            if ((esp_timer_get_time() - start_limit) > 35000) {
                g_sensors.distance = -1.0; 
                goto log_step;
            }
        }
        int64_t t1 = esp_timer_get_time();

        // 4. THE NOISE GATE (Critical Fix)
        // We ignore the first 200 microseconds of the ECHO signal.
        // This bypasses the electrical crosstalk that caused your 0.8cm readings.
        esp_rom_delay_us(200); 

        // 5. Wait for the Falling Edge (Pulse End)
        while(gpio_get_level(ECHO_PIN) == 1) {
            // Maximum range for HC-SR04 is ~400cm (~23ms)
            if ((esp_timer_get_time() - t1) > 40000) {
                g_sensors.distance = -1.0;
                goto log_step;
            }
        }
        int64_t t2 = esp_timer_get_time();

        // 6. Calculate Distance (Speed of sound = 0.0343 cm/us)
        float duration = (float)(t2 - t1);
        g_sensors.distance = (duration * 0.0343) / 2.0;

log_step:
        // Read other atoms (MQ2 and PIR currently disconnected/floating)
        g_sensors.gas = adc1_get_raw(MQ2_CHAN); 
        
        // Log results with clean formatting
        if (g_sensors.distance < 0) {
            ESP_LOGW("ROVER_S3", "Dist: TIMEOUT | Gas Raw: %d", g_sensors.gas);
        } else {
            ESP_LOGI("ROVER_S3", "Dist: %.1f cm | Gas Raw: %d", g_sensors.distance, g_sensors.gas);
        }

        // Poll at 500ms for a balance of performance and stability
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- BLE Client Callback Logic ---

static void ble_app_scan(void);

// 1. Characteristic Discovery Callback
static int ble_on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) 
{
    if (error->status == 0 && chr != NULL) 
    {
        if (ble_uuid_cmp(&chr->uuid.u, &NUS_CHR_RX_UUID.u) == 0) 
        {
            g_rx_char_handle = chr->val_handle;
            ESP_LOGI(TAG, "NUS RX Characteristic Verified! Handle: %d", g_rx_char_handle);
            
            const char *init_str = "ROBO PATROL INIT";
            ble_gattc_write_flat(conn_handle, g_rx_char_handle, init_str, strlen(init_str), NULL, NULL);
        }
    }
    return 0;
}

// 2. Service Discovery Callback (The "Ascertainment" Atom)
static int ble_on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg)
{
    if (error->status == 0 && service != NULL) 
    {
        if (ble_uuid_cmp(&service->uuid.u, (ble_uuid_t *)&NUS_SVC_UUID) == 0) 
        {
            ESP_LOGI(TAG, "NUS Service Confirmed. Handles: 0x%04x - 0x%04x", service->start_handle, service->end_handle);
            ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, ble_on_disc_chr, NULL);
        }
    }
    return 0;
}

// 3. Main GAP Event Handler
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields = {0};
    switch (event->type) 
    {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            
            // Atom: Match 4-byte name "RPGW"
            if (fields.name != NULL && fields.name_len == 4)
            {
                if (memcmp(fields.name, "RPGW", 4) == 0)
                {
                    ESP_LOGI(TAG, "Gateway 'RPGW' found (RSSI: %d). Connecting...", event->disc.rssi);
                    ble_gap_disc_cancel();
                    ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
                }
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) 
            {
                g_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connection Up. Discovering NUS Service...");
                ble_gattc_disc_svc_by_uuid(g_conn_handle, (ble_uuid_t *)&NUS_SVC_UUID, ble_on_disc_svc, NULL);
            }
            else 
            {
                ble_app_scan();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            g_conn_handle = 0;
            g_rx_char_handle = 0;
            ESP_LOGW(TAG, "Disconnected. Resuming Scan...");
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

// --- Telemetry Task ---
void ble_tx_task(void *pv)
{
    char json[128];
    while (1) 
    {
        if (g_conn_handle != 0 && g_rx_char_handle != 0) 
        {
            snprintf(json, sizeof(json), "{\"d\":%.1f,\"m\":%d,\"g\":%d,\"c\":%d}", g_sensors.distance, g_sensors.motion, g_sensors.gas, g_sensors.camera_ok);
            
            ble_gattc_write_flat(g_conn_handle, g_rx_char_handle, json, strlen(json), NULL, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
}

void ble_host_task(void *param) 
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// --- Main App ---
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*
    // --- Guard Rail: Zero Memory to avoid LoadProhibited panics ---
    camera_config_t cam_cfg;
    memset(&cam_cfg, 0, sizeof(camera_config_t));

    cam_cfg.pin_pwdn = -1;
    cam_cfg.pin_reset = -1;
    cam_cfg.pin_xclk = 15;
    cam_cfg.pin_sscb_sda = 4;
    cam_cfg.pin_sscb_scl = 5;
    cam_cfg.pin_d7 = 16;
    cam_cfg.pin_d6 = 17;
    cam_cfg.pin_d5 = 18;
    cam_cfg.pin_d4 = 19;
    cam_cfg.pin_d3 = 20;
    cam_cfg.pin_d2 = 21;
    cam_cfg.pin_d1 = 22;
    cam_cfg.pin_d0 = 23;
    cam_cfg.pin_vsync = 25;
    cam_cfg.pin_href = 26;
    cam_cfg.pin_pclk = 27;
    cam_cfg.xclk_freq_hz = 20000000;
    cam_cfg.ledc_timer = LEDC_TIMER_0;
    cam_cfg.ledc_channel = LEDC_CHANNEL_0;
    cam_cfg.pixel_format = PIXFORMAT_JPEG;
    cam_cfg.frame_size = FRAMESIZE_QQVGA;
    cam_cfg.jpeg_quality = 12;
    cam_cfg.fb_count = 1;
    cam_cfg.fb_location = CAMERA_FB_IN_DRAM; 
    cam_cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    cam_cfg.sccb_i2c_port = 0;

    // --- Guard Rail: Non-fatal initialization check ---
    esp_err_t cam_err = esp_camera_init(&cam_cfg);
    if (cam_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed (0x%x). Headless mode active.", cam_err);
        g_sensors.camera_ok = 0;
    }
    else
    {
        ESP_LOGI(TAG, "Camera Init Success.");
        g_sensors.camera_ok = 1;
    }
    */

    g_sensors.camera_ok = 0; 

    // Create Sensor and Communication Tasks
    xTaskCreate(sensor_poll_task, "sensors", 4096, NULL, 5, NULL);
    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 5, NULL);

    // Initialize NimBLE
    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_scan;
    nimble_port_freertos_init(ble_host_task);
}
