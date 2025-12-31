/* * TESSERA: ROBO PATROL - Master Control Logic (ESP32-S3)
 * Style: Allman (Broken Braces)
 * Features: UART Telemetry Hub, L298N Drivetrain, Commented Camera Setup
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_camera.h"

/* NimBLE Stack */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#define TAG "ROVER_S3"

// --- Hardware Atoms: L298N Drivetrain ---
#define MOTOR_A_IN1    GPIO_NUM_4
#define MOTOR_A_IN2    GPIO_NUM_5
#define MOTOR_B_IN3    GPIO_NUM_6
#define MOTOR_B_IN4    GPIO_NUM_7

// --- Hardware Atoms: UART Bridge (Arduino Hub) ---
#define UART_PORT      UART_NUM_1
#define TXD_PIN        GPIO_NUM_17 
#define RXD_PIN        GPIO_NUM_18 
#define BUF_SIZE       1024

typedef struct 
{
    float distance;
    int gas;
    int motion;
    int camera_ok;
} rover_sensors_t;

static rover_sensors_t g_sensors = {0};
static uint16_t g_conn_handle = 0;
static uint16_t g_rx_char_handle = 0;

// Nordic UART Service (NUS) UUIDs
static const ble_uuid128_t NUS_SVC_UUID = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t NUS_CHR_RX_UUID = 
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

// --- Initialization ---

void motor_init()
{
    gpio_config_t io_conf = 
    {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) | 
                        (1ULL << MOTOR_B_IN3) | (1ULL << MOTOR_B_IN4)
    };
    gpio_config(&io_conf);
}

void uart_hub_init()
{
    const uart_config_t uart_config = 
    {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// --- Navigation Logic ---

void drive_motors(int a1, int a2, int b3, int b4)
{
    gpio_set_level(MOTOR_A_IN1, a1);
    gpio_set_level(MOTOR_A_IN2, a2);
    gpio_set_level(MOTOR_B_IN3, b3);
    gpio_set_level(MOTOR_B_IN4, b4);
}

void arduino_listener_task(void *pv)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    while (1)
    {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            data[len] = '\0';
            float d_val; 
            int g_val;
            
            // Expected format from Arduino: "150.5,400\n"
            if (sscanf((char*)data, "%f,%d", &d_val, &g_val) == 2)
            {
                g_sensors.distance = d_val;
                g_sensors.gas = g_val;

                // Threshold-based avoidance
                if (g_sensors.distance > 0 && g_sensors.distance < 25.0)
                {
                    ESP_LOGW(TAG, "Obstacle Detected! Pivoting...");
                    drive_motors(0, 1, 1, 0); 
                }
                else
                {
                    drive_motors(1, 0, 1, 0); 
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- BLE Callbacks ---

static void ble_app_scan(void);

static int ble_on_disc_chr(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 && chr != NULL)
    {
        if (ble_uuid_cmp(&chr->uuid.u, &NUS_CHR_RX_UUID.u) == 0)
        {
            g_rx_char_handle = chr->val_handle;
            ESP_LOGI(TAG, "NUS RX Handle: %d", g_rx_char_handle);
        }
    }
    return 0;
}

static int ble_on_disc_svc(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg)
{
    if (error->status == 0 && service != NULL)
    {
        if (ble_uuid_cmp(&service->uuid.u, (ble_uuid_t *)&NUS_SVC_UUID) == 0)
        {
            ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, ble_on_disc_chr, NULL);
        }
    }
    return 0;
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields = {0};
    switch (event->type)
    {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (fields.name != NULL && fields.name_len == 4 && memcmp(fields.name, "RPGW", 4) == 0)
            {
                ble_gap_disc_cancel();
                ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, NULL, ble_gap_event, NULL);
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0)
            {
                g_conn_handle = event->connect.conn_handle;
                ble_gattc_disc_svc_by_uuid(g_conn_handle, (ble_uuid_t *)&NUS_SVC_UUID, ble_on_disc_svc, NULL);
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
        if (g_conn_handle != 0 && g_rx_char_handle != 0)
        {
            snprintf(json, sizeof(json), "{\"d\":%.1f,\"g\":%d,\"c\":%d}", 
                     g_sensors.distance, g_sensors.gas, g_sensors.camera_ok);
            ble_gattc_write_flat(g_conn_handle, g_rx_char_handle, json, strlen(json), NULL, NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
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
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* // --- Future Atom: Camera Handling ---
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
    
    esp_err_t cam_err = esp_camera_init(&cam_cfg);
    if (cam_err != ESP_OK) { g_sensors.camera_ok = 0; }
    else { g_sensors.camera_ok = 1; }
    */

    g_sensors.camera_ok = 0;

    motor_init();
    uart_hub_init();

    xTaskCreate(arduino_listener_task, "uart_sync", 4096, NULL, 10, NULL);
    xTaskCreate(ble_tx_task, "ble_tx", 4096, NULL, 5, NULL);

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_scan;
    nimble_port_freertos_init(ble_host_task);
}
