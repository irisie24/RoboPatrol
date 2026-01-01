/* * ROBO PATROL - Integrated Control & Cloud Logic (ESP32-S3)
 * Style: Allman (Broken Braces)
 * Features: UART Hub (Arduino), L298N Drivetrain, WiFi, MQTT Client, Camera Stubs
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_camera.h"

static const char *TAG = "ROBO_PATROL";

// --- Configuration: WiFi & MQTT ---
#define STATUS_LED_GPIO    2
static const char *WIFI_SSID       = "RPGWSSID";
static const char *WIFI_PSK        = "Robo@2026";
static const char *MQTT_URL        = "mqtt://broker.emqx.io:1883";
static const char *MQTT_TOPIC_PUB  = "robopatrol/telemetry";
static const char *MQTT_TOPIC_SUB  = "robopatrol/commands";

// --- Configuration: Hardware Pins ---
#define MOTOR_A_IN1    GPIO_NUM_4
#define MOTOR_A_IN2    GPIO_NUM_5
#define MOTOR_B_IN3    GPIO_NUM_6
#define MOTOR_B_IN4    GPIO_NUM_7

#define UART_PORT      UART_NUM_1
#define TXD_PIN        GPIO_NUM_17 
#define RXD_PIN        GPIO_NUM_18 
#define BUF_SIZE       1024

// --- Global State ---
typedef struct 
{
    float distance;
    int gas;
    int camera_ok;
} rover_sensors_t;

static rover_sensors_t g_sensors = {0};
static esp_mqtt_client_handle_t mqtt_client;
static bool wifi_connected = false;

// --- Motor Control Logic ---
void drive_motors(int a1, int a2, int b3, int b4)
{
    gpio_set_level(MOTOR_A_IN1, a1);
    gpio_set_level(MOTOR_A_IN2, a2);
    gpio_set_level(MOTOR_B_IN3, b3);
    gpio_set_level(MOTOR_B_IN4, b4);
}

// --- MQTT Event Handler (Commands from Cloud) ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED)
    {
        esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_SUB, 0);
        ESP_LOGI(TAG, "MQTT Connected. Subscribed to commands.");
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_PUB, "RoboPatrol Startup", 0, 1, 0);
    }
    else if (event_id == MQTT_EVENT_DATA)
    {
        if (strncmp(event->data, "FORWARD", event->data_len) == 0) drive_motors(1, 0, 1, 0);
        else if (strncmp(event->data, "STOP", event->data_len) == 0) drive_motors(0, 0, 0, 0);
    }
}

// --- WiFi Event Handler ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) 
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        wifi_connected = false;
        esp_wifi_connect();
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        wifi_connected = true;
        if (mqtt_client) esp_mqtt_client_start(mqtt_client);
    }
}

// --- Tasks ---

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
            
            if (sscanf((char*)data, "%f,%d", &d_val, &g_val) == 2)
            {
                g_sensors.distance = d_val;
                g_sensors.gas = g_val;

                // Fail-safe: Local obstacle avoidance overrides cloud
                if (g_sensors.distance > 0 && g_sensors.distance < 25.0)
                {
                    ESP_LOGW(TAG, "Local Obstacle! Overriding...");
                    drive_motors(0, 1, 1, 0); 
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void mqtt_publisher_task(void *pvParameters) 
{
    char json[128];
    while (1) 
    {
        if (wifi_connected && mqtt_client) 
        {
            snprintf(json, sizeof(json), "{\"dist\":%.1f,\"gas\":%d,\"cam\":%d}", 
                     g_sensors.distance, g_sensors.gas, g_sensors.camera_ok);
            
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_PUB, json, 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void status_led_task(void *pvParameters) 
{
    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
    while (1) 
    {
        if (!wifi_connected) 
        {
            gpio_set_level(STATUS_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(100));
        } 
        else 
        {
            gpio_set_level(STATUS_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(STATUS_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// --- Initialization Helpers ---

void hardware_init()
{
    // Motors
    gpio_config_t motor_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) | 
                        (1ULL << MOTOR_B_IN3) | (1ULL << MOTOR_B_IN4)
    };
    gpio_config(&motor_conf);

    // UART for Arduino
    const uart_config_t uart_config = {
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

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

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

    g_sensors.camera_ok = 0; // Default until camera is enabled

    hardware_init();

    // WiFi Setup
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { .sta = { .ssid = "RPGWSSID", .password = "Robo@2026" } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_set_hostname(netif, "RoboPatrol");
    esp_wifi_start();

    // MQTT Setup
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URL,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // Start Tasks
    xTaskCreate(status_led_task, "led_task", 2048, NULL, 1, NULL);
    xTaskCreate(arduino_listener_task, "uart_sync", 4096, NULL, 10, NULL);
    xTaskCreate(mqtt_publisher_task, "mqtt_pub", 4096, NULL, 5, NULL);
}
