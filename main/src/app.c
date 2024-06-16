// File Server
#include "app_internal.h"

// Freertos
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// ESP LOG
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "nvs_flash.h"

// Wifi
#include "wifi.h"
#include "tcp_server.h"
#include "socket_tasks.h"
// Oled
// GPIO
#include "gpio.h"
// CAN
#include "can.h"
// UART
#include "uart.h"
//
#include "EKF_exe.h"
//
#include "GPSPath.h"
// Global variable
System_DataTypedef *system_DATA = NULL;

// static void print_Screen(void *pvParameters)
// {
//     for(;;)
//     {
//         if (system_is_changed(system_DATA))
//         {
//             // system info screen
//             if (!system_is_debug_mode(system_DATA))
//             {
//                 oled_system_info(system_DATA->name, system_DATA->ssid, system_DATA->ip, system_DATA->mcu1_state, system_DATA->mcu2_state);
//                 system_set_sys_state(system_DATA, MODE_NOCHANGE | MODE_BOOT);
//             }
//             // debug UART and CAN screen
//             else
//             {
//                 oled_debug(system_DATA->CAN1_Data, system_DATA->uart_buffer);
//                 memset(system_DATA->CAN1_Data, 0, sizeof(uint8_t) * 8);
//                 memset(system_DATA->uart_buffer, 0, sizeof(uint8_t) * UART_RX_BUFFER_SIZE);
//                 system_set_sys_state(system_DATA, MODE_NOCHANGE | MODE_DEBUG);
//             }
//         }
//         vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
// }
typedef enum
{
    START = 0,
    WIFI_CONNECTED,
    TCP_SERVER_CREATED,
    WEB_SERVER_CREATED
} running_state_t;
void app_main(void)
{
    system_DATA = system_init();
    /* Init and configure GPIO pin used in system*/
    gpio_init(system_DATA);
    /* Uart init*/
    uart_init(system_DATA);
    /*Init and start CAN bus*/
    can_init();
    EKF_ExeInit();
    GPSPathInit();
     static running_state_t running_state = START; // Set the desired running status here
    // Initialize NVS (Non-Volatile Storage)
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);

    // Initialize Wi-Fi
    error = wifi_init();

    if (error == ESP_OK)
    {
        running_state = WIFI_CONNECTED;
    }
    else
    {
        ESP_LOGE("Main", "Wifi is initialized fail!");
    }

    // Start the first socket tasks and TCP server
    error = socket_tasks_init();

    if (error == ESP_OK)
    {
        ESP_LOGI("Main", "TCP server started!");
    }
    else
    {
        ESP_LOGE("Main", "TCP server is fail!");
    }

    while (running_state == TCP_SERVER_CREATED)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}