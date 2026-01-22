/* Laser Beam Acquisition System
 *
 * Main application - EtherCAT motor control with power sensor feedback
 * 
 * EtherCAT (via SOEM) handles Ethernet initialization directly.
 * No TCP/IP stack - pure Layer 2 EtherCAT communication.
 */
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Application modules
#include "motor_control.h"
#include "power_sensor.h"
#include "acquire.h"

static const char *TAG = PROJECT_NAME;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Laser Beam Acquisition System");
    
    // Initialize NVS for configuration storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network stack and default event loop for Ethernet
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize I2C power sensor
    ESP_LOGI(TAG, "Initializing power sensor...");
    ret = power_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power sensor initialization failed: %s", esp_err_to_name(ret));
        // Continue anyway - sensor may not be present
    }
    
    // Initialize motor control (SOEM/EtherCAT)
    // Note: SOEM handles Ethernet hardware initialization internally
    ESP_LOGI(TAG, "Initializing motor control (SOEM/EtherCAT)...");
    ret = motor_control_init("eth0");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor control initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check EtherCAT cable and slave devices");
        // Continue to allow debugging
    }
    
    // Initialize acquisition system
    ESP_LOGI(TAG, "Initializing acquisition system...");
    acquire_init();
    
    ESP_LOGI(TAG, "System initialized");
    ESP_LOGI(TAG, "Starting acquisition state machine...");
    
    // Main control loop
    while (1) {
        // Process motor control (cyclic EtherCAT communication)
        motor_process();
        
        // Run acquisition state machine
        acquire_update();
        
        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

