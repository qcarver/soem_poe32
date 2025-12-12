#ifndef POWER_SENSOR_H
#define POWER_SENSOR_H

/**
 * @file power_sensor.h
 * @brief I2C power sensor driver for laser acquisition system
 * 
 * Interfaces with power sensor at I2C address 0x51, registers 0x68/0x69
 * Used to measure received laser power in µW for beam tracking.
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the I2C power sensor
 * 
 * Configures I2C master on SDA=GPIO2, SCL=GPIO15 and verifies sensor presence.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_sensor_init(void);

/**
 * @brief Read current power level from sensor
 * 
 * @param[out] power_uw Pointer to store power reading in microwatts
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_sensor_read(float *power_uw);

/**
 * @brief Check if sensor is responding
 * 
 * @return true if sensor responds to I2C ping
 */
bool power_sensor_is_connected(void);

/**
 * @brief De-initialize the power sensor and release I2C bus
 */
void power_sensor_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_SENSOR_H
