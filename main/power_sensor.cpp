/**
 * @file power_sensor.cpp
 * @brief I2C power sensor driver implementation
 * 
 * Reads laser power from sensor at I2C address 0x51.
 * Registers 0x68 and 0x69 contain the 16-bit power reading.
 */

#include "power_sensor.h"
#include "acquire_config.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "power_sensor";

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   100

static bool s_initialized = false;

/** Scan the I2C bus and log any responding addresses */
static void i2c_scan_bus(void)
{
    const uint8_t first = 0x03;
    const uint8_t last = 0x77;
    char found[8 * 16] = {0};
    size_t pos = 0;

    for (uint8_t addr = first; addr <= last; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            int written = snprintf(&found[pos], sizeof(found) - pos, "0x%02X ", addr);
            if (written > 0 && (pos + written) < sizeof(found)) {
                pos += written;
            }
        }
    }

    if (pos == 0) {
        ESP_LOGW(TAG, "I2C scan: no devices responded on SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
    } else {
        ESP_LOGW(TAG, "I2C scan: found devices at %s", found);
    }
}

extern "C" {

esp_err_t power_sensor_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    // Try a few standard I2C bus speeds to accommodate different wiring/load
    const int freq_candidates[] = { I2C_FREQ_HZ, 400000, 50000 };

    for (size_t i = 0; i < sizeof(freq_candidates) / sizeof(freq_candidates[0]); ++i) {
        int freq = freq_candidates[i];
        if (i > 0 && freq == freq_candidates[i - 1]) {
            continue;  // Skip duplicates
        }

        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = static_cast<gpio_num_t>(I2C_SDA_GPIO);
        conf.scl_io_num = static_cast<gpio_num_t>(I2C_SCL_GPIO);
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = freq;

        esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C cfg failed (freq=%d): %s", freq, esp_err_to_name(err));
            continue;
        }

        err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C driver install failed (freq=%d): %s", freq, esp_err_to_name(err));
            continue;
        }

        // Verify sensor presence at this speed
        if (power_sensor_is_connected()) {
            s_initialized = true;
            ESP_LOGI(TAG, "Power sensor initialized (addr=0x%02X, SDA=%d, SCL=%d, %d Hz)",
                     POWER_SENSOR_I2C_ADDR, I2C_SDA_GPIO, I2C_SCL_GPIO, freq);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Sensor not detected at 0x%02X on freq=%d; scanning bus...", POWER_SENSOR_I2C_ADDR, freq);
        i2c_scan_bus();
        i2c_driver_delete(I2C_MASTER_NUM);
    }

    ESP_LOGE(TAG, "Power sensor not detected after trying multiple I2C speeds");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t power_sensor_read(float *power_uw)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (power_uw == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2] = {0};
    
    // Read from register 0x68 (first memory register)
    // The sensor likely has a 16-bit value split across two consecutive registers
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (POWER_SENSOR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, POWER_SENSOR_REG1, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (POWER_SENSOR_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor: %s", esp_err_to_name(err));
        *power_uw = 0.0f;
        return err;
    }

    // Combine bytes into 16-bit value
    // Assuming MSB first (big-endian), adjust if sensor uses different format
    uint16_t raw_value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    
    // Convert to microwatts
    // TODO: Apply calibration factor if needed based on sensor datasheet
    *power_uw = static_cast<float>(raw_value);
    
    ESP_LOGD(TAG, "Power: %.2f µW (raw: 0x%04X)", *power_uw, raw_value);
    
    return ESP_OK;
}

bool power_sensor_is_connected(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (POWER_SENSOR_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return (err == ESP_OK);
}

void power_sensor_deinit(void)
{
    if (s_initialized) {
        i2c_driver_delete(I2C_MASTER_NUM);
        s_initialized = false;
        ESP_LOGI(TAG, "Power sensor de-initialized");
    }
}

} // extern "C"
