#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @file motor_control.h
 * @brief Motor control interface for Xeryon drives via SOEM (EtherCAT)
 * 
 * Provides abstraction layer for controlling pan (yaw) and tilt (pitch) axes.
 * Drive1 = Tilt axis, Drive2 = Pan axis
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Drive/axis identifier
 */
typedef enum {
    DRIVE_TILT = 0,  /**< Tilt (pitch) axis - Drive1 */
    DRIVE_PAN  = 1,  /**< Pan (yaw) axis - Drive2 */
    DRIVE_COUNT
} drive_id_t;

/**
 * @brief Drive status
 */
typedef enum {
    DRIVE_STATUS_IDLE,      /**< Drive is idle, ready for commands */
    DRIVE_STATUS_BUSY,      /**< Drive is executing a motion */
    DRIVE_STATUS_ERROR,     /**< Drive has an error condition */
    DRIVE_STATUS_DISABLED   /**< Drive is disabled */
} drive_status_t;

/**
 * @brief Motion parameters for a drive axis
 */
typedef struct {
    int32_t speed;    /**< Motion speed */
    uint16_t acc;     /**< Acceleration */
    uint16_t dec;     /**< Deceleration */
} axis_params_t;

/**
 * @brief Initialize motor control subsystem
 * 
 * Initializes SOEM and discovers EtherCAT slaves.
 * 
 * @param eth_interface Ethernet interface name (e.g., "eth0")
 * @return ESP_OK on success
 */
esp_err_t motor_control_init(const char *eth_interface);

/**
 * @brief Enable a drive
 * 
 * @param drive Drive identifier
 * @return ESP_OK on success
 */
esp_err_t motor_enable(drive_id_t drive);

/**
 * @brief Enable both drives
 * 
 * @return ESP_OK on success
 */
esp_err_t motor_enable_all(void);

/**
 * @brief Disable a drive
 * 
 * @param drive Drive identifier
 * @return ESP_OK on success
 */
esp_err_t motor_disable(drive_id_t drive);

/**
 * @brief Index (home) a drive
 * 
 * @param drive Drive identifier
 * @param params Motion parameters for homing
 * @return ESP_OK on success
 */
esp_err_t motor_index(drive_id_t drive, const axis_params_t *params);

/**
 * @brief Index (home) both drives
 * 
 * @param tilt_params Motion parameters for tilt axis
 * @param pan_params Motion parameters for pan axis
 * @return ESP_OK on success
 */
esp_err_t motor_index_all(const axis_params_t *tilt_params, const axis_params_t *pan_params);

/**
 * @brief Move drive to absolute position in encoder counts
 * 
 * @param drive Drive identifier
 * @param position Target position in encoder counts
 * @param params Motion parameters
 * @return ESP_OK on success
 */
esp_err_t motor_move_absolute_enc(drive_id_t drive, int32_t position, const axis_params_t *params);

/**
 * @brief Move drive to absolute position in degrees
 * 
 * @param drive Drive identifier
 * @param degrees Target position in degrees
 * @param params Motion parameters
 * @return ESP_OK on success
 */
esp_err_t motor_move_absolute_deg(drive_id_t drive, double degrees, const axis_params_t *params);

/**
 * @brief Halt drive motion immediately
 * 
 * @param drive Drive identifier
 * @return ESP_OK on success
 */
esp_err_t motor_halt(drive_id_t drive);

/**
 * @brief Halt all drives immediately
 * 
 * @return ESP_OK on success
 */
esp_err_t motor_halt_all(void);

/**
 * @brief Stop drive with controlled deceleration
 * 
 * @param drive Drive identifier
 * @return ESP_OK on success
 */
esp_err_t motor_stop(drive_id_t drive);

/**
 * @brief Reset drive error condition
 * 
 * @param drive Drive identifier
 * @return ESP_OK on success
 */
esp_err_t motor_reset(drive_id_t drive);

/**
 * @brief Reset encoder position to zero
 * 
 * @param drive Drive identifier
 * @return ESP_OK on success
 */
esp_err_t motor_reset_encoder(drive_id_t drive);

/**
 * @brief Get current position in encoder counts
 * 
 * @param drive Drive identifier
 * @return Current position in encoder counts
 */
int32_t motor_get_position_enc(drive_id_t drive);

/**
 * @brief Get current position in degrees
 * 
 * @param drive Drive identifier
 * @return Current position in degrees
 */
double motor_get_position_deg(drive_id_t drive);

/**
 * @brief Get drive status
 * 
 * @param drive Drive identifier
 * @return Current drive status
 */
drive_status_t motor_get_status(drive_id_t drive);

/**
 * @brief Check if drive has completed motion
 * 
 * @param drive Drive identifier
 * @return true if motion is complete
 */
bool motor_is_done(drive_id_t drive);

/**
 * @brief Check if drive has an error
 * 
 * @param drive Drive identifier
 * @return true if drive has error
 */
bool motor_has_error(drive_id_t drive);

/**
 * @brief Process SOEM cyclic communication
 * 
 * Must be called periodically to maintain EtherCAT communication.
 * Call this from the main loop or a dedicated task.
 */
void motor_process(void);

/**
 * @brief De-initialize motor control subsystem
 */
void motor_control_deinit(void);

// =============================================================================
// Utility Functions
// =============================================================================

/**
 * @brief Convert degrees to encoder counts
 * 
 * @param degrees Angle in degrees
 * @return Encoder counts
 */
int32_t deg_to_enc(double degrees);

/**
 * @brief Convert encoder counts to degrees
 * 
 * @param enc_counts Encoder counts
 * @return Angle in degrees
 */
double enc_to_deg(int32_t enc_counts);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
