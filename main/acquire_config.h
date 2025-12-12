#ifndef ACQUIRE_CONFIG_H
#define ACQUIRE_CONFIG_H

/**
 * @file acquire_config.h
 * @brief Configuration constants for the laser acquisition system
 */

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Encoder Configuration
// =============================================================================

/** Encoder counts per revolution (Xeryon stages) */
#define ENCODER_RESOLUTION          1843200

/** Allowed position error in encoder counts */
#define ENCODER_TOLERANCE           100

// =============================================================================
// Starting Position Configuration (degrees)
// =============================================================================

/** Initial pan (yaw) position in degrees */
#define START_PAN_DEG               (-2.2)

/** Initial tilt (pitch) position in degrees */
#define START_TILT_DEG              (178.4)

// =============================================================================
// Motion Parameters
// =============================================================================

/** Default drive speed */
#define DEFAULT_DRIVE_SPEED         10000

/** Default drive acceleration */
#define DEFAULT_DRIVE_ACC           65000

/** Default drive deceleration */
#define DEFAULT_DRIVE_DEC           65000

// =============================================================================
// Power Detection Thresholds
// =============================================================================

/** Minimum power threshold (µW) to consider signal detected */
#define MIN_POWER_THRESHOLD_UW      70.0f

/** Power threshold (µW) below which signal is considered lost */
#define SIGNAL_LOSS_THRESHOLD_UW    30.0f

// =============================================================================
// Timing Configuration (milliseconds)
// =============================================================================

/** Standard wait/transition time between states */
#define STATE_WAIT_TIME_MS          1000

/** Hold time at peak position */
#define HOLD_TIME_MS                10000

/** Power settle time after move */
#define POWER_SETTLE_TIME_MS        150

/** Signal loss timeout before restarting scan */
#define SIGNAL_LOSS_TIMEOUT_MS      2000

// =============================================================================
// Scan Pattern Configuration
// =============================================================================

/** Maximum points in spiral path buffer */
#define MAX_PATH_POINTS             600

/** Maximum entries in continuous history log */
#define MAX_LOG_ENTRIES             5000

// =============================================================================
// I2C Power Sensor Configuration
// =============================================================================

/** I2C address of power sensor */
#define POWER_SENSOR_I2C_ADDR       0x51

/** Power sensor register 1 */
#define POWER_SENSOR_REG1           0x68

/** Power sensor register 2 */
#define POWER_SENSOR_REG2           0x69

/** I2C SDA GPIO pin */
#define I2C_SDA_GPIO                2

/** I2C SCL GPIO pin */
#define I2C_SCL_GPIO                15

/** I2C clock frequency (Hz) */
#define I2C_FREQ_HZ                 100000

// =============================================================================
// Spiral Pattern Parameters
// =============================================================================

/** Spiral angular step (degrees) per point */
#define SPIRAL_ANGULAR_STEP_DEG     5.0

/** Spiral radial growth per revolution (degrees) */
#define SPIRAL_GROWTH_PER_REV_DEG   0.5

/** Maximum spiral radius (degrees) */
#define SPIRAL_MAX_RADIUS_DEG       5.0

// =============================================================================
// Figure-8 Pattern Parameters
// =============================================================================

/** Figure-8 half-width in pan axis (degrees) */
#define FIG8_WIDTH_DEG              0.5

/** Figure-8 half-height in tilt axis (degrees) */
#define FIG8_HEIGHT_DEG             0.5

/** Figure-8 points per cycle */
#define FIG8_POINTS_PER_CYCLE       32

#ifdef __cplusplus
}
#endif

#endif // ACQUIRE_CONFIG_H
