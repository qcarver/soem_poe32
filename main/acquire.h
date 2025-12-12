/**
 * @file acquire.h
 * @brief Laser beam acquisition system for pan/tilt mirror alignment
 * 
 * This module implements a two-phase acquisition algorithm:
 * 1. Spiral scan - coarse search pattern to find receiver signal
 * 2. Figure-8 scan - fine tracking pattern to maximize signal strength
 * 
 * Ported from CODESYS V3.5 PLC program.
 */

#ifndef ACQUIRE_H
#define ACQUIRE_H

#include <cstdint>
#include <array>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration Constants
// ============================================================================

/** Maximum number of points in a spiral scan path */
#define MAX_PATH_POINTS         600

/** Encoder counts per revolution (Xeryon stage resolution) */
#define ENCODER_RESOLUTION      1843200

/** Default motor speed (encoder units/sec) */
#define DEFAULT_SPEED           10000

/** Default acceleration (encoder units/sec²) */
#define DEFAULT_ACCEL           65000

/** Default deceleration (encoder units/sec²) */
#define DEFAULT_DECEL           65000

/** Minimum power threshold to consider signal detected (µW) */
#define MIN_POWER_THRESHOLD_UW  70.0f

/** Position tolerance in encoder counts */
#define ENCODER_TOLERANCE       100

/** Signal loss timeout before restarting scan (ms) */
#define SIGNAL_LOSS_TIMEOUT_MS  2000

/** Wait time between state transitions (ms) */
#define STATE_WAIT_TIME_MS      1000

/** Power settle time after move (ms) */
#define POWER_SETTLE_TIME_MS    150

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize the acquisition system
 * 
 * Sets up motor drivers, initializes state machine, and prepares for scanning.
 * 
 * @return 0 on success, negative error code on failure
 */
int acquire_init(void);

/**
 * @brief Start the acquisition process
 * 
 * Begins the state machine which will:
 * 1. Enable and index motors
 * 2. Move to starting position
 * 3. Execute spiral scan until signal detected
 * 4. Refine position with Figure-8 tracking
 * 
 * @return 0 on success, negative error code on failure
 */
int acquire_start(void);

/**
 * @brief Stop acquisition and halt motors
 */
void acquire_stop(void);

/**
 * @brief Run one iteration of the acquisition state machine
 * 
 * Call this periodically (e.g., from a FreeRTOS task or main loop).
 * The state machine is non-blocking and maintains its own state.
 */
void acquire_update(void);

/**
 * @brief Get current system state
 * 
 * @return Current state number (0-10)
 */
int acquire_get_state(void);

/**
 * @brief Get current power reading
 * 
 * @return Power in microwatts (µW)
 */
float acquire_get_power_uw(void);

/**
 * @brief Get current pan angle
 * 
 * @return Pan angle in degrees
 */
double acquire_get_pan_deg(void);

/**
 * @brief Get current tilt angle
 * 
 * @return Tilt angle in degrees
 */
double acquire_get_tilt_deg(void);

/**
 * @brief Check if acquisition has locked onto target
 * 
 * @return true if in tracking mode (Figure-8), false otherwise
 */
bool acquire_is_locked(void);

/**
 * @brief Set starting position for scan
 * 
 * @param pan_deg Starting pan angle in degrees
 * @param tilt_deg Starting tilt angle in degrees
 */
void acquire_set_start_position(double pan_deg, double tilt_deg);

/**
 * @brief Set minimum power threshold for detection
 * 
 * @param threshold_uw Power threshold in microwatts
 */
void acquire_set_power_threshold(float threshold_uw);

/**
 * @brief Emergency halt - stop all motors immediately
 */
void acquire_halt(void);

/**
 * @brief Reset the acquisition system
 * 
 * Clears errors and returns to initial state.
 */
void acquire_reset(void);

#ifdef __cplusplus
}
#endif

// ============================================================================
// C++ Internal Types (only available in C++)
// ============================================================================

#ifdef __cplusplus

#include <functional>

namespace acquire {

/**
 * @brief Acquisition system states
 */
enum class State : int {
    INIT_MOTORS = 0,        ///< Enable and initialize drives
    INDEX_MOTORS = 1,       ///< Home/index drives
    MOVE_TO_START = 2,      ///< Move to starting scan position
    GENERATE_SPIRAL = 3,    ///< Generate spiral path pattern
    EXECUTE_SPIRAL = 4,     ///< Execute spiral scan point-by-point
    FIND_PEAK = 5,          ///< Analyze data to find peak power position
    MOVE_TO_PEAK = 6,       ///< Move to peak power position
    FIGURE8_TRACKING = 7,   ///< Figure-8 fine tracking pattern
    HOLD = 8,               ///< Hold at current position
    ERROR = 10              ///< Error state
};

/**
 * @brief Motor axis parameters
 */
struct AxisParams {
    int32_t speed = DEFAULT_SPEED;
    int32_t accel = DEFAULT_ACCEL;
    int32_t decel = DEFAULT_DECEL;
};

/**
 * @brief Path point for scan patterns
 */
struct PathPoint {
    double pan_deg;         ///< Pan angle in degrees
    double tilt_deg;        ///< Tilt angle in degrees
};

/**
 * @brief Drive status information
 */
struct DriveStatus {
    bool enabled;
    bool busy;
    bool done;
    bool has_error;
    int32_t act_position;   ///< Current position in encoder counts
    int32_t error_code;
};

/**
 * @brief Power measurement history entry
 */
struct PowerHistoryEntry {
    double pan_deg;
    double tilt_deg;
    float power_uw;
    uint32_t timestamp_ms;
};

// Utility functions

/**
 * @brief Convert degrees to encoder counts
 */
inline int32_t deg_to_enc(double degrees) {
    // Formula: enc = deg * (2π / 360) * 1e6 / (2π * 1e6 / ENCODER_RESOLUTION)
    // Simplifies to: enc = deg * ENCODER_RESOLUTION / 360
    return static_cast<int32_t>(degrees * ENCODER_RESOLUTION / 360.0);
}

/**
 * @brief Convert encoder counts to degrees
 */
inline double enc_to_deg(int32_t enc) {
    return static_cast<double>(enc) * 360.0 / ENCODER_RESOLUTION;
}

} // namespace acquire

#endif // __cplusplus

#endif // ACQUIRE_H
