#ifndef SCAN_PATTERNS_H
#define SCAN_PATTERNS_H

/**
 * @file scan_patterns.h
 * @brief Scan pattern generation for laser acquisition
 * 
 * Provides spiral and figure-8 scan pattern generation and execution
 * for laser beam tracking.
 */

#include <stdint.h>
#include <stdbool.h>
#include "acquire_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Single point in a scan path
 */
typedef struct {
    double pan_deg;   /**< Pan position in degrees */
    double tilt_deg;  /**< Tilt position in degrees */
} path_point_t;

/**
 * @brief Logged position with power reading
 */
typedef struct {
    double pan_deg;   /**< Pan position in degrees */
    double tilt_deg;  /**< Tilt position in degrees */
    float power_uw;   /**< Power reading in microwatts */
} log_entry_t;

/**
 * @brief Spiral scan parameters
 */
typedef struct {
    double center_pan_deg;     /**< Center pan position */
    double center_tilt_deg;    /**< Center tilt position */
    double angular_step_deg;   /**< Angular step per point (default 5°) */
    double growth_per_rev_deg; /**< Radial growth per revolution (default 0.5°) */
    double max_radius_deg;     /**< Maximum spiral radius (default 5°) */
} spiral_params_t;

/**
 * @brief Figure-8 scan parameters
 */
typedef struct {
    double center_pan_deg;     /**< Center pan position */
    double center_tilt_deg;    /**< Center tilt position */
    double width_deg;          /**< Half-width in pan axis */
    double height_deg;         /**< Half-height in tilt axis */
    int points_per_cycle;      /**< Points per figure-8 cycle */
} figure8_params_t;

// =============================================================================
// Spiral Pattern Functions
// =============================================================================

/**
 * @brief Generate spiral scan path
 * 
 * Creates an outward spiral pattern centered at the given position.
 * 
 * @param params Spiral parameters
 * @param path_buffer Output array for path points
 * @param buffer_size Maximum number of points in buffer
 * @return Number of points generated
 */
int generate_spiral(const spiral_params_t *params, 
                    path_point_t *path_buffer, 
                    int buffer_size);

/**
 * @brief Generate spiral with default parameters
 * 
 * @param center_pan_deg Center pan position
 * @param center_tilt_deg Center tilt position
 * @param path_buffer Output array for path points
 * @param buffer_size Maximum number of points in buffer
 * @return Number of points generated
 */
int generate_spiral_default(double center_pan_deg, 
                            double center_tilt_deg,
                            path_point_t *path_buffer, 
                            int buffer_size);

// =============================================================================
// Figure-8 Pattern Functions
// =============================================================================

/**
 * @brief Calculate figure-8 position at given phase
 * 
 * @param params Figure-8 parameters
 * @param phase Current phase (0.0 to 1.0 for one complete cycle)
 * @param[out] pan_deg Output pan position
 * @param[out] tilt_deg Output tilt position
 */
void figure8_position(const figure8_params_t *params,
                      double phase,
                      double *pan_deg,
                      double *tilt_deg);

/**
 * @brief Generate figure-8 scan path
 * 
 * @param params Figure-8 parameters
 * @param path_buffer Output array for path points
 * @param buffer_size Maximum number of points in buffer
 * @return Number of points generated (equals points_per_cycle)
 */
int generate_figure8(const figure8_params_t *params,
                     path_point_t *path_buffer,
                     int buffer_size);

// =============================================================================
// Peak Finding Functions
// =============================================================================

/**
 * @brief Find position with maximum power in log data
 * 
 * @param log_data Array of logged positions with power
 * @param log_count Number of entries in log
 * @param[out] peak_pan_deg Pan position at peak power
 * @param[out] peak_tilt_deg Tilt position at peak power
 * @param[out] peak_power_uw Power value at peak (optional, can be NULL)
 * @return true if peak found, false if log empty
 */
bool find_peak_power(const log_entry_t *log_data,
                     int log_count,
                     double *peak_pan_deg,
                     double *peak_tilt_deg,
                     float *peak_power_uw);

// =============================================================================
// Logging Functions  
// =============================================================================

/**
 * @brief Initialize the position/power logger
 */
void logger_init(void);

/**
 * @brief Start logging position and power data
 */
void logger_start(void);

/**
 * @brief Stop logging
 */
void logger_stop(void);

/**
 * @brief Log current position and power
 * 
 * Call this periodically (e.g., every 5ms) while logging is active.
 * 
 * @param pan_deg Current pan position
 * @param tilt_deg Current tilt position  
 * @param power_uw Current power reading
 * @return true if logged, false if log full or inactive
 */
bool logger_record(double pan_deg, double tilt_deg, float power_uw);

/**
 * @brief Check if logger is active
 * 
 * @return true if logging
 */
bool logger_is_active(void);

/**
 * @brief Get current log count
 * 
 * @return Number of entries logged
 */
int logger_get_count(void);

/**
 * @brief Get pointer to log data
 * 
 * @return Pointer to internal log buffer (read-only access)
 */
const log_entry_t* logger_get_data(void);

/**
 * @brief Clear log data
 */
void logger_clear(void);

#ifdef __cplusplus
}
#endif

#endif // SCAN_PATTERNS_H
