/**
 * @file scan_patterns.cpp
 * @brief Scan pattern generation and logging implementation
 * 
 * Implements spiral and figure-8 patterns for laser beam acquisition.
 */

#include "scan_patterns.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

static const char *TAG = "scan_patterns";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// Internal Logger State
// =============================================================================

static struct {
    log_entry_t data[MAX_LOG_ENTRIES];
    int index;
    bool active;
} s_logger = {};

// =============================================================================
// Spiral Pattern Implementation
// =============================================================================

extern "C" {

int generate_spiral(const spiral_params_t *params,
                    path_point_t *path_buffer,
                    int buffer_size)
{
    if (params == nullptr || path_buffer == nullptr || buffer_size <= 0) {
        return 0;
    }

    int point_count = 0;
    double angle_rad = 0.0;
    double radius = 0.0;
    
    const double angular_step_rad = params->angular_step_deg * M_PI / 180.0;
    const double growth_rate = params->growth_per_rev_deg / 360.0; // deg per degree of rotation
    
    ESP_LOGI(TAG, "Generating spiral: center=(%.2f, %.2f), max_radius=%.2f",
             params->center_pan_deg, params->center_tilt_deg, params->max_radius_deg);

    while (radius <= params->max_radius_deg && point_count < buffer_size) {
        // Convert polar to cartesian offset
        double pan_offset = radius * cos(angle_rad);
        double tilt_offset = radius * sin(angle_rad);
        
        // Add to center position
        path_buffer[point_count].pan_deg = params->center_pan_deg + pan_offset;
        path_buffer[point_count].tilt_deg = params->center_tilt_deg + tilt_offset;
        
        point_count++;
        
        // Advance angle and radius (Archimedean spiral)
        angle_rad += angular_step_rad;
        radius = growth_rate * (angle_rad * 180.0 / M_PI); // Convert accumulated angle to degrees
    }
    
    ESP_LOGI(TAG, "Generated %d spiral points", point_count);
    return point_count;
}

int generate_spiral_default(double center_pan_deg,
                            double center_tilt_deg,
                            path_point_t *path_buffer,
                            int buffer_size)
{
    spiral_params_t params = {
        .center_pan_deg = center_pan_deg,
        .center_tilt_deg = center_tilt_deg,
        .angular_step_deg = SPIRAL_ANGULAR_STEP_DEG,
        .growth_per_rev_deg = SPIRAL_GROWTH_PER_REV_DEG,
        .max_radius_deg = SPIRAL_MAX_RADIUS_DEG
    };
    
    return generate_spiral(&params, path_buffer, buffer_size);
}

// =============================================================================
// Figure-8 Pattern Implementation
// =============================================================================

void figure8_position(const figure8_params_t *params,
                      double phase,
                      double *pan_deg,
                      double *tilt_deg)
{
    if (params == nullptr || pan_deg == nullptr || tilt_deg == nullptr) {
        return;
    }
    
    // Lissajous figure-8 pattern
    // x = A * sin(t)
    // y = B * sin(2t)
    // This creates a figure-8 (infinity symbol) shape
    
    double t = phase * 2.0 * M_PI;
    
    *pan_deg = params->center_pan_deg + params->width_deg * sin(t);
    *tilt_deg = params->center_tilt_deg + params->height_deg * sin(2.0 * t);
}

int generate_figure8(const figure8_params_t *params,
                     path_point_t *path_buffer,
                     int buffer_size)
{
    if (params == nullptr || path_buffer == nullptr || buffer_size <= 0) {
        return 0;
    }
    
    int points = params->points_per_cycle;
    if (points > buffer_size) {
        points = buffer_size;
    }
    
    ESP_LOGI(TAG, "Generating figure-8: center=(%.2f, %.2f), size=(%.2f x %.2f)",
             params->center_pan_deg, params->center_tilt_deg,
             params->width_deg, params->height_deg);
    
    for (int i = 0; i < points; i++) {
        double phase = static_cast<double>(i) / static_cast<double>(points);
        figure8_position(params, phase, 
                        &path_buffer[i].pan_deg, 
                        &path_buffer[i].tilt_deg);
    }
    
    ESP_LOGI(TAG, "Generated %d figure-8 points", points);
    return points;
}

// =============================================================================
// Peak Finding Implementation
// =============================================================================

bool find_peak_power(const log_entry_t *log_data,
                     int log_count,
                     double *peak_pan_deg,
                     double *peak_tilt_deg,
                     float *peak_power_uw)
{
    if (log_data == nullptr || log_count <= 0 || 
        peak_pan_deg == nullptr || peak_tilt_deg == nullptr) {
        return false;
    }
    
    float max_power = -1.0f;
    int max_index = 0;
    
    for (int i = 0; i < log_count; i++) {
        if (log_data[i].power_uw > max_power) {
            max_power = log_data[i].power_uw;
            max_index = i;
        }
    }
    
    *peak_pan_deg = log_data[max_index].pan_deg;
    *peak_tilt_deg = log_data[max_index].tilt_deg;
    
    if (peak_power_uw != nullptr) {
        *peak_power_uw = max_power;
    }
    
    ESP_LOGI(TAG, "Peak found at index %d: (%.3f, %.3f) @ %.2f µW",
             max_index, *peak_pan_deg, *peak_tilt_deg, max_power);
    
    return true;
}

// =============================================================================
// Logger Implementation
// =============================================================================

void logger_init(void)
{
    memset(&s_logger, 0, sizeof(s_logger));
    ESP_LOGI(TAG, "Logger initialized (capacity=%d entries)", MAX_LOG_ENTRIES);
}

void logger_start(void)
{
    s_logger.index = 0;
    s_logger.active = true;
    ESP_LOGI(TAG, "Logging started");
}

void logger_stop(void)
{
    s_logger.active = false;
    ESP_LOGI(TAG, "Logging stopped (%d entries)", s_logger.index);
}

bool logger_record(double pan_deg, double tilt_deg, float power_uw)
{
    if (!s_logger.active) {
        return false;
    }
    
    if (s_logger.index >= MAX_LOG_ENTRIES) {
        // Stop logging when array is full
        s_logger.active = false;
        ESP_LOGW(TAG, "Log full, logging stopped");
        return false;
    }
    
    s_logger.data[s_logger.index].pan_deg = pan_deg;
    s_logger.data[s_logger.index].tilt_deg = tilt_deg;
    s_logger.data[s_logger.index].power_uw = power_uw;
    s_logger.index++;
    
    return true;
}

bool logger_is_active(void)
{
    return s_logger.active;
}

int logger_get_count(void)
{
    return s_logger.index;
}

const log_entry_t* logger_get_data(void)
{
    return s_logger.data;
}

void logger_clear(void)
{
    s_logger.index = 0;
    s_logger.active = false;
}

} // extern "C"
