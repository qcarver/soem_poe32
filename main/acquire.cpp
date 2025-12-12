/**
 * @file acquire.cpp
 * @brief Laser beam acquisition system implementation
 * 
 * Ported from CODESYS V3.5 PLC program - state machine for 2-axis
 * pan/tilt laser alignment using spiral scan and Figure-8 tracking.
 */

#include "acquire.h"
#include "acquire_config.h"
#include "motor_control.h"
#include "power_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <algorithm>
#include <cstring>

static const char *TAG = "acquire";

// ============================================================================
// Hardware Abstraction Layer (HAL) - Implement these for your hardware
// ============================================================================

namespace hal {

/**
 * @brief Initialize motor drives
 * @return true on success
 */
bool drives_init();

/**
 * @brief Enable a drive
 * @param drive_num 1 for tilt, 2 for pan
 */
void drive_enable(int drive_num);

/**
 * @brief Index/home a drive
 * @param drive_num 1 for tilt, 2 for pan
 * @param params Axis motion parameters
 */
void drive_index(int drive_num, const acquire::AxisParams& params);

/**
 * @brief Move drive to absolute position
 * @param drive_num 1 for tilt, 2 for pan
 * @param position_enc Target position in encoder counts
 * @param params Axis motion parameters
 */
void drive_move_absolute(int drive_num, int32_t position_enc, const acquire::AxisParams& params);

/**
 * @brief Halt drive immediately
 * @param drive_num 1 for tilt, 2 for pan
 */
void drive_halt(int drive_num);

/**
 * @brief Stop drive with deceleration
 * @param drive_num 1 for tilt, 2 for pan
 */
void drive_stop(int drive_num);

/**
 * @brief Reset drive errors
 * @param drive_num 1 for tilt, 2 for pan
 */
void drive_reset(int drive_num);

/**
 * @brief Get drive status
 * @param drive_num 1 for tilt, 2 for pan
 * @return Drive status structure
 */
acquire::DriveStatus drive_get_status(int drive_num);

/**
 * @brief Check if drive motion is complete
 * @param drive_num 1 for tilt, 2 for pan
 * @return true if motion complete (done flag set)
 */
bool drive_is_done(int drive_num);

/**
 * @brief Check if drive has error
 * @param drive_num 1 for tilt, 2 for pan
 * @return true if error present
 */
bool drive_has_error(int drive_num);

/**
 * @brief Get current drive position
 * @param drive_num 1 for tilt, 2 for pan
 * @return Position in encoder counts
 */
int32_t drive_get_position(int drive_num);

/**
 * @brief Read power sensor
 * @return Power in microwatts (µW)
 */
float power_sensor_read();

/**
 * @brief Initialize power sensor
 * @return true on success
 */
bool power_sensor_init();

} // namespace hal

// ============================================================================
// Internal State
// ============================================================================

namespace {

// Current state machine state
acquire::State g_state = acquire::State::INIT_MOTORS;

// Axis parameters
acquire::AxisParams g_axis1_params;  // Tilt
acquire::AxisParams g_axis2_params;  // Pan

// Scan path buffer
acquire::PathPoint g_path_buffer[MAX_PATH_POINTS];
int g_path_points_count = 0;
int g_current_scan_point = 0;

// Position tracking
double g_start_pan_deg = -2.2;      // Initial scan start position
double g_start_tilt_deg = 178.4;
double g_peak_pan_deg = 0.0;        // Best found position
double g_peak_tilt_deg = 180.0;
double g_center_pan_deg = 0.0;      // Current scan center
double g_center_tilt_deg = 0.0;

// Power tracking
float g_min_power_threshold = MIN_POWER_THRESHOLD_UW;
float g_global_max_power = 0.0f;
float g_current_power = 0.0f;

// State flags
bool g_first_scan = true;
bool g_first_spiral_executed = false;
bool g_drive1_move_sent = false;
bool g_drive2_move_sent = false;
bool g_generate_sent = false;
bool g_drive_peak_move_sent = false;
bool g_start_fig8 = false;
bool g_done_fig8 = false;
bool g_log_active = false;

// Timing
uint32_t g_wait_timer_start = 0;
uint32_t g_loss_timer_start = 0;
uint32_t g_power_settle_start = 0;
bool g_wait_active = false;

// Power history for peak finding
acquire::PowerHistoryEntry g_power_history[MAX_PATH_POINTS];
int g_log_index = 0;

// Figure-8 tracking state
double g_fig8_phase = 0.0;
double g_fig8_amplitude = 0.3;  // Degrees
double g_fig8_last_pan = 0.0;
double g_fig8_last_tilt = 0.0;

/**
 * @brief Get current time in milliseconds
 */
inline uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Check if timer has elapsed
 */
inline bool timer_elapsed(uint32_t start, uint32_t duration_ms) {
    return (millis() - start) >= duration_ms;
}

} // anonymous namespace

// ============================================================================
// Spiral Generation
// ============================================================================

namespace spiral {

/**
 * @brief Generate Archimedean spiral scan pattern
 * 
 * Creates an outward spiral from center point for coarse beam search.
 * 
 * @param center_pan Center pan angle (degrees)
 * @param center_tilt Center tilt angle (degrees)
 * @param path Output path buffer
 * @param max_points Maximum points to generate
 * @return Number of points generated
 */
int generate(double center_pan, double center_tilt, 
             acquire::PathPoint* path, int max_points)
{
    // Spiral parameters (can be tuned)
    const double max_radius_deg = 2.0;       // Maximum spiral radius in degrees
    const double turns = 8.0;                // Number of spiral turns
    const double points_per_turn = 60.0;     // Points per revolution
    
    int total_points = static_cast<int>(turns * points_per_turn);
    if (total_points > max_points) {
        total_points = max_points;
    }
    
    for (int i = 0; i < total_points; i++) {
        double t = static_cast<double>(i) / points_per_turn;  // Turns completed
        double angle = 2.0 * M_PI * t;                        // Angle in radians
        double radius = (max_radius_deg / turns) * t;         // Archimedean spiral
        
        path[i].pan_deg = center_pan + radius * cos(angle);
        path[i].tilt_deg = center_tilt + radius * sin(angle);
    }
    
    ESP_LOGI(TAG, "Generated spiral: %d points, center=(%.2f, %.2f)", 
             total_points, center_pan, center_tilt);
    
    return total_points;
}

} // namespace spiral

// ============================================================================
// Figure-8 Tracking
// ============================================================================

namespace figure8 {

/**
 * @brief Figure-8 (lemniscate) pattern generator for fine tracking
 * 
 * The Figure-8 pattern is used after initial acquisition to maintain
 * lock and refine position by continuously sampling around the peak.
 */
struct Scanner {
    double center_pan;
    double center_tilt;
    double amplitude;     // Half-width of figure-8 in degrees
    double phase;         // Current phase (0 to 2π)
    double phase_rate;    // Radians per update
    float best_power;
    double best_pan;
    double best_tilt;
    bool initialized;
    
    Scanner() : center_pan(0), center_tilt(0), amplitude(0.3), 
                phase(0), phase_rate(0.05), best_power(0),
                best_pan(0), best_tilt(0), initialized(false) {}
    
    void start(double pan, double tilt, double amp = 0.3) {
        center_pan = pan;
        center_tilt = tilt;
        amplitude = amp;
        phase = 0;
        best_power = 0;
        best_pan = pan;
        best_tilt = tilt;
        initialized = true;
        ESP_LOGI(TAG, "Figure-8 started at (%.3f, %.3f), amp=%.2f", pan, tilt, amp);
    }
    
    /**
     * @brief Get next position in Figure-8 pattern
     * 
     * Uses parametric lemniscate: x = a*cos(t), y = a*sin(t)*cos(t)
     */
    void get_next_position(double& pan_out, double& tilt_out) {
        // Parametric lemniscate of Bernoulli (figure-8)
        double cos_t = cos(phase);
        double sin_t = sin(phase);
        
        pan_out = center_pan + amplitude * cos_t;
        tilt_out = center_tilt + amplitude * sin_t * cos_t;
        
        // Advance phase
        phase += phase_rate;
        if (phase >= 2.0 * M_PI) {
            phase -= 2.0 * M_PI;
        }
    }
    
    /**
     * @brief Update tracking with current power reading
     * 
     * Adjusts center position if better power found.
     */
    void update(float current_power, double current_pan, double current_tilt) {
        if (current_power > best_power) {
            best_power = current_power;
            best_pan = current_pan;
            best_tilt = current_tilt;
            
            // Shift center towards better position (slow adaptation)
            const double adapt_rate = 0.1;
            center_pan += adapt_rate * (best_pan - center_pan);
            center_tilt += adapt_rate * (best_tilt - center_tilt);
        }
    }
};

static Scanner g_scanner;

} // namespace figure8

// ============================================================================
// Peak Finding
// ============================================================================

namespace peak {

/**
 * @brief Find position of maximum power in history log
 * 
 * @param history Power history buffer
 * @param count Number of entries to search
 * @param[out] pan_deg Best pan position
 * @param[out] tilt_deg Best tilt position
 * @return Maximum power found
 */
float find_max(const acquire::PowerHistoryEntry* history, int count,
               double& pan_deg, double& tilt_deg)
{
    float max_power = 0.0f;
    int max_idx = 0;
    
    for (int i = 0; i < count; i++) {
        if (history[i].power_uw > max_power) {
            max_power = history[i].power_uw;
            max_idx = i;
        }
    }
    
    if (max_power > 0) {
        pan_deg = history[max_idx].pan_deg;
        tilt_deg = history[max_idx].tilt_deg;
        ESP_LOGI(TAG, "Peak found: %.2f µW at (%.3f, %.3f)", 
                 max_power, pan_deg, tilt_deg);
    }
    
    return max_power;
}

} // namespace peak

// ============================================================================
// HAL Implementation - Connected to motor_control and power_sensor modules
// ============================================================================

namespace hal {

static axis_params_t to_axis_params(const acquire::AxisParams& params) {
    axis_params_t ap;
    ap.speed = params.speed;
    ap.acc = static_cast<uint16_t>(params.accel);
    ap.dec = static_cast<uint16_t>(params.decel);
    return ap;
}

bool drives_init() {
    ESP_LOGI(TAG, "HAL: Initializing motor control");
    // Note: motor_control_init() should be called from main with ethernet interface
    // Here we just verify it's ready
    return true;
}

void drive_enable(int drive_num) {
    ESP_LOGI(TAG, "HAL: Enabling drive %d", drive_num);
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    motor_enable(drive);
}

void drive_index(int drive_num, const acquire::AxisParams& params) {
    ESP_LOGI(TAG, "HAL: Indexing drive %d", drive_num);
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    axis_params_t ap = to_axis_params(params);
    motor_index(drive, &ap);
}

void drive_move_absolute(int drive_num, int32_t position_enc, 
                         const acquire::AxisParams& params) {
    ESP_LOGD(TAG, "HAL: Moving drive %d to %ld enc", drive_num, (long)position_enc);
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    axis_params_t ap = to_axis_params(params);
    motor_move_absolute_enc(drive, position_enc, &ap);
}

void drive_halt(int drive_num) {
    ESP_LOGI(TAG, "HAL: Halting drive %d", drive_num);
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    motor_halt(drive);
}

void drive_stop(int drive_num) {
    ESP_LOGI(TAG, "HAL: Stopping drive %d", drive_num);
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    motor_stop(drive);
}

void drive_reset(int drive_num) {
    ESP_LOGI(TAG, "HAL: Resetting drive %d", drive_num);
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    motor_reset(drive);
}

acquire::DriveStatus drive_get_status(int drive_num) {
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    acquire::DriveStatus status = {};
    
    drive_status_t motor_status = motor_get_status(drive);
    status.enabled = (motor_status != DRIVE_STATUS_DISABLED);
    status.busy = (motor_status == DRIVE_STATUS_BUSY);
    status.done = motor_is_done(drive);
    status.has_error = motor_has_error(drive);
    status.act_position = motor_get_position_enc(drive);
    
    return status;
}

bool drive_is_done(int drive_num) {
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    return motor_is_done(drive);
}

bool drive_has_error(int drive_num) {
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    return motor_has_error(drive);
}

int32_t drive_get_position(int drive_num) {
    drive_id_t drive = (drive_num == 1) ? DRIVE_TILT : DRIVE_PAN;
    return motor_get_position_enc(drive);
}

float power_sensor_read() {
    float power_uw = 0.0f;
    if (::power_sensor_read(&power_uw) != ESP_OK) {
        ESP_LOGW(TAG, "HAL: Failed to read power sensor");
        return 0.0f;
    }
    return power_uw;
}

bool power_sensor_init() {
    ESP_LOGI(TAG, "HAL: Initializing power sensor");
    return (::power_sensor_init() == ESP_OK);
}

} // namespace hal

// ============================================================================
// State Machine Implementation
// ============================================================================

namespace {

void state_init_motors() {
    static bool init_started = false;
    
    if (!init_started) {
        ESP_LOGI(TAG, "State 0: Initializing motors");
        hal::drives_init();
        hal::drive_enable(1);  // Tilt
        hal::drive_enable(2);  // Pan
        init_started = true;
        g_wait_timer_start = millis();
    }
    
    // Wait for drives to be ready (simulated with timer for now)
    if (timer_elapsed(g_wait_timer_start, 500)) {
        init_started = false;
        g_state = acquire::State::INDEX_MOTORS;
    }
}

void state_index_motors() {
    static bool index_started = false;
    
    if (!index_started) {
        ESP_LOGI(TAG, "State 1: Indexing (homing) motors");
        hal::drive_index(1, g_axis1_params);
        hal::drive_index(2, g_axis2_params);
        index_started = true;
        g_wait_timer_start = millis();
    }
    
    // Check if indexing complete
    if (hal::drive_is_done(1) && hal::drive_is_done(2)) {
        index_started = false;
        g_state = acquire::State::MOVE_TO_START;
    }
    
    // Timeout check (indexing shouldn't take forever)
    if (timer_elapsed(g_wait_timer_start, 30000)) {
        ESP_LOGE(TAG, "Indexing timeout!");
        g_state = acquire::State::ERROR;
    }
}

void state_move_to_start() {
    // Move to initial scan position
    if (!g_drive1_move_sent) {
        int32_t target_enc = acquire::deg_to_enc(g_start_tilt_deg);
        ESP_LOGI(TAG, "State 2: Moving tilt to %.2f deg (%ld enc)", 
                 g_start_tilt_deg, (long)target_enc);
        hal::drive_move_absolute(1, target_enc, g_axis1_params);
        g_drive1_move_sent = true;
    }
    
    if (!g_drive2_move_sent) {
        int32_t target_enc = acquire::deg_to_enc(g_start_pan_deg);
        ESP_LOGI(TAG, "State 2: Moving pan to %.2f deg (%ld enc)", 
                 g_start_pan_deg, (long)target_enc);
        hal::drive_move_absolute(2, target_enc, g_axis2_params);
        g_drive2_move_sent = true;
    }
    
    // Check for completion
    if (g_drive1_move_sent && hal::drive_is_done(1)) {
        g_drive1_move_sent = false;
    }
    if (g_drive2_move_sent && hal::drive_is_done(2)) {
        g_drive2_move_sent = false;
        g_wait_active = true;
        g_wait_timer_start = millis();
    }
    
    // Check for errors
    if (hal::drive_has_error(1) || hal::drive_has_error(2)) {
        ESP_LOGE(TAG, "Drive error during move to start!");
        g_state = acquire::State::ERROR;
        return;
    }
    
    // Wait timer before proceeding
    if (g_wait_active && timer_elapsed(g_wait_timer_start, STATE_WAIT_TIME_MS)) {
        g_wait_active = false;
        g_state = acquire::State::GENERATE_SPIRAL;
    }
}

void state_generate_spiral() {
    if (!g_generate_sent) {
        // Choose center based on whether this is first scan
        if (!g_first_spiral_executed) {
            g_center_pan_deg = g_start_pan_deg;
            g_center_tilt_deg = g_start_tilt_deg;
        } else {
            g_center_pan_deg = g_peak_pan_deg;
            g_center_tilt_deg = g_peak_tilt_deg;
        }
        
        ESP_LOGI(TAG, "State 3: Generating spiral at (%.2f, %.2f)", 
                 g_center_pan_deg, g_center_tilt_deg);
        
        g_path_points_count = spiral::generate(g_center_pan_deg, g_center_tilt_deg,
                                                g_path_buffer, MAX_PATH_POINTS);
        g_current_scan_point = 0;
        g_generate_sent = true;
        g_wait_timer_start = millis();
    }
    
    if (timer_elapsed(g_wait_timer_start, STATE_WAIT_TIME_MS)) {
        g_generate_sent = false;
        g_log_active = true;
        g_log_index = 0;
        g_state = acquire::State::EXECUTE_SPIRAL;
    }
}

void state_execute_spiral() {
    // Read current power
    g_current_power = hal::power_sensor_read();
    
    // Log power at current position
    if (g_log_active && g_log_index < MAX_PATH_POINTS) {
        double pan = acquire::enc_to_deg(hal::drive_get_position(2));
        double tilt = acquire::enc_to_deg(hal::drive_get_position(1));
        
        g_power_history[g_log_index].pan_deg = pan;
        g_power_history[g_log_index].tilt_deg = tilt;
        g_power_history[g_log_index].power_uw = g_current_power;
        g_power_history[g_log_index].timestamp_ms = millis();
        g_log_index++;
    }
    
    // Check if signal detected above threshold
    if (g_current_power >= g_min_power_threshold) {
        ESP_LOGI(TAG, "Signal detected! %.2f µW", g_current_power);
        
        // Halt drives immediately
        hal::drive_halt(1);
        hal::drive_halt(2);
        
        // Capture current position as peak
        g_peak_tilt_deg = acquire::enc_to_deg(hal::drive_get_position(1));
        g_peak_pan_deg = acquire::enc_to_deg(hal::drive_get_position(2));
        
        g_first_spiral_executed = true;
        g_log_active = false;
        g_global_max_power = g_current_power;
        
        // Go directly to Figure-8 tracking
        g_wait_timer_start = millis();
        g_wait_active = true;
    }
    
    // If not detected, continue moving through spiral
    if (!g_wait_active && g_current_scan_point < g_path_points_count) {
        // Check if previous move is done
        if (hal::drive_is_done(1) && hal::drive_is_done(2)) {
            // Move to next point
            acquire::PathPoint& pt = g_path_buffer[g_current_scan_point];
            int32_t tilt_enc = acquire::deg_to_enc(pt.tilt_deg);
            int32_t pan_enc = acquire::deg_to_enc(pt.pan_deg);
            
            hal::drive_move_absolute(1, tilt_enc, g_axis1_params);
            hal::drive_move_absolute(2, pan_enc, g_axis2_params);
            
            g_current_scan_point++;
        }
    }
    
    // Check if scan complete
    if (g_current_scan_point >= g_path_points_count && 
        hal::drive_is_done(1) && hal::drive_is_done(2)) {
        ESP_LOGI(TAG, "Spiral complete, no signal found above threshold");
        g_first_spiral_executed = true;
        g_log_active = false;
        g_wait_active = true;
        g_wait_timer_start = millis();
    }
    
    // Transition after wait
    if (g_wait_active && timer_elapsed(g_wait_timer_start, STATE_WAIT_TIME_MS)) {
        g_wait_active = false;
        if (g_current_power >= g_min_power_threshold) {
            g_start_fig8 = true;
            g_state = acquire::State::FIGURE8_TRACKING;
        } else {
            // Analyze log for best position even if below threshold
            g_state = acquire::State::FIND_PEAK;
        }
    }
}

void state_find_peak() {
    static bool analysis_done = false;
    
    if (!analysis_done) {
        ESP_LOGI(TAG, "State 5: Analyzing %d samples for peak", g_log_index);
        
        float max_power = peak::find_max(g_power_history, g_log_index,
                                          g_peak_pan_deg, g_peak_tilt_deg);
        
        if (max_power > 0) {
            ESP_LOGI(TAG, "Best position found: (%.3f, %.3f) at %.2f µW",
                     g_peak_pan_deg, g_peak_tilt_deg, max_power);
        }
        
        analysis_done = true;
        g_wait_timer_start = millis();
    }
    
    if (timer_elapsed(g_wait_timer_start, STATE_WAIT_TIME_MS)) {
        analysis_done = false;
        g_state = acquire::State::MOVE_TO_PEAK;
    }
}

void state_move_to_peak() {
    int32_t target_tilt_enc = acquire::deg_to_enc(g_peak_tilt_deg);
    int32_t target_pan_enc = acquire::deg_to_enc(g_peak_pan_deg);
    
    // Calculate position errors
    int32_t tilt_error = abs(hal::drive_get_position(1) - target_tilt_enc);
    int32_t pan_error = abs(hal::drive_get_position(2) - target_pan_enc);
    
    // Send move command if needed
    if (!g_drive_peak_move_sent || tilt_error > ENCODER_TOLERANCE || pan_error > ENCODER_TOLERANCE) {
        hal::drive_move_absolute(1, target_tilt_enc, g_axis1_params);
        hal::drive_move_absolute(2, target_pan_enc, g_axis2_params);
        g_drive_peak_move_sent = true;
    }
    
    // Check completion
    if (hal::drive_is_done(1) && hal::drive_is_done(2)) {
        g_drive_peak_move_sent = false;
        g_power_settle_start = millis();
    }
    
    // Wait for power to settle
    if (!g_drive_peak_move_sent && timer_elapsed(g_power_settle_start, POWER_SETTLE_TIME_MS)) {
        if (tilt_error <= ENCODER_TOLERANCE && pan_error <= ENCODER_TOLERANCE) {
            g_wait_timer_start = millis();
            g_wait_active = true;
        }
    }
    
    if (g_wait_active && timer_elapsed(g_wait_timer_start, STATE_WAIT_TIME_MS)) {
        g_wait_active = false;
        g_log_active = true;
        g_log_index = 0;
        g_start_fig8 = true;
        g_state = acquire::State::FIGURE8_TRACKING;
    }
}

void state_figure8_tracking() {
    // Read power
    g_current_power = hal::power_sensor_read();
    
    // Check for signal loss
    static uint32_t loss_timer_start = 0;
    static bool loss_timer_active = false;
    
    if (g_current_power <= 30.0f) {
        if (!loss_timer_active) {
            loss_timer_start = millis();
            loss_timer_active = true;
        }
    } else {
        loss_timer_active = false;
    }
    
    // Signal lost for too long - restart scan
    if (loss_timer_active && timer_elapsed(loss_timer_start, SIGNAL_LOSS_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Signal lost! Restarting scan.");
        g_done_fig8 = true;
    }
    
    // Execute Figure-8 pattern
    if (g_start_fig8 && !g_done_fig8) {
        if (!figure8::g_scanner.initialized) {
            figure8::g_scanner.start(g_peak_pan_deg, g_peak_tilt_deg);
        }
        
        double next_pan, next_tilt;
        figure8::g_scanner.get_next_position(next_pan, next_tilt);
        
        // Update tracking with current reading
        double current_pan = acquire::enc_to_deg(hal::drive_get_position(2));
        double current_tilt = acquire::enc_to_deg(hal::drive_get_position(1));
        figure8::g_scanner.update(g_current_power, current_pan, current_tilt);
        
        // Track global max
        if (g_current_power > g_global_max_power) {
            g_global_max_power = g_current_power;
        }
        
        // Move to next position
        if (hal::drive_is_done(1) && hal::drive_is_done(2)) {
            hal::drive_move_absolute(1, acquire::deg_to_enc(next_tilt), g_axis1_params);
            hal::drive_move_absolute(2, acquire::deg_to_enc(next_pan), g_axis2_params);
        }
        
        // Log position
        if (g_log_active && g_log_index < MAX_PATH_POINTS) {
            g_power_history[g_log_index].pan_deg = current_pan;
            g_power_history[g_log_index].tilt_deg = current_tilt;
            g_power_history[g_log_index].power_uw = g_current_power;
            g_power_history[g_log_index].timestamp_ms = millis();
            g_log_index++;
        }
    }
    
    // Handle done/restart
    if (g_done_fig8) {
        g_peak_pan_deg = figure8::g_scanner.best_pan;
        g_peak_tilt_deg = figure8::g_scanner.best_tilt;
        
        figure8::g_scanner.initialized = false;
        g_start_fig8 = false;
        g_done_fig8 = false;
        g_first_spiral_executed = true;
        g_generate_sent = false;
        g_wait_timer_start = millis();
        g_wait_active = true;
    }
    
    if (g_wait_active && timer_elapsed(g_wait_timer_start, STATE_WAIT_TIME_MS)) {
        g_wait_active = false;
        // Go back to generate spiral (re-acquire)
        g_state = acquire::State::GENERATE_SPIRAL;
    }
}

void state_error() {
    static bool error_logged = false;
    
    if (!error_logged) {
        ESP_LOGE(TAG, "State 10: ERROR - System halted");
        hal::drive_halt(1);
        hal::drive_halt(2);
        error_logged = true;
    }
    
    // Wait for reset command
}

} // anonymous namespace

// ============================================================================
// Public API Implementation
// ============================================================================

extern "C" {

int acquire_init(void)
{
    ESP_LOGI(TAG, "Initializing acquisition system");
    
    // Initialize hardware
    if (!hal::power_sensor_init()) {
        ESP_LOGE(TAG, "Power sensor init failed");
        return -1;
    }
    
    if (!hal::drives_init()) {
        ESP_LOGE(TAG, "Drives init failed");
        return -2;
    }
    
    // Reset state
    g_state = acquire::State::INIT_MOTORS;
    g_first_scan = true;
    g_first_spiral_executed = false;
    
    ESP_LOGI(TAG, "Acquisition system initialized");
    return 0;
}

int acquire_start(void)
{
    ESP_LOGI(TAG, "Starting acquisition");
    
    if (g_first_scan) {
        g_state = acquire::State::INIT_MOTORS;
        g_first_scan = false;
    }
    
    return 0;
}

void acquire_stop(void)
{
    ESP_LOGI(TAG, "Stopping acquisition");
    hal::drive_halt(1);
    hal::drive_halt(2);
    g_log_active = false;
}

void acquire_update(void)
{
    switch (g_state) {
        case acquire::State::INIT_MOTORS:
            state_init_motors();
            break;
        case acquire::State::INDEX_MOTORS:
            state_index_motors();
            break;
        case acquire::State::MOVE_TO_START:
            state_move_to_start();
            break;
        case acquire::State::GENERATE_SPIRAL:
            state_generate_spiral();
            break;
        case acquire::State::EXECUTE_SPIRAL:
            state_execute_spiral();
            break;
        case acquire::State::FIND_PEAK:
            state_find_peak();
            break;
        case acquire::State::MOVE_TO_PEAK:
            state_move_to_peak();
            break;
        case acquire::State::FIGURE8_TRACKING:
            state_figure8_tracking();
            break;
        case acquire::State::HOLD:
            // Hold at current position - do nothing
            break;
        case acquire::State::ERROR:
            state_error();
            break;
    }
}

int acquire_get_state(void)
{
    return static_cast<int>(g_state);
}

float acquire_get_power_uw(void)
{
    return g_current_power;
}

double acquire_get_pan_deg(void)
{
    return acquire::enc_to_deg(hal::drive_get_position(2));
}

double acquire_get_tilt_deg(void)
{
    return acquire::enc_to_deg(hal::drive_get_position(1));
}

bool acquire_is_locked(void)
{
    return g_state == acquire::State::FIGURE8_TRACKING && g_current_power >= g_min_power_threshold;
}

void acquire_set_start_position(double pan_deg, double tilt_deg)
{
    g_start_pan_deg = pan_deg;
    g_start_tilt_deg = tilt_deg;
    ESP_LOGI(TAG, "Start position set to (%.2f, %.2f)", pan_deg, tilt_deg);
}

void acquire_set_power_threshold(float threshold_uw)
{
    g_min_power_threshold = threshold_uw;
    ESP_LOGI(TAG, "Power threshold set to %.2f µW", threshold_uw);
}

void acquire_halt(void)
{
    ESP_LOGW(TAG, "Emergency halt!");
    hal::drive_halt(1);
    hal::drive_halt(2);
}

void acquire_reset(void)
{
    ESP_LOGI(TAG, "Resetting acquisition system");
    hal::drive_reset(1);
    hal::drive_reset(2);
    
    g_state = acquire::State::INIT_MOTORS;
    g_first_scan = true;
    g_first_spiral_executed = false;
    g_drive1_move_sent = false;
    g_drive2_move_sent = false;
    g_generate_sent = false;
    g_drive_peak_move_sent = false;
    g_start_fig8 = false;
    g_done_fig8 = false;
    g_log_active = false;
    g_wait_active = false;
    
    figure8::g_scanner.initialized = false;
}

} // extern "C"
