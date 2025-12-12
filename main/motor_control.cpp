/**
 * @file motor_control.cpp
 * @brief Motor control implementation using SOEM (EtherCAT) for Xeryon drives
 * 
 * Implements pan/tilt motor control via EtherCAT communication.
 * Uses SOEM's context-based API (ecx_*).
 * 
 * This is a skeleton implementation - SOEM integration depends on your
 * specific Xeryon EtherCAT object dictionary configuration.
 */

#include "motor_control.h"
#include "acquire_config.h"
#include "esp_log.h"
#include <cmath>
#include <cstring>

// SOEM headers - uses context-based API (ecx_*)
extern "C" {
#include "soem/soem.h"
}

static const char *TAG = "motor_ctrl";

// =============================================================================
// SOEM Context and State
// =============================================================================

/** Size of the IO map buffer for PDO data */
#define EC_IOMAP_SIZE 4096

/** Per-drive state tracking */
typedef struct {
    bool enabled;
    bool done;
    bool has_error;
    int32_t actual_position;
    int32_t target_position;
    drive_status_t status;
} drive_state_t;

/** SOEM context and motor control state */
static struct {
    bool initialized;
    
    // SOEM context - the structure has embedded arrays, not pointers
    ecx_contextt context;
    
    // IO map for PDO data exchange
    char io_map[EC_IOMAP_SIZE];
    int expected_wkc;
    
    // Per-drive state
    drive_state_t drives[DRIVE_COUNT];
} s_motor_ctx = {};

// =============================================================================
// Conversion Functions
// =============================================================================

extern "C" {

/**
 * Convert degrees to encoder counts
 * Formula: enc = deg * (ENCODER_RESOLUTION / 360)
 */
int32_t deg_to_enc(double degrees)
{
    return static_cast<int32_t>(degrees * (static_cast<double>(ENCODER_RESOLUTION) / 360.0));
}

/**
 * Convert encoder counts to degrees
 */
double enc_to_deg(int32_t enc_counts)
{
    return static_cast<double>(enc_counts) * 360.0 / static_cast<double>(ENCODER_RESOLUTION);
}

// =============================================================================
// Initialization
// =============================================================================

esp_err_t motor_control_init(const char *eth_interface)
{
    if (s_motor_ctx.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SOEM on interface: %s", eth_interface);

    // Clear context
    memset(&s_motor_ctx.context, 0, sizeof(ecx_contextt));

    // Initialize SOEM
    if (ecx_init(&s_motor_ctx.context, eth_interface) == 0) {
        ESP_LOGE(TAG, "Failed to initialize SOEM on %s", eth_interface);
        return ESP_FAIL;
    }

    // Find and auto-configure slaves
    if (ecx_config_init(&s_motor_ctx.context) == 0) {
        ESP_LOGE(TAG, "No EtherCAT slaves found");
        ecx_close(&s_motor_ctx.context);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Found %d EtherCAT slave(s)", s_motor_ctx.context.slavecount);

    // Verify we have expected number of drives
    if (s_motor_ctx.context.slavecount < DRIVE_COUNT) {
        ESP_LOGW(TAG, "Expected %d drives, found %d", DRIVE_COUNT, s_motor_ctx.context.slavecount);
    }

    // Map IO
    ecx_config_map_group(&s_motor_ctx.context, &s_motor_ctx.io_map, 0);
    ecx_configdc(&s_motor_ctx.context);

    // Wait for all slaves to reach SAFE_OP state
    ecx_statecheck(&s_motor_ctx.context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    // Calculate expected working counter
    s_motor_ctx.expected_wkc = (s_motor_ctx.context.grouplist[0].outputsWKC * 2) + 
                               s_motor_ctx.context.grouplist[0].inputsWKC;

    // Request OP state for all slaves
    s_motor_ctx.context.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_send_processdata(&s_motor_ctx.context);
    ecx_receive_processdata(&s_motor_ctx.context, EC_TIMEOUTRET);
    ecx_writestate(&s_motor_ctx.context, 0);

    // Wait for OP state
    int chk = 200;
    do {
        ecx_send_processdata(&s_motor_ctx.context);
        ecx_receive_processdata(&s_motor_ctx.context, EC_TIMEOUTRET);
        ecx_statecheck(&s_motor_ctx.context, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (s_motor_ctx.context.slavelist[0].state != EC_STATE_OPERATIONAL));

    if (s_motor_ctx.context.slavelist[0].state != EC_STATE_OPERATIONAL) {
        ESP_LOGE(TAG, "Failed to reach OPERATIONAL state");
        ecx_close(&s_motor_ctx.context);
        return ESP_FAIL;
    }

    // Initialize drive states
    memset(s_motor_ctx.drives, 0, sizeof(s_motor_ctx.drives));
    for (int i = 0; i < DRIVE_COUNT; i++) {
        s_motor_ctx.drives[i].status = DRIVE_STATUS_DISABLED;
    }

    s_motor_ctx.initialized = true;
    ESP_LOGI(TAG, "Motor control initialized successfully");

    return ESP_OK;
}

// =============================================================================
// Drive Control
// =============================================================================

esp_err_t motor_enable(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    // TODO: Send enable command via SOEM
    // This depends on the Xeryon EtherCAT object dictionary
    // Typically involves writing to a controlword PDO
    
    ESP_LOGI(TAG, "Enabling drive %d", drive);
    s_motor_ctx.drives[drive].enabled = true;
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_IDLE;
    
    return ESP_OK;
}

esp_err_t motor_enable_all(void)
{
    esp_err_t err = motor_enable(DRIVE_TILT);
    if (err == ESP_OK) {
        err = motor_enable(DRIVE_PAN);
    }
    return err;
}

esp_err_t motor_disable(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disabling drive %d", drive);
    s_motor_ctx.drives[drive].enabled = false;
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_DISABLED;
    
    return ESP_OK;
}

esp_err_t motor_index(drive_id_t drive, const axis_params_t *params)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Indexing drive %d (speed=%ld, acc=%u, dec=%u)",
             drive, params->speed, params->acc, params->dec);

    // TODO: Send index/homing command via SOEM
    // This triggers the drive's internal homing sequence
    
    s_motor_ctx.drives[drive].done = false;
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_BUSY;
    
    return ESP_OK;
}

esp_err_t motor_index_all(const axis_params_t *tilt_params, const axis_params_t *pan_params)
{
    esp_err_t err = motor_index(DRIVE_TILT, tilt_params);
    if (err == ESP_OK) {
        err = motor_index(DRIVE_PAN, pan_params);
    }
    return err;
}

esp_err_t motor_move_absolute_enc(drive_id_t drive, int32_t position, const axis_params_t *params)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_motor_ctx.drives[drive].enabled) {
        ESP_LOGW(TAG, "Drive %d not enabled", drive);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "Move drive %d to %ld enc (speed=%ld)",
             drive, position, params->speed);

    // TODO: Send MoveAbsolute command via SOEM
    // Set target position in PDO, set controlword to start motion
    
    s_motor_ctx.drives[drive].target_position = position;
    s_motor_ctx.drives[drive].done = false;
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_BUSY;
    
    return ESP_OK;
}

esp_err_t motor_move_absolute_deg(drive_id_t drive, double degrees, const axis_params_t *params)
{
    int32_t enc_position = deg_to_enc(degrees);
    ESP_LOGD(TAG, "Move drive %d to %.3f deg (%ld enc)", drive, degrees, enc_position);
    return motor_move_absolute_enc(drive, enc_position, params);
}

esp_err_t motor_halt(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Halting drive %d", drive);
    
    // TODO: Send halt command via SOEM (immediate stop)
    
    s_motor_ctx.drives[drive].done = true;
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_IDLE;
    
    return ESP_OK;
}

esp_err_t motor_halt_all(void)
{
    motor_halt(DRIVE_TILT);
    motor_halt(DRIVE_PAN);
    return ESP_OK;
}

esp_err_t motor_stop(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Stopping drive %d", drive);
    
    // TODO: Send stop command via SOEM (controlled decel)
    
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_IDLE;
    
    return ESP_OK;
}

esp_err_t motor_reset(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resetting drive %d", drive);
    
    // TODO: Send reset command via SOEM
    
    s_motor_ctx.drives[drive].has_error = false;
    s_motor_ctx.drives[drive].status = DRIVE_STATUS_IDLE;
    
    return ESP_OK;
}

esp_err_t motor_reset_encoder(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resetting encoder on drive %d", drive);
    
    // TODO: Send encoder reset command via SOEM
    
    s_motor_ctx.drives[drive].actual_position = 0;
    
    return ESP_OK;
}

// =============================================================================
// Status Queries
// =============================================================================

int32_t motor_get_position_enc(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return 0;
    }
    
    // TODO: Read actual position from PDO input
    return s_motor_ctx.drives[drive].actual_position;
}

double motor_get_position_deg(drive_id_t drive)
{
    return enc_to_deg(motor_get_position_enc(drive));
}

drive_status_t motor_get_status(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return DRIVE_STATUS_ERROR;
    }
    return s_motor_ctx.drives[drive].status;
}

bool motor_is_done(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return false;
    }
    return s_motor_ctx.drives[drive].done;
}

bool motor_has_error(drive_id_t drive)
{
    if (!s_motor_ctx.initialized || drive >= DRIVE_COUNT) {
        return true;
    }
    return s_motor_ctx.drives[drive].has_error;
}

// =============================================================================
// Cyclic Processing
// =============================================================================

void motor_process(void)
{
    if (!s_motor_ctx.initialized) {
        return;
    }

    // Send and receive process data
    ecx_send_processdata(&s_motor_ctx.context);
    int wkc = ecx_receive_processdata(&s_motor_ctx.context, EC_TIMEOUTRET);
    
    if (wkc < s_motor_ctx.expected_wkc) {
        ESP_LOGW(TAG, "Working counter mismatch: %d < %d", wkc, s_motor_ctx.expected_wkc);
    }

    // TODO: Update drive states from input PDOs
    // - Read actual positions from context.slavelist[n].inputs
    // - Read statuswords
    // - Update done flags based on target reached
    // - Update error flags
    
    // Example pseudo-code for reading from slaves:
    // for (int i = 0; i < DRIVE_COUNT && i <= s_motor_ctx.context.slavecount; i++) {
    //     uint8_t *inputs = s_motor_ctx.context.slavelist[i+1].inputs;
    //     s_motor_ctx.drives[i].actual_position = *(int32_t*)(inputs + POSITION_OFFSET);
    //     uint16_t statusword = *(uint16_t*)(inputs + STATUSWORD_OFFSET);
    //     s_motor_ctx.drives[i].done = (statusword & TARGET_REACHED_BIT) != 0;
    //     s_motor_ctx.drives[i].has_error = (statusword & FAULT_BIT) != 0;
    // }
}

// =============================================================================
// Cleanup
// =============================================================================

void motor_control_deinit(void)
{
    if (s_motor_ctx.initialized) {
        // Request INIT state
        s_motor_ctx.context.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&s_motor_ctx.context, 0);
        
        // Close SOEM
        ecx_close(&s_motor_ctx.context);
        
        s_motor_ctx.initialized = false;
        ESP_LOGI(TAG, "Motor control de-initialized");
    }
}

} // extern "C"
