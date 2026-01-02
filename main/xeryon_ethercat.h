#ifndef XERYON_ETHERCAT_H
#define XERYON_ETHERCAT_H

/**
 * @file xeryon_ethercat.h
 * @brief Xeryon EtherCAT PDO definitions and structures
 * 
 * Defines the Process Data Object (PDO) layout for Xeryon motor drives.
 * Uses Xeryon's proprietary command/status protocol (NOT CiA 402).
 * 
 * Protocol uses string-based commands (INDX, SCAN, DPOS, etc.) rather
 * than standard CANopen controlwords.
 */

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// PDO Structures - Xeryon Proprietary Protocol
// =============================================================================

/**
 * @brief RxPDO - Output data sent TO the drive (master -> slave)
 * 
 * This is what the master writes to control the drive.
 * Total size: 17 bytes
 */
typedef struct __attribute__((packed)) {
    char     command[4];         /**< Command string: INDX, SCAN, DPOS, RSET, ENBL, HALT, STOP */
    int32_t  data_reg1;          /**< Data register 1: Target position or direction */
    uint32_t data_reg2;          /**< Data register 2: Speed (um/s, 0-1048575) */
    uint16_t data_reg3;          /**< Data register 3: Acceleration (m/s^2, 0-65535) */
    uint16_t data_reg4;          /**< Data register 4: Deceleration (m/s^2, 0-65535) */
    uint8_t  execute;            /**< Execute register: 0=prepare, 1=execute */
} xeryon_outputs_t;

/**
 * @brief TxPDO - Input data received FROM the drive (slave -> master)
 * 
 * This is what the drive sends back with status and position.
 * Total size: 8 bytes
 */
typedef struct __attribute__((packed)) {
    int32_t  actual_position;    /**< Actual position (26-bit signed: +/- 33554431) */
    uint8_t  status_bytes[3];    /**< Status (24 bits) - see XERYON_STATUS_* flags */
    uint8_t  slot_position;      /**< Slot position (8 bits) */
} xeryon_inputs_t;

/**
 * @brief Helper union for accessing 24-bit status as uint32_t
 */
typedef union {
    uint8_t  bytes[3];
    struct {
        uint32_t flags : 24;
        uint32_t reserved : 8;
    };
} xeryon_status_u;

/**
 * @brief Complete EtherCAT message structure
 * 
 * Contains both command (outputs) and status (inputs) in one packet.
 * Total size: 25 bytes (17 bytes output + 8 bytes input)
 */
typedef struct __attribute__((packed)) {
    xeryon_outputs_t outputs;    /**< Commands sent to drive */
    xeryon_inputs_t  inputs;     /**< Status received from drive */
} ethercat_msg_t;

// =============================================================================
// PDO Offset Constants
// =============================================================================

// Output (RxPDO) offsets - Total: 17 bytes
#define XERYON_OUT_COMMAND_OFFSET           0   /**< 4 bytes: Command string */
#define XERYON_OUT_DATA_REG1_OFFSET         4   /**< 4 bytes: Target pos/direction */
#define XERYON_OUT_DATA_REG2_OFFSET         8   /**< 4 bytes: Speed */
#define XERYON_OUT_DATA_REG3_OFFSET         12  /**< 2 bytes: Acceleration */
#define XERYON_OUT_DATA_REG4_OFFSET         14  /**< 2 bytes: Deceleration */
#define XERYON_OUT_EXECUTE_OFFSET           16  /**< 1 byte: Execute flag */

// Input (TxPDO) offsets - Total: 8 bytes
#define XERYON_IN_ACTUAL_POSITION_OFFSET    0   /**< 4 bytes: Actual position */
#define XERYON_IN_STATUS_OFFSET             4   /**< 3 bytes: Status bits */
#define XERYON_IN_SLOT_POSITION_OFFSET      7   /**< 1 byte: Slot position */

// =============================================================================
// Command Register Strings
// =============================================================================

#define XERYON_CMD_INDX     "INDX"      /**< Find home point in the rig */
#define XERYON_CMD_SCAN     "SCAN"      /**< Move continuously at fixed speed */
#define XERYON_CMD_DPOS     "DPOS"      /**< Move to target position */
#define XERYON_CMD_RSET     "RSET"      /**< Reset the controller fully */
#define XERYON_CMD_ENBL     "ENBL"      /**< Enable/disable motor signals */
#define XERYON_CMD_HALT     "HALT"      /**< Switch the actuator off */
#define XERYON_CMD_STOP     "STOP"      /**< HALT with states and flags applied */

// Execute register values
#define XERYON_EXECUTE_PREPARE          0           /**< Prepare command (don't execute) */
#define XERYON_EXECUTE_RUN              1           /**< Execute the command */

// =============================================================================
// Status Bits (24-bit status word)
// =============================================================================

#define XERYON_STATUS_ENABLED           (1UL << 0)  /**< Bit 0: Enabled */
#define XERYON_STATUS_BOUNDING_STOP     (1UL << 1)  /**< Bit 1: Bounding stop */
#define XERYON_STATUS_AMP1_HOT          (1UL << 2)  /**< Bit 2: Amplifier 1 hot */
#define XERYON_STATUS_AMP2_HOT          (1UL << 3)  /**< Bit 3: Amplifier 2 hot */
#define XERYON_STATUS_FORCE_ZERO        (1UL << 4)  /**< Bit 4: Force zero */
#define XERYON_STATUS_MOTOR_ON          (1UL << 5)  /**< Bit 5: Motor on */
#define XERYON_STATUS_CLOSE_LOOP        (1UL << 6)  /**< Bit 6: Close loop control */
#define XERYON_STATUS_HOMED             (1UL << 7)  /**< Bit 7: Homed */
#define XERYON_STATUS_LOCATED_HOME      (1UL << 8)  /**< Bit 8: Located home */
#define XERYON_STATUS_HOMING            (1UL << 9)  /**< Bit 9: Homing in progress */
#define XERYON_STATUS_TARGET_ACQUIRED   (1UL << 10) /**< Bit 10: Target acquired */
#define XERYON_STATUS_ERROR_COMP        (1UL << 11) /**< Bit 11: Error compensation */
#define XERYON_STATUS_ENCODER_ERROR     (1UL << 12) /**< Bit 12: Encoder error */
#define XERYON_STATUS_SCANNING          (1UL << 13) /**< Bit 13: Scanning */
#define XERYON_STATUS_PAST_LOW_END      (1UL << 14) /**< Bit 14: Past low end */
#define XERYON_STATUS_PAST_HIGH_END     (1UL << 15) /**< Bit 15: Past high end */
#define XERYON_STATUS_MAX_ERRORS        (1UL << 16) /**< Bit 16: Max errors */
#define XERYON_STATUS_OPTIMAL_FREQ      (1UL << 17) /**< Bit 17: Optimal search freq */
#define XERYON_STATUS_TIMEOUT           (1UL << 18) /**< Bit 18: Timeout */
#define XERYON_STATUS_CMD_RECEIVED      (1UL << 19) /**< Bit 19: Command received */
#define XERYON_STATUS_EMERGENCY_STOP    (1UL << 20) /**< Bit 20: Emergency stop */
#define XERYON_STATUS_TIMEOUT_ACQUIRE   (1UL << 21) /**< Bit 21: Timeout before acquire */
// Bits 22-23: N/A

// =============================================================================
// Motion Profile Parameters
// =============================================================================

// Position limits (26-bit signed)
#define XERYON_POSITION_MIN             (-33554431) /**< Minimum position value */
#define XERYON_POSITION_MAX             (33554431)  /**< Maximum position value */

// Speed limits (20-bit, units: um/s)
#define XERYON_SPEED_MIN                0           /**< Minimum speed */
#define XERYON_SPEED_MAX                1048575     /**< Maximum speed (um/s) */

// Acceleration/Deceleration limits (16-bit, units: m/s^2)
#define XERYON_ACCEL_MIN                0           /**< Minimum acceleration */
#define XERYON_ACCEL_MAX                65535       /**< Maximum acceleration */
#define XERYON_ACCEL_DEFAULT            65000       /**< Default acceleration */

#define XERYON_DECEL_MIN                0           /**< Minimum deceleration */
#define XERYON_DECEL_MAX                65535       /**< Maximum deceleration */
#define XERYON_DECEL_DEFAULT            65000       /**< Default deceleration */

// Direction values for INDX and SCAN commands (data_reg1)
#define XERYON_DIRECTION_NEGATIVE       (-1)        /**< Move in negative direction */
#define XERYON_DIRECTION_POSITIVE       (1)         /**< Move in positive direction */

// =============================================================================
// Helper Macros and Functions
// =============================================================================

/**
 * @brief Extract 24-bit status value from status bytes
 */
static inline uint32_t xeryon_get_status(const uint8_t status_bytes[3])
{
    return (uint32_t)status_bytes[0] | 
           ((uint32_t)status_bytes[1] << 8) | 
           ((uint32_t)status_bytes[2] << 16);
}

/**
 * @brief Check if drive is enabled
 */
#define XERYON_IS_ENABLED(status) \
    ((status) & XERYON_STATUS_ENABLED)

/**
 * @brief Check if drive has reached target
 */
#define XERYON_IS_TARGET_ACQUIRED(status) \
    ((status) & XERYON_STATUS_TARGET_ACQUIRED)

/**
 * @brief Check if homing is complete
 */
#define XERYON_IS_HOMED(status) \
    ((status) & XERYON_STATUS_HOMED)

/**
 * @brief Check if homing is in progress
 */
#define XERYON_IS_HOMING(status) \
    ((status) & XERYON_STATUS_HOMING)

/**
 * @brief Check if drive is scanning
 */
#define XERYON_IS_SCANNING(status) \
    ((status) & XERYON_STATUS_SCANNING)

/**
 * @brief Check if motor is on
 */
#define XERYON_IS_MOTOR_ON(status) \
    ((status) & XERYON_STATUS_MOTOR_ON)

/**
 * @brief Check for any error conditions
 */
#define XERYON_HAS_ERROR(status) \
    ((status) & (XERYON_STATUS_ENCODER_ERROR | \
                 XERYON_STATUS_MAX_ERRORS | \
                 XERYON_STATUS_TIMEOUT | \
                 XERYON_STATUS_EMERGENCY_STOP | \
                 XERYON_STATUS_TIMEOUT_ACQUIRE))

/**
 * @brief Set command string in output PDO
 */
static inline void xeryon_set_command(char dest[4], const char *cmd)
{
    memcpy(dest, cmd, 4);
}

#ifdef __cplusplus
}
#endif

#endif // XERYON_ETHERCAT_H
