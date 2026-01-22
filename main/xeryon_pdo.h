/**
 * @file xeryon_pdo.h
 * @brief Xeryon EtherCAT PDO structure definitions
 * 
 * PDO byte layout derived from XeryonDrive.xml:
 * - RxPdo (0x1600): PLC → Drive control (20 bytes)
 * - TxPdo (0x1A00): Drive → PLC feedback (9 bytes)
 */

#ifndef XERYON_PDO_H
#define XERYON_PDO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * RxPdo (0x1600) - Output PDO: PLC to Drive
 * Control commands and motion parameters
 * Total: 20 bytes (but only 17 used, 3 padding)
 * ======================================================================== */

/** Rx PDO: Command byte field offsets within output buffer */
typedef struct {
    int32_t  command;          /**< Offset 0-3: Control command/word (UDINT) */
    int32_t  target_position;  /**< Offset 4-7: Target position in encoder counts (DINT) */
    uint32_t speed;            /**< Offset 8-11: Speed (UDINT) */
    uint16_t acceleration;     /**< Offset 12-13: Acceleration ramp (UINT) */
    uint16_t deceleration;     /**< Offset 14-15: Deceleration ramp (UINT) */
    uint8_t  execute;          /**< Offset 16: Execute strobe/latch bit (BYTE) */
    /* Bytes 17-19: Padding (reserved) */
} xeryon_rx_pdo_t;

/* Command definitions for RxPdo command field */
#define XERYON_CMD_DPOS     0x01  /**< Absolute position move command */
#define XERYON_CMD_INDEX    0x02  /**< Home/index command */
#define XERYON_CMD_ENABLE   0x03  /**< Enable motor command */
#define XERYON_CMD_DISABLE  0x04  /**< Disable motor command */
#define XERYON_CMD_STOP     0x05  /**< Controlled stop command */
#define XERYON_CMD_HALT     0x06  /**< Emergency halt/quickstop */
#define XERYON_CMD_RESET    0x07  /**< Reset drive/errors */

/* ========================================================================
 * TxPdo (0x1A00) - Input PDO: Drive to PLC
 * Status feedback and actual position
 * Total: 9 bytes
 * ======================================================================== */

/**
 * TxPdo Status Byte Bits
 * Byte 4 contains bits 0-7, Byte 5 contains bits 8-15, Byte 6 contains bits 16-23
 */
typedef struct {
    int32_t actual_position;        /**< Offset 0-3: Actual position in encoder counts (DINT) */
    
    /* Byte 4 - Status flags 0-7 */
    uint8_t amplifiers_enabled : 1; /**< Bit 0: Amplifiers powered */
    uint8_t end_stop : 1;           /**< Bit 1: Either end stop active */
    uint8_t thermal_protection_1 : 1; /**< Bit 2: Thermal limit 1 triggered */
    uint8_t thermal_protection_2 : 1; /**< Bit 3: Thermal limit 2 triggered */
    uint8_t force_zero : 1;         /**< Bit 4: Force zero active */
    uint8_t motor_on : 1;           /**< Bit 5: Motor actively running */
    uint8_t closed_loop : 1;        /**< Bit 6: Closed loop mode active */
    uint8_t encoder_index : 1;      /**< Bit 7: Index pulse found */
    
    /* Byte 5 - Status flags 8-15 */
    uint8_t encoder_valid : 1;      /**< Bit 8: Encoder position valid */
    uint8_t searching_index : 1;    /**< Bit 9: Currently searching for index */
    uint8_t position_reached : 1;   /**< Bit 10: Target position reached (motion complete) */
    uint8_t error_compensation : 1; /**< Bit 11: Error compensation active */
    uint8_t encoder_error : 1;      /**< Bit 12: Encoder read error */
    uint8_t scanning : 1;           /**< Bit 13: Scanning mode active */
    uint8_t left_end_stop : 1;      /**< Bit 14: Left limit triggered */
    uint8_t right_end_stop : 1;     /**< Bit 15: Right limit triggered */
    
    /* Byte 6 - Status flags 16-23 */
    uint8_t error_limit : 1;        /**< Bit 16: Position error limit exceeded */
    uint8_t searching_frequency : 1; /**< Bit 17: Searching optimal frequency */
    uint8_t safety_timeout : 1;     /**< Bit 18: Safety timeout triggered */
    uint8_t execute_ack : 1;        /**< Bit 19: Execute command acknowledged */
    uint8_t emergency_stop : 1;     /**< Bit 20: Emergency stop active */
    uint8_t position_fail : 1;      /**< Bit 21: Position failed to reach target */
    uint8_t reserved_bit22 : 1;     /**< Bit 22: Reserved */
    uint8_t reserved_bit23 : 1;     /**< Bit 23: Reserved */
    
    uint8_t slot;                   /**< Offset 8: Slot/Channel ID */
} xeryon_tx_pdo_t;

/* ========================================================================
 * Helper macros for PDO access via SOEM
 * ======================================================================== */

/**
 * Cast output buffer to Rx PDO structure
 * Usage: xeryon_rx_pdo_t *rx = RX_PDO_PTR(context.slavelist[n].outputs);
 */
#define RX_PDO_PTR(outputs_ptr) ((xeryon_rx_pdo_t*)(outputs_ptr))

/**
 * Cast input buffer to Tx PDO structure
 * Usage: xeryon_tx_pdo_t *tx = TX_PDO_PTR(context.slavelist[n].inputs);
 */
#define TX_PDO_PTR(inputs_ptr) ((xeryon_tx_pdo_t*)(inputs_ptr))

#ifdef __cplusplus
}
#endif

#endif /* XERYON_PDO_H */
