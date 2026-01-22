# PDO Implementation Summary

## Files Created

### 1. **XeryonDrive_formatted.xml** (`docs/`)
- Properly indented XML with clear documentation of PDO structure
- **RxPdo (0x1600)**: PLC→Drive control commands (20 bytes total)
  - Command field (4 bytes) - identifies operation
  - Target position (4 bytes) - destination in encoder counts
  - Speed (4 bytes) - motion velocity
  - Acceleration (2 bytes) - ramp rate
  - Deceleration (2 bytes) - ramp rate
  - Execute strobe (1 byte) - latch to trigger command
  - Padding (3 bytes)

- **TxPdo (0x1A00)**: Drive→PLC feedback (9 bytes total)
  - Actual position (4 bytes) - current encoder position
  - Status flags (4 bytes) - packed bits for 24 status conditions
  - Slot ID (1 byte)

### 2. **xeryon_pdo.h** (`main/`)
C struct definitions for type-safe PDO access:

```cpp
typedef struct {
    int32_t  command;          // Command ID (XERYON_CMD_*)
    int32_t  target_position;  // Target position in encoder counts
    uint32_t speed;            // Motion speed
    uint16_t acceleration;     // Acceleration ramp
    uint16_t deceleration;     // Deceleration ramp
    uint8_t  execute;          // Strobe bit
} xeryon_rx_pdo_t;

typedef struct {
    int32_t actual_position;   // Current position
    // 24 individual status bits for:
    // - Amplifiers enabled, end stops, thermal protection
    // - Motor state, closed loop, encoder status
    // - Position reached, error conditions
    // - Limits, timeouts, failures
    uint8_t slot;              // Channel ID
} xeryon_tx_pdo_t;
```

**Command Codes Defined:**
- `XERYON_CMD_DPOS` (0x01) - Absolute move
- `XERYON_CMD_INDEX` (0x02) - Home/index
- `XERYON_CMD_ENABLE` (0x03) - Enable motor
- `XERYON_CMD_DISABLE` (0x04) - Disable motor
- `XERYON_CMD_STOP` (0x05) - Controlled stop
- `XERYON_CMD_HALT` (0x06) - Emergency halt
- `XERYON_CMD_RESET` (0x07) - Reset/clear errors

Helper macros for casting SOEM buffers to PDO structs.

---

## Changes to motor_control.cpp

All TODO comments replaced with actual implementations:

### Motor Enable/Disable
```cpp
motor_enable(drive_id_t drive)
  ↓
Writes XERYON_CMD_ENABLE to RxPdo output buffer
Sets execute strobe bit
Updates drive state to DRIVE_STATUS_IDLE
```

### Motor Homing/Indexing
```cpp
motor_index(drive_id_t drive, const axis_params_t *params)
  ↓
Writes XERYON_CMD_INDEX to RxPdo
Includes speed, acceleration, deceleration parameters
Sets execute strobe
Updates drive state to DRIVE_STATUS_BUSY
```

### Absolute Position Move
```cpp
motor_move_absolute_enc(drive_id_t drive, int32_t position, const axis_params_t *params)
  ↓
Writes XERYON_CMD_DPOS to RxPdo
Sets target position, speed, accel/decel
Sets execute strobe to trigger motion
Updates drive state to DRIVE_STATUS_BUSY
```

### Motion Control (Halt/Stop/Reset)
```cpp
motor_halt(drive_id_t drive)     → Writes XERYON_CMD_HALT
motor_stop(drive_id_t drive)     → Writes XERYON_CMD_STOP
motor_reset(drive_id_t drive)    → Writes XERYON_CMD_RESET
motor_reset_encoder(drive_id_t)  → Clears position tracking
```

### Status Reading
```cpp
motor_get_position_enc(drive_id_t drive)
  ↓
Reads actual_position from TxPdo input buffer
Returns encoder counts
```

### Cyclic Processing
```cpp
motor_process(void)
  ↓
Sends/receives EtherCAT process data via ecx_send/receive_processdata()
For each drive:
  - Reads actual position from TxPdo
  - Checks position_reached flag → updates done status
  - Monitors error flags (safety_timeout, position_fail, error_limit)
  - Checks thermal protection
  - Validates encoder state for homing
```

---

## Key Implementation Details

### Slave Addressing
- Slave index = `drive_id + 1` (SOEM slave 0 is gateway, actual drives start at 1)
- Slave count checked before each PDO access

### Output PDO Pattern (RxPdo Write)
```cpp
xeryon_rx_pdo_t *rx = RX_PDO_PTR(slavelist[slave_idx].outputs);
rx->command = XERYON_CMD_*;
rx->execute = 1;  // Strobe to trigger
```

### Input PDO Pattern (TxPdo Read)
```cpp
xeryon_tx_pdo_t *tx = TX_PDO_PTR(slavelist[slave_idx].inputs);
if (tx->position_reached) { /* motion complete */ }
if (tx->encoder_valid) { /* can rely on position */ }
```

### Error Handling
- Per-drive error tracking with `has_error` flag
- Automatic status update to `DRIVE_STATUS_ERROR` on:
  - Safety timeout
  - Position fail
  - Error limit exceeded
  - Thermal protection triggered
- Logging of error conditions with drive index and flags

### State Machine Integration
- `DRIVE_STATUS_BUSY` during motion
- `DRIVE_STATUS_IDLE` when at rest or motion complete
- `DRIVE_STATUS_ERROR` on fault conditions

---

## Next Steps (Validation)

1. **Test EtherCAT communication**: Use SOEM's slaveinfo utility to verify PDO mapping
   ```bash
   ./slaveinfo eth0
   ```

2. **Verify struct sizes**: Ensure PDO structs match actual byte layout
   - RxPdo should be 20 bytes (though 17 used)
   - TxPdo should be 9 bytes

3. **Test drive enable sequence**:
   - Call `motor_control_init()`
   - Call `motor_enable(DRIVE_TILT)` and `motor_enable(DRIVE_PAN)`
   - Verify amplifier enable signals on drives

4. **Test homing**: 
   - Call `motor_index()` with params
   - Monitor `motor_get_status()` until `DRIVE_STATUS_IDLE`
   - Check `encoder_valid` flag is set

5. **Test motion**:
   - Call `motor_move_absolute_enc()` with target position
   - Poll `motor_is_done()` in cyclic loop calling `motor_process()`
   - Verify actual position converges to target

---

## Notes

- All PDO offsets derived from XeryonDrive.xml RxPdo/TxPdo definitions
- Execute strobe (byte 16 in RxPdo) pulses high each cycle to trigger commands
- Status bits are packed into 3 bytes (bytes 4-6 of TxPdo); struct handles bitfield access
- Error recovery via `motor_reset()` clears fault flags and resets error state
- Position tracking maintains 32-bit signed int for both linear (mm) and rotary (degrees) via encoder conversion
