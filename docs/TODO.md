# SOEM POE32 - Implementation TODO List

## EtherCAT Packet Communication - Outstanding Tasks

This document tracks the implementation tasks needed to complete the EtherCAT communication layer for the Xeryon motor drives.

### Overview
The motor control module (`main/motor_control.cpp`) has a functional SOEM framework but requires completion of PDO (Process Data Object) read/write operations. These TODOs involve:
- Writing command data to **output PDOs** (`context.slavelist[n].outputs`)
- Reading status data from **input PDOs** (`context.slavelist[n].inputs`)

All implementations depend on the Xeryon EtherCAT object dictionary configuration.

---

## 1. Motor Enable Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_enable()` (Line ~184-194)  
**Priority:** High

### Task
Implement enable command via SOEM controlword PDO.

### Details
- Write to `s_motor_ctx.context.slavelist[drive+1].outputs`
- Typically involves setting specific bits in a controlword (e.g., bits 0-3 for state machine transitions)
- Standard CANopen/CiA 402 pattern: 0x06 → 0x07 → 0x0F for enable sequence
- Must determine correct PDO offset for controlword in Xeryon object dictionary

### Example Pattern
```cpp
uint8_t *outputs = s_motor_ctx.context.slavelist[drive+1].outputs;
uint16_t *controlword = (uint16_t*)(outputs + CONTROLWORD_OFFSET);
*controlword = 0x0F; // Enable operation
```

---

## 2. Index/Homing Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_index()` (Line ~213-223)  
**Priority:** High

### Task
Send homing command to trigger internal homing sequence.

### Details
- Write homing parameters to output PDOs:
  - Homing method (object 0x6098)
  - Homing speed (object 0x6099)
  - Homing acceleration (object 0x609A)
- Set controlword bit to start homing (typically bit 4)
- May require SDO writes for configuration before PDO command

### Notes
- Homing is critical for establishing absolute position reference
- Different homing methods available (limit switch, index pulse, current limit, etc.)

---

## 3. Absolute Move Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_move_absolute_enc()` (Line ~237-254)  
**Priority:** High

### Task
Write target position and motion parameters to PDOs, trigger motion.

### Details
- Write to output PDOs:
  - Target position (32-bit encoder counts)
  - Speed setpoint
  - Acceleration/deceleration parameters
- Set controlword bits for "new setpoint" and motion start
- Update `target_position` in local state

### Example Pattern
```cpp
uint8_t *outputs = s_motor_ctx.context.slavelist[drive+1].outputs;
*(int32_t*)(outputs + TARGET_POSITION_OFFSET) = position;
*(uint32_t*)(outputs + SPEED_OFFSET) = params->speed;
uint16_t *controlword = (uint16_t*)(outputs + CONTROLWORD_OFFSET);
*controlword |= 0x10; // New setpoint bit
```

---

## 4. Halt Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_halt()` (Line ~264-275)  
**Priority:** Medium

### Task
Send immediate stop command (quickstop).

### Details
- Set halt bit in controlword (typically bit 8)
- Or trigger quickstop state (controlword = 0x02)
- Should stop motion immediately with quickstop deceleration ramp

---

## 5. Stop Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_stop()` (Line ~284-294)  
**Priority:** Medium

### Task
Send controlled stop command (normal deceleration).

### Details
- Clear "enable operation" or set specific stop bits in controlword
- Uses normal deceleration profile (vs. halt's quickstop)
- Difference from halt: controlled vs. emergency stop

---

## 6. Reset Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_reset()` (Line ~300-311)  
**Priority:** Medium

### Task
Send fault reset command to clear error state.

### Details
- Set fault reset bit in controlword (typically bit 7)
- Required to recover from error/fault conditions
- May need to cycle: set bit, wait, clear bit

---

## 7. Encoder Reset Command
**File:** `main/motor_control.cpp`  
**Function:** `motor_reset_encoder()` (Line ~313-323)  
**Priority:** Low

### Task
Reset encoder position to zero.

### Details
- May require SDO write to specific object
- Or special controlword command
- Check Xeryon documentation for supported method

---

## 8. Status Reading (Cyclic)
**File:** `main/motor_control.cpp`  
**Function:** `motor_process()` (Line ~376-413)  
**Priority:** Critical

### Task
Read drive status from input PDOs in cyclic processing loop.

### Details
Must read and update for each drive:
- **Actual position** (int32_t) - current encoder value
- **Statusword** (uint16_t) - drive state and flags
  - Bit 10: Target reached
  - Bit 3: Fault
  - Bits 0-6: State machine status
- Update local state:
  - `actual_position`
  - `done` flag (target reached)
  - `has_error` flag (fault bit)
  - `status` enum based on statusword

### Example Implementation
```cpp
for (int i = 0; i < DRIVE_COUNT && i < s_motor_ctx.context.slavecount; i++) {
    uint8_t *inputs = s_motor_ctx.context.slavelist[i+1].inputs;
    
    // Read position
    s_motor_ctx.drives[i].actual_position = *(int32_t*)(inputs + POSITION_OFFSET);
    
    // Read statusword
    uint16_t statusword = *(uint16_t*)(inputs + STATUSWORD_OFFSET);
    
    // Update flags
    s_motor_ctx.drives[i].done = (statusword & (1 << 10)) != 0; // Target reached
    s_motor_ctx.drives[i].has_error = (statusword & (1 << 3)) != 0; // Fault
    
    // Update status enum
    if (statusword & (1 << 3)) {
        s_motor_ctx.drives[i].status = DRIVE_STATUS_ERROR;
    } else if (statusword & (1 << 10)) {
        s_motor_ctx.drives[i].status = DRIVE_STATUS_IDLE;
    } else {
        s_motor_ctx.drives[i].status = DRIVE_STATUS_BUSY;
    }
}
```

---

## Required Information

To complete these tasks, you need:

1. **Xeryon EtherCAT Object Dictionary** - Defines PDO mapping
2. **PDO Offsets** - Where in the input/output buffers each field is located:
   - `CONTROLWORD_OFFSET` - Usually 0
   - `TARGET_POSITION_OFFSET` - Usually 2 or 4
   - `SPEED_OFFSET`
   - `POSITION_OFFSET` (input)
   - `STATUSWORD_OFFSET` (input) - Usually 0
3. **Controlword Bit Definitions** - Xeryon-specific or CiA 402 standard
4. **Statusword Bit Definitions** - Standard or custom bits

### How to Discover PDO Mapping

Use the `slaveinfo` tool from SOEM to inspect the slave configuration:
```bash
cd SOEM/build
sudo ./test/linux/slaveinfo/slaveinfo eth0
```

This will show:
- PDO mapping configuration
- Object dictionary entries
- Input/output sizes and offsets

---

## Testing Strategy

1. **Enable/Disable** - Verify state transitions
2. **Position Reading** - Confirm actual position updates in `motor_process()`
3. **Homing** - Test index operation completes successfully
4. **Motion** - Test absolute moves with target reached detection
5. **Error Handling** - Test fault conditions and reset
6. **Stop Commands** - Verify halt vs. stop behavior

---

## Notes

- All PDO access must happen in `motor_process()` cyclic loop
- SDO access (if needed) should be done during initialization or exceptional cases
- Thread safety: currently single-threaded, but consider adding mutex if multi-threaded
- The commented pseudo-code in `motor_process()` provides the structure for reading inputs

---

## Status

- [ ] Task 1: Motor Enable Command
- [ ] Task 2: Index/Homing Command  
- [ ] Task 3: Absolute Move Command
- [ ] Task 4: Halt Command
- [ ] Task 5: Stop Command
- [ ] Task 6: Reset Command
- [ ] Task 7: Encoder Reset Command
- [ ] Task 8: Status Reading (Cyclic) - **Most Critical**

**Last Updated:** 2026-01-01
