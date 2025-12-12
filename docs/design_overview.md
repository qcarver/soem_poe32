# Laser Beam Acquisition System - Design Overview

## Project Summary

This ESP-IDF project implements a laser beam acquisition system for a 2-axis pan/tilt mechanism. The system uses Xeryon piezo-driven rotary motors controlled via EtherCAT (using SOEM library) and an I2C power sensor to:

1. **Acquire** a laser beam using a spiral scan pattern
2. **Track** the beam using a figure-8 pattern for continuous alignment

The code was ported from a CODESYS V3.5 PLC program to ESP32 C++.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Application Layer                          │
│                                                                     │
│  ┌─────────────┐  ┌─────────────────┐  ┌─────────────────────────┐ │
│  │ acquire.cpp │  │ scan_patterns.  │  │     acquire_config.h    │ │
│  │ State       │  │ cpp             │  │     Configuration       │ │
│  │ Machine     │  │ Pattern Gen     │  │     Constants           │ │
│  └──────┬──────┘  └────────┬────────┘  └─────────────────────────┘ │
│         │                  │                                        │
└─────────┼──────────────────┼────────────────────────────────────────┘
          │                  │
┌─────────┴──────────────────┴────────────────────────────────────────┐
│                     Hardware Abstraction Layer                      │
│                                                                     │
│  ┌──────────────────────┐       ┌───────────────────────┐          │
│  │   motor_control.cpp  │       │   power_sensor.cpp    │          │
│  │   SOEM/EtherCAT      │       │   I2C Driver          │          │
│  └──────────┬───────────┘       └───────────┬───────────┘          │
│             │                               │                       │
└─────────────┼───────────────────────────────┼───────────────────────┘
              │                               │
┌─────────────┴───────────────────────────────┴───────────────────────┐
│                         Hardware Layer                              │
│                                                                     │
│  ┌──────────────────────┐       ┌───────────────────────┐          │
│  │   Xeryon Drives      │       │   Power Sensor        │          │
│  │   (EtherCAT)         │       │   (I2C @ 0x51)        │          │
│  │   Drive1: Tilt       │       │   SDA: GPIO2          │          │
│  │   Drive2: Pan        │       │   SCL: GPIO15         │          │
│  └──────────────────────┘       └───────────────────────┘          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## File Structure

| File | Purpose |
|------|---------|
| `main/acquire_config.h` | Configuration constants (encoder resolution, positions, thresholds) |
| `main/acquire.h/.cpp` | Main state machine - acquisition logic |
| `main/motor_control.h/.cpp` | SOEM/EtherCAT motor control abstraction |
| `main/power_sensor.h/.cpp` | I2C laser power sensor driver |
| `main/scan_patterns.h/.cpp` | Spiral/figure-8 pattern generation and logging |
| `main/main.c` | Entry point, Ethernet initialization |

---

## State Machine

The acquisition follows this state flow:

```
┌───────────────────────────────────────────────────────────────────────┐
│                                                                       │
│   ┌───────┐    ┌───────┐    ┌─────────┐    ┌──────────┐             │
│   │ State │    │ State │    │ State   │    │ State    │             │
│   │   0   │───►│   1   │───►│   2     │───►│   3      │             │
│   │ Init  │    │ Index │    │ Move to │    │ Generate │             │
│   │Motors │    │Motors │    │ Start   │    │ Spiral   │             │
│   └───────┘    └───────┘    └─────────┘    └────┬─────┘             │
│                                                  │                    │
│                                                  ▼                    │
│   ┌───────────────────────────────────────────────────────────────┐  │
│   │                         State 4                               │  │
│   │                    Execute Spiral Scan                        │  │
│   │  ┌────────────────────────────────────────────────────────┐   │  │
│   │  │ For each point in spiral path:                         │   │  │
│   │  │   1. Move to position                                  │   │  │
│   │  │   2. Read power sensor                                 │   │  │
│   │  │   3. Log position + power                              │   │  │
│   │  │   4. If power >= threshold → HALT & go to State 7      │   │  │
│   │  └────────────────────────────────────────────────────────┘   │  │
│   └─────────────────────────────────┬─────────────────────────────┘  │
│                                     │                                │
│               ┌─────────────────────┴─────────────────────┐         │
│               │ Power >= MIN_POWER_THRESHOLD (70µW)?      │         │
│               └─────────────┬─────────────────┬───────────┘         │
│                      No     │                 │ Yes                  │
│                             ▼                 │                      │
│                    ┌────────────┐             │                      │
│                    │  State 5   │             │                      │
│                    │ Find Peak  │             │                      │
│                    │ (analyze   │             │                      │
│                    │  log data) │             │                      │
│                    └─────┬──────┘             │                      │
│                          │                    │                      │
│                          ▼                    │                      │
│                    ┌────────────┐             │                      │
│                    │  State 6   │             │                      │
│                    │ Move to    │◄────────────┘                      │
│                    │ Peak       │                                    │
│                    └─────┬──────┘                                    │
│                          │                                           │
│                          ▼                                           │
│   ┌───────────────────────────────────────────────────────────────┐  │
│   │                         State 7                               │  │
│   │                    Figure-8 Tracking                          │  │
│   │  ┌────────────────────────────────────────────────────────┐   │  │
│   │  │ Continuously trace figure-8 around beam center:        │   │  │
│   │  │   - Parametric lissajous curve (freq ratio 1:2)        │   │  │
│   │  │   - Adjust center based on power gradient              │   │  │
│   │  │   - If signal lost for 2s → return to State 3          │   │  │
│   │  └────────────────────────────────────────────────────────┘   │  │
│   └───────────────────────────────────────────────────────────────┘  │
│                          │                                           │
│                          │ Signal lost (power < 30µW for 2s)        │
│                          ▼                                           │
│                    ┌────────────┐                                    │
│                    │  State 3   │                                    │
│                    │ Generate   │◄─── Re-center on last good        │
│                    │ Spiral     │     position and restart          │
│                    └────────────┘                                    │
│                                                                       │
│   ┌────────────┐                                                     │
│   │  State 10  │◄── Any error condition                             │
│   │   ERROR    │                                                     │
│   └────────────┘                                                     │
│                                                                       │
└───────────────────────────────────────────────────────────────────────┘
```

### State Descriptions

| State | Name | Description |
|-------|------|-------------|
| 0 | INIT_MOTORS | Enable and initialize both Xeryon drives |
| 1 | INDEX_MOTORS | Home/index both drives to establish reference |
| 2 | MOVE_TO_START | Move to starting scan position (configurable) |
| 3 | GENERATE_SPIRAL | Generate spiral path buffer centered on target |
| 4 | EXECUTE_SPIRAL | Execute point-to-point spiral scan with power logging |
| 5 | FIND_PEAK | Analyze logged data to find highest power position |
| 6 | MOVE_TO_PEAK | Move to the peak power position |
| 7 | FIGURE8_TRACKING | Continuous figure-8 tracking pattern |
| 8 | HOLD | Dwell/hold position (reserved) |
| 10 | ERROR | Error state - requires reset |

---

## Configuration Parameters

All configurable values are in `acquire_config.h`:

### Encoder Settings
```cpp
#define ENCODER_RESOLUTION 1843200  // Encoder counts per revolution
```

### Starting Position
```cpp
#define START_PAN_DEG  -2.2    // Initial pan position (degrees)
#define START_TILT_DEG 178.4   // Initial tilt position (degrees)
```

### Power Thresholds
```cpp
#define MIN_POWER_THRESHOLD_UW 70.0f   // Minimum power to trigger acquisition
#define SIGNAL_LOSS_THRESHOLD_UW 30.0f // Below this = signal lost
```

### Motion Parameters
```cpp
#define DEFAULT_SPEED 10000    // Default motion speed
#define DEFAULT_ACC   65000    // Default acceleration
#define DEFAULT_DEC   65000    // Default deceleration
#define POSITION_TOLERANCE_ENC 100  // Allowed position error (encoder counts)
```

### Timing
```cpp
#define STATE_WAIT_MS 1000           // Inter-state delay
#define SIGNAL_LOSS_TIMEOUT_MS 2000  // Time before re-scan on signal loss
#define POWER_SETTLE_MS 150          // Power sensor settling time
```

### I2C Power Sensor
```cpp
#define POWER_SENSOR_I2C_ADDR 0x51
#define POWER_SENSOR_SDA_PIN  GPIO_NUM_2
#define POWER_SENSOR_SCL_PIN  GPIO_NUM_15
```

---

## SOEM Integration Notes

The `motor_control.cpp` module provides a hardware abstraction over SOEM. **Key TODOs remain**:

### PDO Mapping Required
The Xeryon drives communicate via EtherCAT PDOs (Process Data Objects). You'll need to:

1. **Obtain the Xeryon EtherCAT Object Dictionary** - This defines the PDO structure
2. **Implement PDO read/write** in the TODO sections of `motor_control.cpp`:

```cpp
// Example: Writing target position to output PDO
uint8_t *outputs = s_motor_ctx.context.slavelist[drive+1].outputs;
*(int32_t*)(outputs + TARGET_POSITION_OFFSET) = position;
*(uint16_t*)(outputs + CONTROLWORD_OFFSET) = CONTROL_START_MOTION;

// Example: Reading actual position from input PDO  
uint8_t *inputs = s_motor_ctx.context.slavelist[drive+1].inputs;
int32_t actual_pos = *(int32_t*)(inputs + ACTUAL_POSITION_OFFSET);
uint16_t statusword = *(uint16_t*)(inputs + STATUSWORD_OFFSET);
```

### SOEM API Used
This implementation uses SOEM's **context-based API** (`ecx_*` functions):
- `ecx_init()` - Initialize network interface
- `ecx_config_init()` - Discover and configure slaves
- `ecx_config_map_group()` - Map PDO data
- `ecx_send_processdata()` / `ecx_receive_processdata()` - Cyclic data exchange

---

## Coordinate System

- **Tilt (Drive 1)**: Elevation axis - typically ~180° range
- **Pan (Drive 2)**: Azimuth axis - typically small adjustments around center
- **Encoder**: 1,843,200 counts/revolution → ~5120 counts/degree

### Conversion Functions
```cpp
int32_t deg_to_enc(double degrees);  // Degrees → encoder counts
double enc_to_deg(int32_t enc);      // Encoder counts → degrees
```

---

## Scan Patterns

### Spiral Pattern
- **Type**: Archimedean spiral
- **Parameters**: 
  - Rings: 8 (configurable)
  - Points per ring: 20 (configurable)
  - Spacing: 0.1° per ring
  - Max radius: ~0.8° from center

### Figure-8 Pattern
- **Type**: Lissajous curve with frequency ratio 1:2
- **Parameters**:
  - Amplitude: 0.3° (configurable)
  - Period: ~2 seconds per cycle
- **Adaptive**: Center adjusts based on power gradient

---

## Logging System

The system maintains a circular log buffer for debugging and peak analysis:

```cpp
typedef struct {
    double pan_deg;
    double tilt_deg;
    float power_uw;
    uint32_t timestamp_ms;
} log_entry_t;

#define MAX_LOG_ENTRIES 600
```

Functions:
- `logger_init()` - Initialize/clear log
- `logger_add_entry()` - Add position/power sample
- `logger_get_entries()` - Retrieve log data
- `find_peak_power()` - Analyze log for maximum power point

---

## Next Steps / TODO

1. **Complete PDO Mapping**: Implement Xeryon-specific EtherCAT PDO structure
2. **Tune Parameters**: Adjust spiral density, figure-8 amplitude based on beam size
3. **Add Diagnostics**: Implement logging to SD card or network
4. **Error Recovery**: Add automatic recovery from transient errors
5. **Real-time Constraints**: Consider FreeRTOS task priorities for motor control loop

---

## Build & Flash

```bash
# Configure (if needed)
idf.py menuconfig

# Build
idf.py build

# Flash (adjust port as needed)
idf.py -p /dev/ttyUSB0 flash

# Monitor
idf.py -p /dev/ttyUSB0 monitor
```

---

## Hardware Requirements

- **ESP32-POE-ISO** (Olimex) or similar with Ethernet
- **Xeryon Piezo Rotary Stages** (2x) with EtherCAT interface
- **I2C Power Sensor** at address 0x51
- **EtherCAT Network** connecting ESP32 to drives
