# CODESYS to C++ Port - Code Mapping Reference

This document maps the original CODESYS V3.5 PLC program to the ported ESP-IDF C++ implementation.

---

## Quick Reference

| CODESYS Entity | C++ Equivalent | File |
|----------------|----------------|------|
| `PLC_PRG` main program | `acquire_run()` state machine | [acquire.cpp](../main/acquire.cpp) |
| `GVL.Drive1` / `GVL.Drive2` | `motor_*()` functions | [motor_control.cpp](../main/motor_control.cpp) |
| `Custom_I2C_Power_Sensor` | `power_sensor_*()` functions | [power_sensor.cpp](../main/power_sensor.cpp) |
| `fbGenerateSpiral` | `generate_spiral()` | [scan_patterns.cpp](../main/scan_patterns.cpp) |
| `fbExecuteSpiral` | State 4 logic in `acquire_run()` | [acquire.cpp](../main/acquire.cpp#L650) |
| `fbFindPeak` | `find_peak_power()` | [scan_patterns.cpp](../main/scan_patterns.cpp) |
| `fbFigure8Scan` | `figure8_position()` + State 7 | [scan_patterns.cpp](../main/scan_patterns.cpp) |
| `AxisParams` structure | `axis_params_t` | [motor_control.h](../main/motor_control.h) |
| `PathPoint` structure | `path_point_t` | [scan_patterns.h](../main/scan_patterns.h) |
| `dposDegToEnc()` | `deg_to_enc()` | [motor_control.cpp](../main/motor_control.cpp) |
| `systemState` variable | `g_state` | [acquire.cpp](../main/acquire.cpp) |

---

## Annotated Legacy CODESYS PLC Code

Below is the original PLC program with annotations linking to the C++ implementation.

```pascal
(* ═══════════════════════════════════════════════════════════════════════════
   CODESYS V3.5 PLC PROGRAM - LASER BEAM ACQUISITION
   
   This file is annotated with links to the C++ port.
   Format: [CPP:filename#section] or [CPP:filename:function_name]
   ═══════════════════════════════════════════════════════════════════════════ *)

PROGRAM PLC_PRG
VAR
    (* ─────────────────────────────────────────────────────────────────────────
       DRIVE 1 (TILT) VARIABLES
       [CPP:motor_control.h - DRIVE_TILT enum, drive_state_t struct]
       ───────────────────────────────────────────────────────────────────────── *)
    drive1Enable : BOOL;           (* → motor_enable(DRIVE_TILT) *)
    drive1EnableOld : BOOL;
    drive1Index : BOOL;            (* → motor_index(DRIVE_TILT, params) *)
    drive1MoveAbs : BOOL;          (* → motor_move_absolute_enc(DRIVE_TILT, ...) *)
    drive1Dpos : BOOL;
    drive1Halt : BOOL;             (* → motor_halt(DRIVE_TILT) *)
    drive1Stop : BOOL;             (* → motor_stop(DRIVE_TILT) *)
    drive2ResetEncoder : BOOL;     (* → motor_reset_encoder(DRIVE_PAN) *)
    drive1ResetEncoder : BOOL;     (* → motor_reset_encoder(DRIVE_TILT) *)
    drive1Reset : BOOL;            (* → motor_reset(DRIVE_TILT) *)
    
    drive1State : INT;             (* → motor_get_status(DRIVE_TILT) *)
    drive1Target : BOOL;

    (* Motion parameters - [CPP:acquire_config.h - DEFAULT_SPEED/ACC/DEC] *)
    drive1Speed : DINT := 10000;   (* → DEFAULT_SPEED *)
    drive1Acc :  DINT := 65000;    (* → DEFAULT_ACC *)
    drive1Dec :  DINT := 65000;    (* → DEFAULT_DEC *)
    drive1ErrorString: STRING;

    (* ─────────────────────────────────────────────────────────────────────────
       DRIVE 2 (PAN) VARIABLES  
       [CPP:motor_control.h - DRIVE_PAN enum]
       ───────────────────────────────────────────────────────────────────────── *)
    drive2Enable : BOOL;           (* → motor_enable(DRIVE_PAN) *)
    drive2EnableOld : BOOL;
    drive2Index : BOOL;            (* → motor_index(DRIVE_PAN, params) *)
    drive2MoveAbs : BOOL;          (* → motor_move_absolute_enc(DRIVE_PAN, ...) *)
    drive2Dpos : BOOL;
    drive2Halt : BOOL;             (* → motor_halt(DRIVE_PAN) *)
    drive2Stop : BOOL;             (* → motor_stop(DRIVE_PAN) *)
    drive2Reset : BOOL;            (* → motor_reset(DRIVE_PAN) *)

    drive2State : INT;
    drive2Target : BOOL;

    drive2Speed : DINT := 10000;
    drive2Acc :  DINT := 65000;
    drive2Dec :  DINT := 65000;
    target_min : DINT;
    target_max : DINT;
    
    drive2ErrorString: STRING;

    dpos: DINT;
    Halt: INT;

    (* ─────────────────────────────────────────────────────────────────────────
       SYSTEM STATE AND SENSOR VARIABLES
       [CPP:acquire.cpp - g_state, acquire_config.h]
       ───────────────────────────────────────────────────────────────────────── *)
    temperature_reading : REAL;
    systemState : INT := 0;        (* → g_state in acquire.cpp *)
    
    (* Drive status - [CPP:motor_control.h - drive_status_t enum] *)
    drive1Status : XeryonDriveStatus;   (* → motor_get_status() returns drive_status_t *)
    drive2Status : XeryonDriveStatus;
    drive1Error : BOOL;            (* → motor_has_error(DRIVE_TILT) *)
    drive2Error : BOOL;            (* → motor_has_error(DRIVE_PAN) *)
    
    targetPanEnc : DINT;           (* → g_target_pan_enc *)
    targetTiltEnc : DINT;          (* → g_target_tilt_enc *)
    
    (* ─────────────────────────────────────────────────────────────────────────
       AXIS PARAMETERS STRUCTURE
       [CPP:motor_control.h - axis_params_t struct]
       ───────────────────────────────────────────────────────────────────────── *)
    Axis1Params : AxisParams := (Speed := 10000, Acc := 65000, Dec := 65000);
    Axis2Params : AxisParams := (Speed := 10000, Acc := 65000, Dec := 65000);
    (* C++ equivalent:
       typedef struct {
           int32_t speed;
           uint16_t acc;
           uint16_t dec;
       } axis_params_t;
       
       static axis_params_t g_tilt_params = {DEFAULT_SPEED, DEFAULT_ACC, DEFAULT_DEC};
       static axis_params_t g_pan_params = {DEFAULT_SPEED, DEFAULT_ACC, DEFAULT_DEC};
    *)
    
    (* ─────────────────────────────────────────────────────────────────────────
       FUNCTION BLOCKS → C++ FUNCTIONS
       [CPP:scan_patterns.h/cpp, acquire.cpp state machine]
       ───────────────────────────────────────────────────────────────────────── *)
    fbInit : InitDrives;           (* → acquire.cpp State 0: init_motors_state() *)
    fbIndex : IndexDrives;         (* → acquire.cpp State 1: index_motors_state() *)
    fbGenerateSpiral : GenerateSpiral;  (* → scan_patterns.cpp: generate_spiral() *)
    fbExecuteSpiral : ExecuteSpiral;    (* → acquire.cpp State 4 inline logic *)
    fbFindPeak : FindPeakPower;    (* → scan_patterns.cpp: find_peak_power() *)
    fbFigure8Scan: Figure8Scan;    (* → scan_patterns.cpp: figure8_position() *)
    
    initMotors : BOOL := TRUE;     (* → state entry flags in acquire.cpp *)
    indexMotors : BOOL := FALSE;
    
    (* ─────────────────────────────────────────────────────────────────────────
       SCAN CONTROL VARIABLES
       [CPP:acquire.cpp - static variables in anonymous namespace]
       ───────────────────────────────────────────────────────────────────────── *)
    firstSpiralExecuted : BOOL := FALSE;  (* → g_first_spiral_executed *)
    startSpiral  : BOOL := FALSE;         (* → g_start_spiral *)
    
    (* Peak position tracking - [CPP:acquire.cpp - g_peak_pan_deg, g_peak_tilt_deg] *)
    peakPanDeg   : LREAL := 0.0;          (* → g_peak_pan_deg *)
    peakTiltDeg  : LREAL := 180.0;        (* → g_peak_tilt_deg *)
    
    (* Starting position - [CPP:acquire_config.h - START_PAN_DEG, START_TILT_DEG] *)
    startPanDeg   : LREAL := -2.3;        (* → START_PAN_DEG = -2.2 *)
    startTiltDeg  : LREAL := 177.9;       (* → START_TILT_DEG = 178.4 *)
    
    CenterPan : LREAL;                    (* → g_center_pan *)
    CenterTilt : LREAL;                   (* → g_center_tilt *)
    CenterPanC3 : LREAL;
    CenterTiltC3 : LREAL;
    
    (* Power threshold - [CPP:acquire_config.h - MIN_POWER_THRESHOLD_UW] *)
    MIN_POWER_THRESHOLD : REAL := 70.0;   (* → MIN_POWER_THRESHOLD_UW = 70.0f *)
    
    (* ─────────────────────────────────────────────────────────────────────────
       PATH BUFFER
       [CPP:scan_patterns.h - path_point_t, MAX_PATH_POINTS]
       ───────────────────────────────────────────────────────────────────────── *)
    PathBuffer : ARRAY [0..599] OF PathPoint;  (* → path_point_t path_buffer[MAX_PATH_POINTS] *)
    currentScanPoint : INT := 0;               (* → g_current_scan_point *)
    spiralPointsCount : DINT := 0;             (* → g_spiral_points_count *)
    
    (* C++ PathPoint equivalent:
       typedef struct {
           double pan_deg;
           double tilt_deg;
       } path_point_t;
    *)
    
    firstScan : BOOL := TRUE;              (* → g_first_scan *)
    drivePeakMoveSent : BOOL := FALSE;     (* → g_peak_move_sent *)
    tiltError: DINT;                       (* → calculated inline *)
    panError: DINT;
    
    (* ─────────────────────────────────────────────────────────────────────────
       TIMERS
       [CPP:acquire.cpp - uses esp_timer_get_time() and elapsed time checks]
       ───────────────────────────────────────────────────────────────────────── *)
    waitStart : BOOL := FALSE;
    HoldTimer : TON;                       (* → g_hold_timer_start + HOLD_TIME_MS *)
    HOLD_TIME : TIME := T#10S;             (* → HOLD_TIME_MS = 10000 *)
    WaitTimer_State2 : TON;                (* → g_wait_timer_start per state *)
    WaitTimer_State3 : TON;
    WaitTimer_State4 : TON;
    WaitTimer_State5 : TON;
    WaitTimer_State6 : TON;
    WaitTimer_State7 : TON;
    PowerSettleTimer : TON;                (* → POWER_SETTLE_MS = 150 *)
    LossTimer : TON;                       (* → SIGNAL_LOSS_TIMEOUT_MS = 2000 *)
    WAIT_TIME : TIME := T#1S;              (* → STATE_WAIT_MS = 1000 *)
    
    (* Timer implementation note:
       CODESYS TON timers replaced with:
       
       static uint32_t g_timer_start = 0;
       
       // Start timer
       g_timer_start = get_time_ms();
       
       // Check elapsed
       if (get_time_ms() - g_timer_start >= TIMEOUT_MS) {
           // Timer expired
       }
    *)
    
    holdMoveSent : BOOL := FALSE;
    generateSent : BOOL := FALSE;          (* → g_generate_sent *)
    startGenerateC3 : BOOL := FALSE;
    startFindPeak : BOOL := FALSE;         (* → g_start_find_peak *)
    startFig8 : BOOL := FALSE;             (* → g_start_fig8 *)
    doneFig8 : BOOL := FALSE;              (* → g_done_fig8 *)
    powerSettle : BOOL := FALSE;           (* → g_power_settle *)
    GlobalMaxPower_uW : REAL;              (* → g_global_max_power_uw *)
END_VAR

VAR CONSTANT
    (* Position tolerance - [CPP:acquire_config.h - POSITION_TOLERANCE_ENC] *)
    ENC_TOL : DINT := 100;                 (* → POSITION_TOLERANCE_ENC = 100 *)
END_VAR

(* ═══════════════════════════════════════════════════════════════════════════
   INITIALIZATION / FIRST SCAN SETUP
   [CPP:acquire.cpp - acquire_init() and first iteration of acquire_run()]
   ═══════════════════════════════════════════════════════════════════════════ *)
IF firstScan THEN
    systemState := 0;
    startPanDeg  := -2.2;                  (* → START_PAN_DEG in acquire_config.h *)
    startTiltDeg := 178.4;                 (* → START_TILT_DEG in acquire_config.h *)
    firstScan := FALSE;
END_IF

(* ═══════════════════════════════════════════════════════════════════════════
   DRIVE FUNCTION BLOCK CALLS
   [CPP:motor_control.cpp - motor_process() called from main loop]
   ═══════════════════════════════════════════════════════════════════════════ *)
GVL.Drive1();                              (* → motor_process() handles cyclic update *)
GVL.Drive2();

drive1ErrorString := TO_STRING(GVL.Drive1.Error);
drive2ErrorString := TO_STRING(GVL.Drive2.Error);

(* ═══════════════════════════════════════════════════════════════════════════
   ENCODER READING CONVERSION
   [CPP:motor_control.cpp - enc_to_deg() function]
   
   Formula: deg = enc / ((2*PI/360) * 1E6 / ((2*PI*1E6)/1843200))
   Simplified: deg = enc * 360 / 1843200
   ═══════════════════════════════════════════════════════════════════════════ *)
GVL.encReadingDegTilt := GVL.Drive1.ActPosition / ((2 * 3.14159265359) / 360 * 1000000 / ((2*3.14159265359*1E6)/1843200));
GVL.encReadingDegPan := GVL.Drive2.ActPosition / ((2 * 3.14159265359) / 360 * 1000000 / ((2*3.14159265359*1E6)/1843200));

(* C++ equivalent:
   double enc_to_deg(int32_t enc_counts) {
       return (double)enc_counts * 360.0 / (double)ENCODER_RESOLUTION;
   }
*)

(* ═══════════════════════════════════════════════════════════════════════════
   POWER SENSOR READING
   [CPP:power_sensor.cpp - power_sensor_read()]
   ═══════════════════════════════════════════════════════════════════════════ *)
GVL.power_uw := Custom_I2C_Power_Sensor.Power_uW;

(* C++ equivalent:
   float power_uw;
   power_sensor_read(&power_uw);
*)

(* ═══════════════════════════════════════════════════════════════════════════
   STATE MACHINE
   [CPP:acquire.cpp - acquire_run() function]
   ═══════════════════════════════════════════════════════════════════════════ *)
CASE systemState OF

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 0: INITIALIZE MOTORS
       [CPP:acquire.cpp - case STATE_INIT_MOTORS]
       ───────────────────────────────────────────────────────────────────────── *)
    0:
        IF initMotors THEN
            fbInit(Drive1 := GVL.Drive1, Drive2 := GVL.Drive2);
        END_IF
        
        IF fbInit.Done THEN
            initMotors := FALSE;
            indexMotors := TRUE;
            systemState := 1;
        END_IF
        
        (* C++ equivalent:
           case STATE_INIT_MOTORS:
               if (!g_init_started) {
                   hal::drives_init();
                   hal::drive_enable(1);
                   hal::drive_enable(2);
                   g_init_started = true;
               }
               if (hal::drive_is_done(1) && hal::drive_is_done(2)) {
                   g_state = STATE_INDEX_MOTORS;
               }
               break;
        *)

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 1: INDEX/HOME MOTORS
       [CPP:acquire.cpp - case STATE_INDEX_MOTORS]
       ───────────────────────────────────────────────────────────────────────── *)
    1:
        IF indexMotors THEN
            fbIndex(Drive1 := GVL.Drive1, Drive2 := GVL.Drive2, 
                    Axis1 := Axis1Params, Axis2 := Axis2Params);
        END_IF
        
        IF fbIndex.Done THEN
            indexMotors := FALSE;
            systemState := 2;
        END_IF
        
        (* C++ equivalent:
           case STATE_INDEX_MOTORS:
               if (!g_index_started) {
                   hal::drive_index(1, g_tilt_params);
                   hal::drive_index(2, g_pan_params);
                   g_index_started = true;
               }
               if (hal::drive_is_done(1) && hal::drive_is_done(2)) {
                   g_state = STATE_MOVE_TO_START;
               }
               break;
        *)

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 2: MOVE TO STARTING POSITION
       [CPP:acquire.cpp - case STATE_MOVE_TO_START]
       ───────────────────────────────────────────────────────────────────────── *)
    2:
        IF NOT drive1MoveAbs THEN
            targetTiltEnc := dposDegToEnc(startTiltDeg);  (* → deg_to_enc(START_TILT_DEG) *)
            GVL.Drive1.MoveAbsolute(targetTiltEnc, drive1Speed, 
                                    DINT_TO_UINT(drive1Acc), DINT_TO_UINT(drive1Dec));
            drive1MoveAbs := TRUE;
        END_IF
    
        IF NOT drive2MoveAbs THEN
            targetPanEnc := dposDegToEnc(startPanDeg);    (* → deg_to_enc(START_PAN_DEG) *)
            GVL.Drive2.MoveAbsolute(targetPanEnc, drive2Speed, 
                                    DINT_TO_UINT(drive2Acc), DINT_TO_UINT(drive2Dec));
            drive2MoveAbs := TRUE;
        END_IF
    
        IF GVL.Drive1.Done AND drive1MoveAbs THEN
            drive1MoveAbs := FALSE;
        END_IF
        IF GVL.Drive2.Done AND drive2MoveAbs THEN
            drive2MoveAbs := FALSE;
            waitStart := TRUE;
        END_IF
        
        WaitTimer_State2(IN := waitStart, PT := T#1S);
        
        IF GVL.Drive1.HasError OR GVL.Drive2.HasError THEN
            systemState := 10;                            (* → STATE_ERROR *)
        END_IF
    
        IF WaitTimer_State2.Q THEN
            waitStart := FALSE;
            WaitTimer_State2(IN := FALSE);
            systemState := 3;
        END_IF
        
        (* C++ equivalent in acquire.cpp handles this with timer checks *)
        
    (* ─────────────────────────────────────────────────────────────────────────
       STATE 3: GENERATE SPIRAL PATTERN
       [CPP:acquire.cpp - case STATE_GENERATE_SPIRAL]
       [CPP:scan_patterns.cpp - generate_spiral()]
       ───────────────────────────────────────────────────────────────────────── *)
    3:
        startGenerateC3 := FALSE;
        IF NOT generateSent THEN
            IF NOT firstSpiralExecuted THEN
                CenterPanC3 := startPanDeg;
                CenterTiltC3 := startTiltDeg;
            ELSE
                CenterPanC3 := peakPanDeg;
                CenterTiltC3 := peakTiltDeg;
            END_IF
            startGenerateC3 := TRUE;
            fbGenerateSpiral(StartGenerate := startGenerateC3, 
                           CenterPan := CenterPanC3, CenterTilt := CenterTiltC3, 
                           PathData := PathBuffer);
            generateSent := TRUE;
            currentScanPoint := 0;
        ELSE
            startGenerateC3 := FALSE;
            fbGenerateSpiral(StartGenerate := startGenerateC3, 
                           CenterPan := CenterPanC3, CenterTilt := CenterTiltC3, 
                           PathData := PathBuffer);
        END_IF
        
        spiralPointsCount := fbGenerateSpiral.PointsCount;
        
        IF fbGenerateSpiral.Done THEN
            generateSent := FALSE;
            waitStart := TRUE;
        END_IF
        
        WaitTimer_State3(IN := waitStart, PT := T#1S);
        IF WaitTimer_State3.Q THEN
            waitStart := FALSE;
            WaitTimer_State3(IN := FALSE);
            startSpiral := TRUE;
            startGenerateC3 := FALSE;
            GVL.LogActive := TRUE;                        (* → logger_init() *)
            GVL.LogIndex := 0;
            systemState := 4;
        END_IF
        
        (* C++ equivalent:
           case STATE_GENERATE_SPIRAL: {
               double center_pan = g_first_spiral_executed ? g_peak_pan_deg : START_PAN_DEG;
               double center_tilt = g_first_spiral_executed ? g_peak_tilt_deg : START_TILT_DEG;
               
               spiral_params_t params = {
                   .center_pan_deg = center_pan,
                   .center_tilt_deg = center_tilt,
                   .rings = SPIRAL_RINGS,
                   .points_per_ring = SPIRAL_POINTS_PER_RING,
                   .ring_spacing_deg = SPIRAL_RING_SPACING_DEG
               };
               
               g_spiral_points_count = generate_spiral(&params, g_path_buffer, MAX_PATH_POINTS);
               logger_init();
               g_state = STATE_EXECUTE_SPIRAL;
           } break;
        *)

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 4: EXECUTE SPIRAL SCAN
       [CPP:acquire.cpp - case STATE_EXECUTE_SPIRAL]
       ───────────────────────────────────────────────────────────────────────── *)
    4:
        IF NOT startSpiral THEN
            startSpiral := TRUE;
        END_IF
        
        fbExecuteSpiral(
            Drive1 := GVL.Drive1, Drive2 := GVL.Drive2, 
            Axis1 := Axis1Params, Axis2 := Axis2Params,
            StartMove := startSpiral, CurrentPower_uW := GVL.power_uw, 
            Min_Power_uW := MIN_POWER_THRESHOLD,
            PathData := PathBuffer, PointsCount := spiralPointsCount,
            CurrentIndex := currentScanPoint);
            
        (* EARLY EXIT: Power threshold exceeded *)
        IF GVL.power_uw >= MIN_POWER_THRESHOLD THEN
            GVL.Drive1.Halt();                            (* → motor_halt(DRIVE_TILT) *)
            GVL.Drive2.Halt();                            (* → motor_halt(DRIVE_PAN) *)
            
            peakTiltDeg := GVL.encReadingDegTilt;
            peakPanDeg := GVL.encReadingDegPan;
            
            startSpiral := FALSE;
            firstSpiralExecuted := TRUE;
            GVL.LogActive := FALSE;
            waitStart := TRUE;
        END_IF

        WaitTimer_State4(IN := waitStart, PT := WAIT_TIME);
        
        IF fbExecuteSpiral.Done AND NOT fbExecuteSpiral.Executing THEN
            firstSpiralExecuted := TRUE;
            GVL.LogActive := FALSE;
            waitStart := TRUE;
        END_IF
        
        IF WaitTimer_State4.Q THEN
            WaitTimer_State4(IN:=FALSE);
            waitStart := FALSE;
            startSpiral := FALSE;
            
            IF GVL.power_uw >= MIN_POWER_THRESHOLD THEN
                GVL.LogActive := TRUE;
                GVL.LogIndex := 0;
                GlobalMaxPower_uW := GVL.power_uw;
                startFig8 := TRUE;
                systemState := 7;                         (* → STATE_FIGURE8_TRACKING *)
            END_IF
        END_IF
        
        (* C++ scan loop is inline in acquire_run() state 4 *)

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 5: FIND PEAK POWER POSITION
       [CPP:acquire.cpp - case STATE_FIND_PEAK]
       [CPP:scan_patterns.cpp - find_peak_power()]
       ───────────────────────────────────────────────────────────────────────── *)
    5:
        IF NOT startFindPeak THEN
            startFindPeak := TRUE;
        END_IF
        
        IF startFindPeak THEN
            fbFindPeak(StartAnalysis := startFindPeak, 
                      MaxIndexToSearch := GVL.LogIndex, 
                      HistoryData := GVL.ContinuousHistoryData);
        END_IF
        
        WaitTimer_State5(IN := waitStart, PT := WAIT_TIME);
        
        IF fbFindPeak.Done THEN
            waitStart := TRUE;
        END_IF
        
        IF WaitTimer_State5.Q THEN
            WaitTimer_State5(IN:=FALSE);
            waitStart := FALSE;
            peakTiltDeg := fbFindPeak.PeakTiltDeg;
            peakPanDeg  := fbFindPeak.PeakPanDeg;
            fbFindPeak.StartAnalysis := FALSE;
            startFindPeak := FALSE;
            systemState := 6;
        END_IF
        
        (* C++ equivalent:
           case STATE_FIND_PEAK: {
               double peak_pan, peak_tilt;
               float peak_power;
               
               if (find_peak_power(logger_get_entries(), logger_get_count(),
                                   &peak_pan, &peak_tilt, &peak_power)) {
                   g_peak_pan_deg = peak_pan;
                   g_peak_tilt_deg = peak_tilt;
               }
               g_state = STATE_MOVE_TO_PEAK;
           } break;
        *)

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 6: MOVE TO PEAK POWER POSITION
       [CPP:acquire.cpp - case STATE_MOVE_TO_PEAK]
       ───────────────────────────────────────────────────────────────────────── *)
    6:
        targetTiltEnc := dposDegToEnc(peakTiltDeg);
        targetPanEnc  := dposDegToEnc(peakPanDeg);
    
        tiltError := ABS(GVL.Drive1.ActPosition - targetTiltEnc);
        panError  := ABS(GVL.Drive2.ActPosition - targetPanEnc);
        
        IF (NOT drivePeakMoveSent) OR (tiltError > ENC_TOL) OR (panError > ENC_TOL) THEN
            GVL.Drive1.MoveAbsolute(targetTiltEnc, drive1Speed, 
                                    DINT_TO_UINT(drive1Acc), DINT_TO_UINT(drive1Dec));
            GVL.Drive2.MoveAbsolute(targetPanEnc, drive2Speed, 
                                    DINT_TO_UINT(drive2Acc), DINT_TO_UINT(drive2Dec));
            drivePeakMoveSent := TRUE;
        END_IF
        
        PowerSettleTimer(IN := powerSettle, PT := T#150MS);  (* → POWER_SETTLE_MS *)
        
        IF GVL.Drive1.Done OR GVL.Drive2.Done THEN
            drivePeakMoveSent := FALSE;
            powerSettle := TRUE;
        END_IF
        
        IF PowerSettleTimer.Q THEN
            powerSettle := FALSE;
            PowerSettleTimer(IN := FALSE);

            IF (tiltError <= ENC_TOL) AND (panError <= ENC_TOL) THEN
                waitStart := TRUE;
            ELSE
                waitStart := FALSE;
            END_IF
        END_IF
        
        WaitTimer_State6(IN := waitStart, PT := WAIT_TIME);
        IF WaitTimer_State6.Q THEN
            WaitTimer_State6(IN:=FALSE);
            GVL.LogActive := TRUE;
            GVL.LogIndex := 0;
            startFig8 := TRUE;
        END_IF

    (* ─────────────────────────────────────────────────────────────────────────
       STATE 7: FIGURE-8 TRACKING
       [CPP:acquire.cpp - case STATE_FIGURE8_TRACKING]
       [CPP:scan_patterns.cpp - figure8_position()]
       ───────────────────────────────────────────────────────────────────────── *)
    7:
        (* Signal loss timer - restart scan if lost for 2 seconds *)
        LossTimer(IN := (GVL.power_uw <= 30.0), PT := T#2S);  (* → SIGNAL_LOSS_TIMEOUT_MS *)
        
        IF startFig8 THEN
            fbFigure8Scan(
                Drive1 := GVL.Drive1, Drive2 := GVL.Drive2, 
                Axis1:= Axis1Params, Axis2:= Axis2Params,
                StartScan := startFig8, CurrentPower_uW := GVL.power_uw,
                CenterPan := peakPanDeg, CenterTilt := peakTiltDeg,
                GlobalMaxPower_uW := GlobalMaxPower_uW);
        END_IF
        
        IF LossTimer.Q OR doneFig8 THEN
            doneFig8 := TRUE;
            startFig8 := FALSE;
            startSpiral := FALSE;
        END_IF
        
        IF doneFig8 THEN
            peakPanDeg := fbFigure8Scan.LastPan;
            peakTiltDeg := fbFigure8Scan.LastTilt;
            startFig8 := FALSE;
            startSpiral := FALSE;
            doneFig8 := FALSE;
            waitStart := TRUE;
            firstSpiralExecuted := TRUE;
            startGenerateC3 := FALSE;
            generateSent := FALSE;
        END_IF
        
        WaitTimer_State7(IN := waitStart, PT := WAIT_TIME);
        IF WaitTimer_State7.Q THEN
            waitStart := FALSE;
            systemState := 3;                             (* → Back to GENERATE_SPIRAL *)
        END_IF
        
        (* C++ equivalent:
           case STATE_FIGURE8_TRACKING: {
               // Check for signal loss
               if (current_power < SIGNAL_LOSS_THRESHOLD_UW) {
                   if (get_time_ms() - g_loss_timer_start >= SIGNAL_LOSS_TIMEOUT_MS) {
                       g_peak_pan_deg = last_good_pan;
                       g_peak_tilt_deg = last_good_tilt;
                       g_state = STATE_GENERATE_SPIRAL;
                       break;
                   }
               } else {
                   g_loss_timer_start = get_time_ms();
               }
               
               // Calculate figure-8 position
               double pan_offset, tilt_offset;
               figure8_position(g_fig8_phase, FIGURE8_AMPLITUDE_DEG,
                               &pan_offset, &tilt_offset);
               
               // Move to new position
               double target_pan = g_peak_pan_deg + pan_offset;
               double target_tilt = g_peak_tilt_deg + tilt_offset;
               hal::drive_move_absolute(2, deg_to_enc(target_pan), g_pan_params);
               hal::drive_move_absolute(1, deg_to_enc(target_tilt), g_tilt_params);
               
               g_fig8_phase += FIGURE8_PHASE_INCREMENT;
           } break;
        *)

    8:
        (* HOLD state - reserved for future use *)
        
END_CASE

(* ═══════════════════════════════════════════════════════════════════════════
   MANUAL DRIVE CONTROLS
   [CPP:acquire.cpp - can be called via acquire_manual_halt(), etc.]
   ═══════════════════════════════════════════════════════════════════════════ *)
IF(drive1Halt) THEN
    GVL.Drive1.Halt();                                    (* → motor_halt(DRIVE_TILT) *)
END_IF

IF(drive1Stop) THEN
    GVL.Drive1.Stop();                                    (* → motor_stop(DRIVE_TILT) *)
END_IF

IF(drive1Reset) THEN
    GVL.Drive1.Reset();                                   (* → motor_reset(DRIVE_TILT) *)
END_IF

IF(drive1ResetEncoder) THEN
    GVL.Drive1.ResetEncoder();                            (* → motor_reset_encoder(DRIVE_TILT) *)
END_IF

IF(drive2Halt) THEN
    GVL.Drive2.Halt();                                    (* → motor_halt(DRIVE_PAN) *)
END_IF

IF(drive2Stop) THEN
    GVL.Drive2.Stop();                                    (* → motor_stop(DRIVE_PAN) *)
END_IF

IF(drive2Reset) THEN
    GVL.Drive2.Reset();                                   (* → motor_reset(DRIVE_PAN) *)
END_IF

IF(drive2ResetEncoder) THEN
    GVL.Drive2.ResetEncoder();                            (* → motor_reset_encoder(DRIVE_PAN) *)
END_IF

(* Manual target position commands *)
IF drive1Target THEN
    targetTiltEnc := dposDegToEnc(startTiltDeg);
    GVL.Drive1.MoveAbsolute(targetTiltEnc, drive1Speed, 
                            DINT_TO_UINT(drive1Acc), DINT_TO_UINT(drive1Dec));
END_IF

IF drive2Target THEN
    targetPanEnc := dposDegToEnc(startPanDeg);
    GVL.Drive2.MoveAbsolute(targetPanEnc, drive2Speed, 
                            DINT_TO_UINT(drive2Acc), DINT_TO_UINT(drive2Dec));
END_IF
```

---

## Data Type Mappings

| CODESYS Type | C++ Type | Notes |
|--------------|----------|-------|
| `BOOL` | `bool` | |
| `INT` | `int16_t` | |
| `DINT` | `int32_t` | |
| `UINT` | `uint16_t` | |
| `REAL` | `float` | |
| `LREAL` | `double` | |
| `TIME` | `uint32_t` | Milliseconds |
| `STRING` | `char[]` or `std::string` | |
| `TON` (timer) | Manual timestamp + elapsed check | See timer pattern below |
| `ARRAY[0..N]` | `type arr[N+1]` | |

### Timer Pattern Conversion

**CODESYS TON Timer:**
```pascal
WaitTimer(IN := waitStart, PT := T#1S);
IF WaitTimer.Q THEN
    (* Timer expired *)
END_IF
```

**C++ Equivalent:**
```cpp
static uint32_t timer_start = 0;
static bool timer_running = false;

// Start timer
if (wait_start && !timer_running) {
    timer_start = get_time_ms();
    timer_running = true;
}

// Check expiration
if (timer_running && (get_time_ms() - timer_start >= 1000)) {
    timer_running = false;
    // Timer expired
}
```

---

## Function Block to Function Mapping

### InitDrives → acquire.cpp State 0
```pascal
(* CODESYS *)
fbInit(Drive1 := GVL.Drive1, Drive2 := GVL.Drive2);
IF fbInit.Done THEN ...
```

```cpp
// C++
hal::drives_init();
hal::drive_enable(1);
hal::drive_enable(2);
// Wait for done flags
```

### GenerateSpiral → generate_spiral()
```pascal
(* CODESYS *)
fbGenerateSpiral(StartGenerate := TRUE, CenterPan := x, CenterTilt := y, PathData := buffer);
spiralPointsCount := fbGenerateSpiral.PointsCount;
```

```cpp
// C++
spiral_params_t params = {x, y, 8, 20, 0.1};
int count = generate_spiral(&params, buffer, MAX_PATH_POINTS);
```

### FindPeakPower → find_peak_power()
```pascal
(* CODESYS *)
fbFindPeak(StartAnalysis := TRUE, HistoryData := GVL.ContinuousHistoryData);
peakPanDeg := fbFindPeak.PeakPanDeg;
peakTiltDeg := fbFindPeak.PeakTiltDeg;
```

```cpp
// C++
double peak_pan, peak_tilt;
float peak_power;
find_peak_power(log_entries, count, &peak_pan, &peak_tilt, &peak_power);
```

### Figure8Scan → figure8_position()
```pascal
(* CODESYS *)
fbFigure8Scan(StartScan := TRUE, CenterPan := x, CenterTilt := y, ...);
```

```cpp
// C++ - called each cycle
double pan_offset, tilt_offset;
figure8_position(phase, amplitude, &pan_offset, &tilt_offset);
// Apply: target = center + offset
```

---

## Global Variable Mappings

| CODESYS GVL | C++ Location | Description |
|-------------|--------------|-------------|
| `GVL.Drive1` | `motor_*()` functions | Tilt drive |
| `GVL.Drive2` | `motor_*()` functions | Pan drive |
| `GVL.power_uw` | `power_sensor_read()` | Power reading |
| `GVL.encReadingDegTilt` | `motor_get_position_deg(DRIVE_TILT)` | Tilt position |
| `GVL.encReadingDegPan` | `motor_get_position_deg(DRIVE_PAN)` | Pan position |
| `GVL.LogActive` | Logger state | Logging enabled |
| `GVL.LogIndex` | `logger_get_count()` | Log entry count |
| `GVL.ContinuousHistoryData` | `logger_get_entries()` | Log buffer |

---

## Configuration Value Changes

Some values were adjusted during the port:

| Parameter | CODESYS Value | C++ Value | Reason |
|-----------|---------------|-----------|--------|
| `startPanDeg` | -2.3 | -2.2 | Updated default |
| `startTiltDeg` | 177.9 | 178.4 | Updated default |
| Timer resolution | T#1S | 1000ms | Same, different representation |

---

## Files Quick Reference

- **State Machine**: [acquire.cpp](../main/acquire.cpp)
- **Motor Control**: [motor_control.cpp](../main/motor_control.cpp)
- **Power Sensor**: [power_sensor.cpp](../main/power_sensor.cpp)
- **Scan Patterns**: [scan_patterns.cpp](../main/scan_patterns.cpp)
- **Configuration**: [acquire_config.h](../main/acquire_config.h)
- **Motor API**: [motor_control.h](../main/motor_control.h)
