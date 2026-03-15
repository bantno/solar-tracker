# Milestone 1: Project Scaffold + HAL

## Implementation Tasks
- [x] Update `platformio.ini` with lib_deps and monitor_speed
- [x] Create `src/config/config.h` — all pin assignments, thresholds, calibration, timing
- [x] Create `src/comms/system_state.h` — PinLockState, TrackingMode, SystemState
- [x] Create `src/comms/comms_interface.h` + `.cpp` — stub comms with hardcoded values
- [x] Create `src/hal/hal_as5600.h` + `.cpp` — IEncoder interface + AS5600 implementation
- [x] Create `src/hal/hal_adc.h` + `.cpp` — IAdc interface + Teensy ADC implementation
- [x] Create `src/hal/hal_pwm.h` + `.cpp` — IPwm interface + Servo-based ESC control
- [x] Create `src/hal/hal_pin_motor.h` + `.cpp` — IPinMotor interface + debug stub
- [x] Create `src/main.cpp` — sensor-reading main loop, non-blocking timing
- [x] Create `src/core/solar_model.h` — ISolarModel abstract interface
- [x] Create `src/bt/bt_node.h` — behavior tree infrastructure (header-only)

## Verification
- [x] `pio run` compiles without errors for teensy40
- [x] All files exist in correct directory structure
- [x] No `delay()` calls anywhere
- [x] No magic numbers outside `config.h`
- [x] `comms_interface` is the only seam for external data
- [x] CLAUDE.md updated with lessons learned

## Review
Milestone 1 complete. All 11 source modules created, project compiles clean for Teensy 4.0.
Flash: 16.5 KB code + 5.3 KB data. RAM1: 7.4 KB variables + 14.3 KB code.

One fix during build: switched from PWMServo to Servo library because PWMServo lacks
`writeMicroseconds()` — lesson captured in CLAUDE.md.
