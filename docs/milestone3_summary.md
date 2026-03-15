# Milestone 3 — Tilt Controller, Hybrid Tracker, and Behavior Tree

## Completed: 2026-03-15

## Overview

Wired the existing scaffold into a working closed-loop solar tracking control
system. The Teensy now: reads sensors at 20 Hz, evaluates safety at 1 Hz,
computes new tilt targets every 15 minutes via the hybrid tracker, and runs a
PID controller at 20 Hz until the encoder reads within deadband of the target.

No new hardware introduced — entirely software.

## Timing Architecture

Three independent timer loops, all non-blocking (`millis()` pattern):

| Timer | Interval | Purpose |
|---|---|---|
| `MAIN_LOOP_INTERVAL_MS` | 50 ms (20 Hz) | Sensor reads + PID move tick |
| `SAFETY_INTERVAL_MS` | 1000 ms (1 Hz) | Safety evaluation + BT tick |
| `TRACKING_INTERVAL_MS` | 900000 ms (15 min) | New target computation |

The PID runs inside the 20 Hz loop — active only while `moveInProgress` is
true. Between moves, the motor holds at neutral (1500 us).

## Files Created

### src/core/tilt_controller.h / .cpp

PID position controller. Pure C++, no hardware dependencies.

- **Anti-windup** via back-calculation: when output saturates, subtracts
  `K_aw * (saturated - unsaturated)` from integrator
- **Derivative on measurement**: differentiates `current_deg` not error,
  preventing derivative kick on new target
- **dt clamping**: [1 ms, 100 ms]; first tick after `reset()` is P-only
- **Deadband**: returns exactly 1500 us when `|error| < deadband_deg`
- **Output mapping**: 1500 us = neutral, clamped to [1000, 2000] us

### src/core/hybrid_tracker.h / .cpp

Decides tilt target. Called only on `TRACKING_INTERVAL`, not every tick.

- **SENSOR_BASED mode** (irradiance > threshold): `target = current + (quadTop - quadBottom) * SENSOR_TRACK_GAIN`
- **ASTRONOMICAL mode** (irradiance <= threshold): `target = solar_model->compute().optimal_tilt_deg`
- **Mode hysteresis**: mode switch requires `MODE_HOLD_DURATION_MS` (2 min)
  of continuous triggering condition — prevents toggling on intermittent clouds
- Clamped to `[TILT_BETA_MIN_DEG, TILT_BETA_MAX_DEG]`
- Sets `moveInProgress = true` when new target differs from current by more
  than `TILT_PID_DEADBAND_DEG`

### src/core/safety_monitor.h / .cpp

Evaluates safety conditions on the `SAFETY_INTERVAL`. Never commands actuators
directly — only sets flags and targets in SystemState.

- `safetyWindOverride` — wind > `WIND_MAX_MS` → commands `TILT_WIND_SAFE_DEG`
- `safetyFlightLock` — aircraft in flight
- `safetyNightReset` — twilight < `TWILIGHT_THRESHOLD` → commands `TILT_NIGHT_RESET_DEG` on false-to-true edge
- `safetyEscNotArmed` — ESC not armed

### test/test_tilt_controller.cpp

6 test cases + simulation trace:
1. Step response — settles 0° → 45° within 100 ticks, output always in [1000, 2000]
2. Deadband — output exactly 1500 us when |error| < 0.5°
3. Anti-windup — integral stays bounded under sustained saturation
4. Reset — zeroes integrator/derivative, no derivative kick on first tick
5. Direction — positive error → output > 1500, negative → output < 1500
6. Move complete — `isSettled()` false on first tick of 45° step, true when converged

### test/test_hybrid_tracker.cpp

6 test cases with `MockSolarModel`:
1. Mode selection — irradiance above/below threshold switches modes
2. Mode hysteresis — no switch before `MODE_HOLD_DURATION_MS` elapses
3. Sensor-based target — quadTop=600, quadBottom=400 → target increases
4. Astronomical target — uses mock model's fixed output
5. Clamping — extreme sensor values capped to `TILT_BETA_MAX_DEG`
6. moveInProgress trigger — set when target differs > deadband, unchanged within

## Files Modified

### src/config/config.h

Added 14 constants:

| Constant | Default | Unit |
|---|---|---|
| `SAFETY_INTERVAL_MS` | 1000 | ms |
| `TRACKING_INTERVAL_MS` | 900000 | ms |
| `TILT_PID_KP` | 8.0 | us/deg |
| `TILT_PID_KI` | 0.5 | us/(deg*s) |
| `TILT_PID_KD` | 1.2 | us*s/deg |
| `TILT_PID_KAW` | 0.1 | dimensionless |
| `TILT_PID_OUT_MIN` | 1000 | us |
| `TILT_PID_OUT_MAX` | 2000 | us |
| `TILT_PID_DEADBAND_DEG` | 0.5 | degrees |
| `SENSOR_TRACK_GAIN` | 0.02 | deg/ADC count |
| `MODE_HOLD_DURATION_MS` | 120000 | ms |
| `TWILIGHT_THRESHOLD` | 0.1 | normalized 0-1 |
| `TILT_NIGHT_RESET_DEG` | 0.0 | degrees |
| `TILT_WIND_SAFE_DEG` | 0.0 | degrees |

### src/comms/system_state.h

Added 6 fields:
- `bool moveInProgress` — true while PID is driving toward target
- `float moveDuration_s` — duration of last completed move
- `float verticalSensorError` — quadTop - quadBottom (telemetry)
- `float horizontalSensorError` — quadLeft - quadRight (telemetry)
- `bool safetyWindOverride`, `safetyFlightLock`, `safetyNightReset`, `safetyEscNotArmed`

### src/main.cpp

Complete rewrite of `loop()` with:
- Full control loop: sensor reads → PID move → safety → tracking → telemetry
- Behavior tree (static allocation, file-scope nodes):
  - Selector root with safety branch (blocks movement when unsafe) and tracking
    branch (permits movement when safe)
  - BT ticked on safety interval only, not every 20 Hz tick
- Extended telemetry at 2 Hz: `[TRACKER]`, `[PID]`, `[SAFETY]`, `[SOLAR]`, `[SENSOR]` lines
- `moveInProgress` is sole authority on whether PID drives the motor

### platformio.ini

Added `build_flags = -DPLATFORMIO_BUILD` to teensy40 env to allow pure C++
core files to compile (PlatformIO defines `ARDUINO` globally; the `#error`
guards now check `!defined(PLATFORMIO_BUILD)` to skip in PIO builds while
still catching accidental Arduino IDE compilation).

### src/core/solar_math.cpp

Updated `#error` guard from `#ifdef ARDUINO` to
`#if defined(ARDUINO) && !defined(PLATFORMIO_BUILD)` for PlatformIO
compatibility.

## Behavior Tree Structure

```
Selector (root_bt)
├── Sequence (safety_branch)
│   ├── Condition: any_safety_flag_set
│   │     → SUCCESS if any safety flag is true
│   └── Action: handle_safety_override
│         → flight lock / ESC not armed: freeze movement
│         → wind / night: targets already set by SafetyMonitor
│         → resets PID controller
│         → returns RUNNING
└── Sequence (tracking_branch)
    ├── Condition: esc_armed
    │     → SUCCESS if ESC is armed
    └── Action: permit_tracking
          → no-op, returns SUCCESS
```

The BT is a pure safety gate. Tracking and PID are driven by their own timers.

## Test Results

All 12 desktop tests pass:
- `test_pid.exe` — 6/6 passed. PID settles 0° → 45° in 47 ticks (~2.4s simulated).
- `test_tracker.exe` — 6/6 passed. Mode hysteresis, clamping, and moveInProgress logic verified.

Simulation trace (20° → 35°, 50ms ticks): settles in 42 ticks (2.1s).
Output ramps smoothly from 1620 us → 1500 us neutral.

## Build Results

Teensy 4.0 build: clean success.
- FLASH: 25,160 bytes code + 6,332 data
- RAM1: 8,512 variables + 23,032 code
- RAM2: 12,416 variables
