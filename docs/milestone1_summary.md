# Milestone 1 — Project Scaffold + HAL

**Status:** Complete and compiling
**Target:** Teensy 4.0 (ARM Cortex-M7, 600 MHz)
**Build System:** PlatformIO + Arduino framework
**Date:** 2026-03-14

---

## Overview

Milestone 1 establishes the full project scaffold for a solar wing tracker: hardware abstraction layer (HAL), sensor pipeline, ESC actuator control, behavior tree framework, and serial telemetry. All sensors are read at 20 Hz, telemetry is output at 2 Hz, and the ESC arming state machine is operational. The codebase compiles cleanly for Teensy 4.0 with no warnings.

**Binary size:** firmware.hex ~85 KB, firmware.elf ~243 KB
**Teensy 4.0 capacity:** 1 MB Flash / 512 KB RAM — ample headroom for future milestones

---

## Directory Structure

```
src/
├── main.cpp                        Entry point — setup/loop, HAL instances, telemetry
├── bt/
│   └── bt_node.h                   Hand-rolled behavior tree (BtNode, Sequence, Selector, Condition, Action)
├── comms/
│   ├── comms_interface.h           Communications function declarations
│   ├── comms_interface.cpp         Stub implementation (TODO: microROS)
│   └── system_state.h              Central SystemState struct + enums
├── config/
│   └── config.h                    Pin map, calibration constants, thresholds
├── core/
│   └── solar_model.h               ISolarModel abstract interface + I/O structs
└── hal/
    ├── hal_as5600.h                IEncoder interface + HalAs5600 declaration
    ├── hal_as5600.cpp              HalAs5600 implementation (AS5600 I2C encoder)
    ├── hal_adc.h                   IAdc interface + HalAdc declaration
    ├── hal_adc.cpp                 HalAdc implementation (12-bit ADC reads)
    ├── hal_pwm.h                   IPwm interface + HalPwm declaration
    ├── hal_pwm.cpp                 HalPwm implementation (Servo ESC with arming FSM)
    ├── hal_pin_motor.h             IPinMotor interface + HalPinMotor declaration
    └── hal_pin_motor.cpp           HalPinMotor stub (serial debug only)
```

**Total:** 15 source files (9 `.h` + 5 `.cpp` + `main.cpp`)

---

## Module Reference

### bt/ — Behavior Tree

| File | Purpose |
|------|---------|
| `bt_node.h` | ~150-line hand-rolled BT framework — no STL, no heap |

**Abstract interface:**
- `BtNode` — `virtual NodeStatus tick(SystemState& state)`

**Concrete nodes (inline):**
- `Sequence` — ticks children in order; fails on first FAILURE
- `Selector` — ticks children in order; succeeds on first SUCCESS
- `Condition` — wraps `bool(*)(const SystemState&)` predicate
- `Action` — wraps `NodeStatus(*)(SystemState&)` function

**Enum:** `NodeStatus { SUCCESS, FAILURE, RUNNING }`
**Constraint:** `BT_MAX_CHILDREN = 8` (static allocation per composite)

---

### comms/ — Communications

| File | Purpose |
|------|---------|
| `comms_interface.h` | Declares `commsInit()`, `commsUpdate()`, `commsPublish()` |
| `comms_interface.cpp` | Stub — hardcoded Paris coordinates + time |
| `system_state.h` | Central `SystemState` struct (single source of truth) |

**Enums:**
- `PinLockState { UNLOCKED, LOCKING, LOCKED, UNLOCKING }`
- `TrackingMode { ASTRONOMICAL, SENSOR_BASED }`

**Struct: `SystemState`** — fields grouped by subsystem:

| Group | Fields |
|-------|--------|
| Time/Position | `year`, `month`, `day`, `hour`, `minute`, `second`, `latitude`, `longitude` |
| Flight Mode | `inFlight` |
| Encoder | `tiltAngleDeg`, `encoderRaw` |
| Quad Sensor | `quadTop`, `quadBottom`, `quadLeft`, `quadRight` |
| Environmental | `irradianceWm2`, `windSpeedMs`, `twilight` |
| Actuator | `tiltPulseUs`, `escArmed` |
| Tracking | `trackingMode`, `targetTiltDeg` |
| Pin Lock | `pinLockState` |
| Solar Model | `solarElevationDeg`, `solarAzimuthDeg`, `optimalTiltDeg` |

---

### config/ — Configuration

| File | Purpose |
|------|---------|
| `config.h` | All pin assignments, calibration factors, thresholds, timing |

See [Pin Map](#pin-map) and [Calibration Constants](#calibration-constants) below.

---

### core/ — Core Algorithms

| File | Purpose |
|------|---------|
| `solar_model.h` | `ISolarModel` abstract interface + `SolarModelInput`/`SolarModelOutput` structs |

**Abstract interface:**
- `ISolarModel` — `virtual SolarModelOutput compute(const SolarModelInput& input)`

**Structs:**
- `SolarModelInput { latitude, longitude, year, month, day, hour, minute, second }`
- `SolarModelOutput { optimal_tilt_deg, solar_elevation_deg, solar_azimuth_deg }`

No concrete implementation yet — interface only.

---

### hal/ — Hardware Abstraction Layer

| File | Interface | Implementation | Status |
|------|-----------|----------------|--------|
| `hal_as5600.h/cpp` | `IEncoder` | `HalAs5600` | Fully implemented |
| `hal_adc.h/cpp` | `IAdc` | `HalAdc` | Fully implemented |
| `hal_pwm.h/cpp` | `IPwm` | `HalPwm` | Fully implemented |
| `hal_pin_motor.h/cpp` | `IPinMotor` | `HalPinMotor` | **Stub** |

**IEncoder** (`hal_as5600.h`):
- `virtual bool init()`
- `virtual float readAngleDegrees()`
- `virtual uint16_t readRawCounts()`

**IAdc** (`hal_adc.h`):
- `virtual void init()`
- `virtual float readIrradiance()`
- `virtual float readWindSpeed()`
- `virtual uint16_t readQuadSensor(QuadChannel ch)`
- `virtual float readTwilight()`

**IPwm** (`hal_pwm.h`):
- `virtual bool init()`
- `virtual void setTiltPulseWidth(uint16_t us)`
- `virtual void setNeutral()`
- `virtual void arm(uint32_t now_ms)`
- `virtual bool isArmed()`

**IPinMotor** (`hal_pin_motor.h`):
- `virtual void init()`
- `virtual void setPinMotor(uint8_t motorIndex, PinMotorDir dir, uint8_t speed)`

**Additional enums:**
- `QuadChannel { TOP, BOTTOM, LEFT, RIGHT }` (hal_adc.h)
- `PinMotorDir { FORWARD, REVERSE, STOP }` (hal_pin_motor.h)
- `ArmState { IDLE, ARMING, ARMED }` (hal_pwm.h, private)

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                        main.cpp                             │
│          setup() → init all modules                         │
│          loop()  → 20 Hz read sensors, 2 Hz telemetry       │
└────────────┬──────────────────────────────────┬─────────────┘
             │                                  │
             ▼                                  ▼
┌────────────────────────┐         ┌────────────────────────┐
│      Sensor HAL        │         │      Actuator HAL      │
│  ┌──────────────────┐  │         │  ┌──────────────────┐  │
│  │ IEncoder         │  │         │  │ IPwm             │  │
│  │ (HalAs5600)      │──┐        │  │ (HalPwm)         │  │
│  └──────────────────┘  │ │       │  └──────────────────┘  │
│  ┌──────────────────┐  │ │       │  ┌──────────────────┐  │
│  │ IAdc             │  │ │       │  │ IPinMotor        │  │
│  │ (HalAdc)         │──┤ │       │  │ (HalPinMotor)    │  │
│  └──────────────────┘  │ │       │  │  [STUB]          │  │
└────────────────────────┘ │       │  └──────────────────┘  │
                           │       └────────────────────────┘
                           │                  ▲
                           ▼                  │
                  ┌─────────────────┐         │
                  │  SystemState    │─────────┘
                  │  (single source │
                  │   of truth)     │
                  └────────┬────────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
     ┌──────────────┐ ┌─────────┐ ┌─────────────┐
     │ ISolarModel  │ │ BtNode  │ │ Comms       │
     │ [NO IMPL]    │ │ [TREE   │ │ [microROS   │
     │              │ │  TBD]   │ │  TODO]      │
     └──────────────┘ └─────────┘ └─────────────┘
         future          future        future
```

**Data flow:** Sensors → HAL → SystemState → Telemetry (Serial)
**Future seams:** ISolarModel feeds optimal tilt → BT tree evaluates conditions → actuators respond

---

## Pin Map

All assignments from `config.h`. Teensy 4.0 GPIO numbering.

### ESC / Servo

| Constant | Pin | Function |
|----------|-----|----------|
| `PIN_ESC_PWM` | 2 | PWM output for wing tilt actuator |

### I2C (AS5600 Encoder)

| Constant | Pin | Function |
|----------|-----|----------|
| `PIN_I2C_SDA` | 18 | Teensy 4.0 Wire SDA |
| `PIN_I2C_SCL` | 19 | Teensy 4.0 Wire SCL |

### Quad Photo-Sensor (4-Quadrant Sun Sensor)

| Constant | Pin | Analog | Function |
|----------|-----|--------|----------|
| `PIN_QUAD_TOP` | 14 | A0 | Top quadrant |
| `PIN_QUAD_BOTTOM` | 15 | A1 | Bottom quadrant |
| `PIN_QUAD_LEFT` | 16 | A2 | Left quadrant |
| `PIN_QUAD_RIGHT` | 17 | A3 | Right quadrant |

### Environmental Sensors

| Constant | Pin | Analog | Function |
|----------|-----|--------|----------|
| `PIN_IRRADIANCE` | 20 | A6 | Solar irradiance sensor |
| `PIN_WIND` | 21 | A7 | Wind speed sensor |
| `PIN_TWILIGHT` | 22 | A8 | Ambient light / twilight sensor |

### Pin Motor H-Bridge (4 Motors)

| Motor | DIR_A | DIR_B | EN (PWM) |
|-------|-------|-------|----------|
| 0 | Pin 3 | Pin 4 | Pin 5 |
| 1 | Pin 6 | Pin 7 | Pin 8 |
| 2 | Pin 9 | Pin 10 | Pin 11 |
| 3 | Pin 12 | Pin 24 | Pin 25 |

**Total GPIO usage:** 22 pins

---

## Calibration Constants

| Constant | Value | Unit | Purpose |
|----------|-------|------|---------|
| `ADC_RESOLUTION_BITS` | 12 | bits | Teensy 4.0 ADC resolution |
| `ADC_MAX_COUNTS` | 4095 | counts | 12-bit max value |
| `ADC_REF_VOLTAGE` | 3.3 | V | ADC reference voltage |
| `IRRADIANCE_CAL_FACTOR` | 1500/4095 | W/m²/count | Linear irradiance scale (0–1500 W/m²) |
| `WIND_CAL_FACTOR` | 50/4095 | m·s⁻¹/count | Linear wind scale (0–50 m/s) |
| `TWILIGHT_CAL_FACTOR` | 1/4095 | 1/count | Normalized 0.0–1.0 |
| `IRRADIANCE_THRESHOLD_WM2` | 200 | W/m² | Min irradiance to engage tracking |
| `WIND_MAX_MS` | 15 | m/s | Max wind before stowing |
| `SENSOR_BALANCE_THRESHOLD` | 5 | counts | Quad sensor "balanced" threshold |
| `PWM_PULSE_MIN_US` | 1000 | µs | ESC full reverse |
| `PWM_PULSE_MAX_US` | 2000 | µs | ESC full forward |
| `PWM_PULSE_NEUTRAL` | 1500 | µs | ESC neutral |
| `PWM_FREQUENCY_HZ` | 50 | Hz | Standard servo frequency |
| `ESC_ARM_DURATION_MS` | 2000 | ms | Hold low throttle to arm |
| `SEASONAL_P` | 0.764 | — | Frydrychowicz-Jastrzębska tilt coefficient |
| `SEASONAL_Q` | 2.14 | — | Frydrychowicz-Jastrzębska tilt coefficient |
| `MAIN_LOOP_INTERVAL_MS` | 50 | ms | 20 Hz control loop |
| `SERIAL_PRINT_INTERVAL_MS` | 500 | ms | 2 Hz telemetry output |

---

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| `robtillaart/AS5600` | ^0.6.1 | AS5600 12-bit magnetic rotary encoder I2C driver |
| `luni64/TeensyTimerTool` | ^1.4.1 | Teensy 4.0 hardware timer abstraction |
| `Servo` | framework-bundled | ESC pulse-width control via `writeMicroseconds()` |

---

## Design Decisions

### Abstract interfaces for all HAL modules
Every hardware driver sits behind a pure virtual interface (`IEncoder`, `IAdc`, `IPwm`, `IPinMotor`). This enables mock implementations for desktop unit testing, driver swaps without touching application code, and dependency injection.

### Hand-rolled behavior tree (~150 lines)
BehaviorTree.CPP is heavy for embedded (STL, dynamic allocation). The hand-rolled BT uses static allocation (`BT_MAX_CHILDREN = 8`), no heap, and no STL — suitable for real-time embedded with the same architectural benefits.

### Servo library over PWMServo on Teensy 4.0
`PWMServo` only exposes `write(angle)`. The standard `Servo` library provides `writeMicroseconds()`, which is required for precise ESC pulse-width control.

### 12-bit ADC with linear calibration
Teensy 4.0 supports 12-bit ADC (4096 counts). All analog sensors use simple linear calibration factors defined in `config.h` — easy to tune without code changes.

### Swappable solar model via ISolarModel
The solar position algorithm is behind an abstract interface so the implementation can be swapped (Liu-Jordan, SPA, Grena, Frydrychowicz-Jastrzębska) without touching the control loop.

### Central SystemState struct
A single `SystemState` struct is the sole data contract between modules. Passed by reference, it enables functional design and makes the system trivially testable.

---

## Stubs and TODOs

### HalPinMotor (stub)
**Reason:** H-bridge / motor driver IC not yet selected.
**Current behavior:** `init()` prints a debug message; `setPinMotor()` prints motor index, direction, and speed to Serial. 12 GPIO pins are pre-allocated in `config.h`.

### Communications — microROS (stub)
**Current behavior:**
- `commsInit()` — no-op (TODO: microROS node init)
- `commsUpdate()` — hardcodes Paris coordinates (48.8566°N, 2.3522°E), time 2026-03-14 12:00:00 UTC, `inFlight = false`
- `commsPublish()` — no-op (TODO: microROS publishers)

### ISolarModel (interface only)
No concrete implementation. The interface and I/O structs are defined; candidate algorithms are noted in comments.

### Behavior tree (framework only)
The BT node types are implemented but no tree is defined — no `Sequence`/`Selector` instances are constructed in `main.cpp` yet.

### Location constants (placeholder)
`DEFAULT_LATITUDE` and `DEFAULT_LONGITUDE` are hardcoded to Paris, France.

---

## Interface Summary

| # | Interface | File | Methods | Concrete Impl |
|---|-----------|------|---------|---------------|
| 1 | `IEncoder` | hal/hal_as5600.h | 3 | `HalAs5600` |
| 2 | `IAdc` | hal/hal_adc.h | 5 | `HalAdc` |
| 3 | `IPwm` | hal/hal_pwm.h | 5 | `HalPwm` |
| 4 | `IPinMotor` | hal/hal_pin_motor.h | 2 | `HalPinMotor` (stub) |
| 5 | `ISolarModel` | core/solar_model.h | 1 | None |
| 6 | `BtNode` | bt/bt_node.h | 1 | Sequence, Selector, Condition, Action |

---

## Next Milestones

This scaffold directly enables:

1. **Solar Model Implementation** — plug a concrete `ISolarModel` (e.g., SPA or Grena) behind the existing interface
2. **Behavior Tree Definition** — wire up `Sequence`/`Selector`/`Condition`/`Action` nodes into a control tree in `main.cpp`
3. **Hybrid Tracker** — combine astronomical (solar model) and sensor-based (quad photo-sensor) tracking via `TrackingMode`
4. **Pin Motor Integration** — replace `HalPinMotor` stub once H-bridge IC is selected
5. **microROS Comms** — replace comms stubs with real microROS node for GPS, time sync, and flight mode
6. **Control Law** — closed-loop tilt control using encoder feedback + solar model target
