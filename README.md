# Solar Tracker

Embedded firmware for autonomous solar wing tilt control on a Teensy 4.x platform. Tracks the sun using a hybrid astronomical/sensor strategy and drives a GreenJay brushless ESC via closed-loop PID.

## Hardware

| Component | Role |
|---|---|
| Teensy 4.1 | Main solar tracking controller |
| Teensy 4.0 | Wing deploy / lock-pin actuator nodes |
| AS5600 | Magnetic rotary encoder (tilt angle feedback) |
| Quad photodiode array | 4-channel irradiance sensor (A0–A3) |
| GreenJay ESC | Brushless tilt motor driver (PWM, 50 Hz) |
| Linear actuator × 2 | Lock-pin extension/retraction with limit switches |
| XM125 (60 GHz radar) | Distance sensor (test harness) |
| LDR pair (A0/A1) | Alternate light-differential tracking sensor |

## Firmware Targets

| PlatformIO Env | Board | Description |
|---|---|---|
| `teensy41` | Teensy 4.1 | Main tracker: hybrid solar model + PID servo loop |
| `teensy40_wing_deploy` | Teensy 4.0 | Manual ESC control via serial arrow-key commands |
| `teensy40_pin_extender` | Teensy 4.0 | Lock-pin linear actuator state machine |
| `teensy40_distance` | Teensy 4.0 | XM125 mmWave radar test harness |
| `teensy41_ldr_compare` | Teensy 4.1 | LDR pair logging to SD card (bench calibration) |
| `teensy41_ldr_track` | Teensy 4.1 | Real-time LDR differential PID tracking |

## Architecture

```
HAL Layer          IEncoder · IAdc · IPwm · IEsc · IButton
                          ↓
System State       Single shared struct (sensors, actuator, safety flags)
                          ↓
Control Core       SolarMath (ISolarModel) → HybridTracker → TiltController
                   SafetyMonitor
                          ↓
Behavior Tree      Safety branch (overrides) / Tracking branch
                          ↓
main.cpp           20 Hz sensors+PID · 1 Hz safety · 15 min tracking · 2 Hz telemetry
```

### Tracking Modes

**Astronomical** — Frydrychowicz-Jastrzębska & Bugała modified Liu-Jordan radiation model computes the optimal tilt angle β to maximize global irradiance G_β on the panel surface. Updated every 15 minutes.

**Sensor-based** — Uses the quad photodiode array differential to fine-tune the tilt target. Engages when irradiance exceeds the minimum threshold.

Mode transitions use 2-minute hysteresis to prevent hunting.

### Control Loop

- **20 Hz:** Read AS5600 encoder + sensors, run ESC arming FSM, step PID if a move is active
- **1 Hz:** Evaluate safety flags (wind, flight lock, twilight); run behavior tree
- **15 min:** Compute solar position, select mode, issue new tilt target if delta > deadband (0.5°)
- **2 Hz:** Serial telemetry (tracker mode, PID state, safety flags, solar geometry)

### PID Controller

Anti-windup back-calculation (Kaw = 0.1), output clamped to [1100, 1900] µs. Settled when |error| < 0.5° for one cycle.

| Gain | Value |
|---|---|
| Kp | 30.0 |
| Ki | 0.5 |
| Kd | 1.2 |

### Safety Monitor

Freezes tracking and commands 0° (horizontal) when any of the following trigger:

- Wind speed > 15 m/s
- Flight lock asserted
- Twilight (normalized irradiance < 0.1) — resets overnight, re-arms at dawn

## Building

```bash
# Main tracker
pio run -e teensy41

# LDR differential tracker
pio run -e teensy41_ldr_track

# Flash to connected board
pio run -e teensy41 --target upload
```

## Tests

Pure C++ desktop tests (no Arduino dependency), run via PlatformIO native:

```bash
pio test -e native
```

Test coverage: solar math, PID controller, hybrid tracker mode switching, integration loop, edge cases (horizon, midnight, equinox), safety scenario transitions.

## Source Layout

```
src/
├── config/config.h          # All pin assignments and calibration constants
├── main.cpp                 # Main tracker entry point
├── main_wing_deploy.cpp     # Manual ESC debug harness
├── main_pin_extender.cpp    # Lock-pin actuator test
├── main_ldr_compare.cpp     # LDR pair bench logging
├── main_ldr_track.cpp       # LDR feedback PID tracker
├── core/
│   ├── solar_math           # ISolarModel: solar position + optimal tilt
│   ├── hybrid_tracker       # Mode switching + target selection
│   ├── tilt_controller      # PID servo with anti-windup
│   └── safety_monitor       # Wind/night/flight safety evaluation
├── hal/
│   ├── hal_as5600           # AS5600 magnetic encoder (I2C)
│   ├── hal_adc              # 12-bit ADC channels
│   ├── hal_pwm              # Servo PWM + ESC arming FSM
│   ├── hal_greenjay_esc     # GreenJay 2-phase arming sequence
│   ├── hal_pin_extender     # Linear actuator + limit switch
│   └── hal_button           # Debounced active-low button
├── comms/
│   ├── system_state.h       # Shared state struct + enums
│   └── comms_interface      # Serial init, telemetry, external data pull
└── bt/
    └── bt_node.h            # Hand-rolled behavior tree (no STL, static allocation)
```
