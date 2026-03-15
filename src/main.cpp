#include <Arduino.h>

#include "config/config.h"
#include "comms/system_state.h"
#include "comms/comms_interface.h"
#include "hal/hal_as5600.h"
#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
#include "hal/hal_pin_motor.h"
#include "core/solar_math.h"
#include "core/hybrid_tracker.h"
#include "core/tilt_controller.h"
#include "core/safety_monitor.h"
#include "bt/bt_node.h"

// --- HAL instances (concrete implementations) ---
static HalAs5600   encoder;
static HalAdc      adc;
static HalPwm      pwm;
static HalPinMotor pinMotor;

// --- System state ---
static SystemState state;

// --- Core modules ---
static SolarMath      solar_math;
static HybridTracker  hybrid_tracker(&solar_math);
static TiltController tilt_controller(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                                      TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                                      TILT_PID_DEADBAND_DEG, TILT_PID_KAW);
static SafetyMonitor  safety_monitor;

// --- BT callback functions ---
static bool any_safety_flag_set(const SystemState& s) {
    return s.safetyWindOverride || s.safetyFlightLock ||
           s.safetyNightReset  || s.safetyEscNotArmed;
}

static NodeStatus handle_safety_override(SystemState& s) {
    // Flight lock: freeze — do not move
    if (s.safetyFlightLock) {
        s.moveInProgress = false;
        pwm.setNeutral();
    }
    // ESC not armed: cannot move
    if (s.safetyEscNotArmed) {
        s.moveInProgress = false;
    }
    // Wind override and night reset targets are already set by SafetyMonitor
    tilt_controller.reset();
    return NodeStatus::RUNNING;
}

static bool esc_is_armed(const SystemState& s) {
    return s.escArmed;
}

static NodeStatus permit_tracking(SystemState& /*s*/) {
    // No-op: tracking is driven by timers in loop()
    return NodeStatus::SUCCESS;
}

// --- BT nodes (static allocation) ---
static Condition cond_any_safety_flag(any_safety_flag_set);
static Action    act_safety_override(handle_safety_override);
static Condition cond_esc_armed(esc_is_armed);
static Action    act_permit_tracking(permit_tracking);
static Sequence  safety_branch;
static Sequence  tracking_branch;
static Selector  root_bt;

// --- Timing ---
static uint32_t lastLoopMs     = 0;
static uint32_t lastPrintMs    = 0;
static uint32_t lastSafetyMs   = 0;
static uint32_t lastTrackingMs = 0;
static uint32_t moveStartMs    = 0;

// --- Telemetry ---
static void print_telemetry(const SystemState& s) {
    // [TRACKER] line
    Serial.print("[TRACKER] mode=");
    Serial.print(s.trackingMode == TrackingMode::ASTRONOMICAL ? "ASTRONOMICAL" : "SENSOR_BASED");
    Serial.print("  target=");
    Serial.print(s.targetTiltDeg, 1);
    Serial.print("\xC2\xB0  current=");
    Serial.print(s.tiltAngleDeg, 1);
    Serial.print("\xC2\xB0  move=");
    Serial.print(s.moveInProgress ? "ACTIVE" : "IDLE");
    Serial.print("  dur=");
    Serial.print(s.moveDuration_s, 1);
    Serial.println("s");

    // [PID] line
    Serial.print("[PID]     err=");
    float err = tilt_controller.getError();
    if (err >= 0.0f) Serial.print("+");
    Serial.print(err, 1);
    Serial.print("\xC2\xB0  integral=");
    Serial.print(tilt_controller.getIntegral(), 2);
    Serial.print("  deriv=");
    Serial.print(tilt_controller.getLastDerivative(), 2);
    Serial.print("  output=");
    Serial.print((int)tilt_controller.getLastOutput());
    Serial.println("us");

    // [SAFETY] line
    Serial.print("[SAFETY]  ");
    Serial.print(s.safetyWindOverride ? "W+" : "w-");
    Serial.print("  ");
    Serial.print(s.safetyFlightLock   ? "F+" : "f-");
    Serial.print("  ");
    Serial.print(s.safetyNightReset   ? "N+" : "n-");
    Serial.print("  ");
    Serial.println(s.escArmed          ? "A+" : "a-");

    // [SOLAR] line
    Serial.print("[SOLAR]   elev=");
    Serial.print(s.solarElevationDeg, 1);
    Serial.print("\xC2\xB0  azim=");
    Serial.print(s.solarAzimuthDeg, 1);
    Serial.println("\xC2\xB0");

    // [SENSOR] line
    Serial.print("[SENSOR]  v_err=");
    if (s.verticalSensorError >= 0.0f) Serial.print("+");
    Serial.print((int)s.verticalSensorError);
    Serial.print("  h_err=");
    if (s.horizontalSensorError >= 0.0f) Serial.print("+");
    Serial.print((int)s.horizontalSensorError);
    Serial.print("  irr=");
    Serial.print((int)s.irradianceWm2);
    Serial.print("W/m\xC2\xB2  wind=");
    Serial.print(s.windSpeedMs, 1);
    Serial.println("m/s");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait up to 3 s for Serial (USB enumeration)
    }
    Serial.println("=== Solar Wing Tracker — Milestone 3 ===");

    encoder.init();
    adc.init();
    pwm.init();
    pinMotor.init();
    commsInit();

    // Build behavior tree
    safety_branch.addChild(&cond_any_safety_flag);
    safety_branch.addChild(&act_safety_override);
    tracking_branch.addChild(&cond_esc_armed);
    tracking_branch.addChild(&act_permit_tracking);
    root_bt.addChild(&safety_branch);
    root_bt.addChild(&tracking_branch);

    // Set default location
    state.latitude  = DEFAULT_LATITUDE;
    state.longitude = DEFAULT_LONGITUDE;

    Serial.println("[INIT] All modules initialized");
}

void loop() {
    uint32_t now = millis();

    // 1. 20 Hz: sensor reads + PID move tick
    if ((now - lastLoopMs) >= MAIN_LOOP_INTERVAL_MS) {
        lastLoopMs = now;

        // Read encoder
        state.tiltAngleDeg = encoder.readAngleDegrees();
        state.encoderRaw   = encoder.readRawCounts();

        // Read quad sensor
        state.quadTop    = adc.readQuadSensor(QuadChannel::TOP);
        state.quadBottom = adc.readQuadSensor(QuadChannel::BOTTOM);
        state.quadLeft   = adc.readQuadSensor(QuadChannel::LEFT);
        state.quadRight  = adc.readQuadSensor(QuadChannel::RIGHT);

        // Read environmental sensors
        state.irradianceWm2 = adc.readIrradiance();
        state.windSpeedMs   = adc.readWindSpeed();
        state.twilight      = adc.readTwilight();

        // Update ESC arm state machine
        pwm.arm(now);
        state.escArmed = pwm.isArmed();

        // Pull external data (time, GPS, flight mode)
        commsUpdate(&state);

        // PID move — runs only while a move is in progress
        if (state.moveInProgress && state.escArmed) {
            float pulse_us = tilt_controller.update(
                state.tiltAngleDeg, state.targetTiltDeg, now);
            pwm.setTiltPulseWidth(static_cast<uint16_t>(pulse_us));

            if (tilt_controller.isSettled()) {
                state.moveInProgress = false;
                state.moveDuration_s = (now - moveStartMs) / 1000.0f;
                pwm.setNeutral();
                tilt_controller.reset();
            }
        }
    }

    // 2. Safety interval (default 1 Hz)
    if ((now - lastSafetyMs) >= SAFETY_INTERVAL_MS) {
        lastSafetyMs = now;
        safety_monitor.update(state, now);
        root_bt.tick(state);
    }

    // 3. Tracking interval (default 15 min)
    if ((now - lastTrackingMs) >= TRACKING_INTERVAL_MS) {
        lastTrackingMs = now;
        // Only compute new target if no safety flag is blocking
        if (!state.safetyWindOverride && !state.safetyFlightLock &&
            !state.safetyNightReset   &&  state.escArmed) {
            float prev_target = state.targetTiltDeg;
            hybrid_tracker.update(state, now);
            // If target changed meaningfully, begin a new move
            if (fabsf(state.targetTiltDeg - prev_target) > TILT_PID_DEADBAND_DEG) {
                tilt_controller.reset();
                moveStartMs = now;
                state.moveInProgress = true;
            }
        }
    }

    // 4. 2 Hz telemetry
    if ((now - lastPrintMs) >= SERIAL_PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        print_telemetry(state);
    }
}
