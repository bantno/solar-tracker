// Desktop integration test — wires all modules together simulating loop()
// Compile: g++ -std=c++17 -I src test/test_integration.cpp
//          src/core/tilt_controller.cpp src/core/hybrid_tracker.cpp
//          src/core/safety_monitor.cpp src/core/solar_math.cpp
//          -o test_integration.exe

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "core/tilt_controller.h"
#include "core/hybrid_tracker.h"
#include "core/safety_monitor.h"
#include "core/solar_math.h"
#include "comms/system_state.h"
#include "config/config.h"
#include "core/solar_model.h"

static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT(cond, msg) do { \
    if (!(cond)) { \
        printf("  FAIL: %s\n", msg); \
        tests_failed++; \
        return; \
    } \
} while(0)

#define ASSERT_MSG(cond, fmt, ...) do { \
    if (!(cond)) { \
        printf("  FAIL: " fmt "\n", __VA_ARGS__); \
        tests_failed++; \
        return; \
    } \
} while(0)

// Mock solar model with configurable output
class MockSolarModel : public ISolarModel {
public:
    SolarModelOutput fixed = {35.0f, 45.0f, 180.0f};
    SolarModelOutput compute(const SolarModelInput&) override { return fixed; }
};

// ---------------------------------------------------------------------------
// Simulate one iteration of the main loop timing structure
// ---------------------------------------------------------------------------
struct SimContext {
    SystemState      state;
    TiltController   pid;
    HybridTracker    tracker;
    SafetyMonitor    safety;

    uint32_t last_loop_ms     = 0;
    uint32_t last_safety_ms   = 0;
    uint32_t last_tracking_ms = 0;
    uint32_t move_start_ms    = 0;

    SimContext(ISolarModel* model)
        : pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
              TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
              TILT_PID_DEADBAND_DEG, TILT_PID_KAW),
          tracker(model) {
        state.latitude  = DEFAULT_LATITUDE;
        state.longitude = DEFAULT_LONGITUDE;
        state.escArmed  = true;
        state.twilight  = 0.8f;  // daytime
        state.irradianceWm2 = 100.0f;  // below threshold → astronomical
    }

    // Run one full tick at the given wall time
    void tick(uint32_t now_ms) {
        // 20 Hz main loop
        if (now_ms - last_loop_ms >= MAIN_LOOP_INTERVAL_MS) {
            last_loop_ms = now_ms;

            // PID move
            if (state.moveInProgress && state.escArmed) {
                float pulse = pid.update(state.tiltAngleDeg, state.targetTiltDeg, now_ms);
                (void)pulse;

                // Simulate actuator: drive toward target
                float drive = (pulse - 1500.0f) / 100.0f;
                state.tiltAngleDeg += drive;

                if (pid.isSettled()) {
                    state.moveInProgress = false;
                    state.moveDuration_s = (now_ms - move_start_ms) / 1000.0f;
                    pid.reset();
                }
            }
        }

        // Safety (1 Hz)
        if (now_ms - last_safety_ms >= SAFETY_INTERVAL_MS) {
            last_safety_ms = now_ms;
            safety.update(state, now_ms);

            // Simplified BT logic: if any safety flag, block movement
            if (state.safetyWindOverride || state.safetyFlightLock ||
                state.safetyNightReset  || state.safetyEscNotArmed) {
                if (state.safetyFlightLock || state.safetyEscNotArmed) {
                    state.moveInProgress = false;
                }
                pid.reset();
            }
        }

        // Tracking (15 min — can be triggered manually by setting last_tracking_ms)
        if (now_ms - last_tracking_ms >= TRACKING_INTERVAL_MS) {
            last_tracking_ms = now_ms;
            if (!state.safetyWindOverride && !state.safetyFlightLock &&
                !state.safetyNightReset  &&  state.escArmed) {
                float prev = state.targetTiltDeg;
                tracker.update(state, now_ms);
                if (std::fabs(state.targetTiltDeg - prev) > TILT_PID_DEADBAND_DEG) {
                    pid.reset();
                    move_start_ms = now_ms;
                    state.moveInProgress = true;
                }
            }
        }
    }

    // Force a tracking update at next tick
    void forceTracking() { last_tracking_ms = 0; }
};

// ---------------------------------------------------------------------------
// Test 1: Normal astronomical tracking cycle
// Target is set by solar model, PID drives to it
// ---------------------------------------------------------------------------
static void test_astronomical_tracking_cycle() {
    printf("Test 1: Astronomical tracking cycle\n");
    MockSolarModel mock;
    mock.fixed = {35.0f, 45.0f, 180.0f};
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 10.0f;

    // Force a tracking update at t=0
    ctx.last_tracking_ms = 0 - TRACKING_INTERVAL_MS; // will trigger immediately
    ctx.tick(0);

    ASSERT(ctx.state.trackingMode == TrackingMode::ASTRONOMICAL,
           "Should be in ASTRONOMICAL mode (low irradiance)");
    ASSERT_MSG(std::fabs(ctx.state.targetTiltDeg - 35.0f) < 0.01f,
               "Target should be 35.0, got %.2f", ctx.state.targetTiltDeg);
    ASSERT(ctx.state.moveInProgress, "Move should be in progress");

    // Run PID for 5 seconds (100 ticks at 50ms)
    for (uint32_t t = 50; t <= 5000; t += 50) {
        ctx.tick(t);
    }

    ASSERT_MSG(std::fabs(ctx.state.tiltAngleDeg - 35.0f) < 1.0f,
               "Should be near target after 5s, got %.2f", ctx.state.tiltAngleDeg);
    ASSERT(!ctx.state.moveInProgress, "Move should be complete");
    ASSERT(ctx.state.moveDuration_s > 0.0f, "Move duration should be recorded");

    printf("  PASS (settled at %.2f deg in %.1fs)\n",
           ctx.state.tiltAngleDeg, ctx.state.moveDuration_s);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// Test 2: Sensor-based tracking with high irradiance
// ---------------------------------------------------------------------------
static void test_sensor_based_tracking() {
    printf("Test 2: Sensor-based tracking\n");
    MockSolarModel mock;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg  = 20.0f;
    ctx.state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 200.0f;
    ctx.state.quadTop       = 600;
    ctx.state.quadBottom    = 400;

    // Need to wait for hysteresis before mode switches
    // First trigger tracking to start the hysteresis clock
    uint32_t t = 0;
    ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
    ctx.tick(t);

    // Mode won't switch yet — advance past hysteresis
    t = MODE_HOLD_DURATION_MS + TRACKING_INTERVAL_MS + 1;
    ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
    ctx.tick(t);

    ASSERT(ctx.state.trackingMode == TrackingMode::SENSOR_BASED,
           "Should switch to SENSOR_BASED after hysteresis");

    // vertical_error = 200, gain = 0.02 → offset = 4 degrees
    float expected_target = 20.0f + 200.0f * SENSOR_TRACK_GAIN;
    ASSERT_MSG(std::fabs(ctx.state.targetTiltDeg - expected_target) < 0.1f,
               "Target should be ~%.1f, got %.2f", expected_target, ctx.state.targetTiltDeg);

    printf("  PASS (mode=SENSOR_BASED, target=%.2f)\n", ctx.state.targetTiltDeg);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// Test 3: Safety interruption mid-move — wind spike triggers stow
// ---------------------------------------------------------------------------
static void test_wind_interruption_mid_move() {
    printf("Test 3: Wind interruption mid-move\n");
    MockSolarModel mock;
    mock.fixed = {40.0f, 50.0f, 180.0f};
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 10.0f;
    ctx.state.windSpeedMs  = 2.0f;  // safe

    // Start a tracking move
    uint32_t t = 0;
    ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
    ctx.tick(t);
    ASSERT(ctx.state.moveInProgress, "Move should start");

    // Run for 1 second (partial convergence)
    for (t = 50; t <= 1000; t += 50) {
        ctx.tick(t);
    }
    float mid_move_angle = ctx.state.tiltAngleDeg;
    ASSERT(ctx.state.moveInProgress, "Should still be moving at 1s");
    ASSERT(mid_move_angle > 10.0f && mid_move_angle < 40.0f,
           "Should be partially converged");

    // Wind spike!
    ctx.state.windSpeedMs = WIND_MAX_MS + 5.0f;
    // Advance to next safety tick
    t = 1050;
    ctx.last_safety_ms = t - SAFETY_INTERVAL_MS;
    ctx.tick(t);

    ASSERT(ctx.state.safetyWindOverride, "Wind override should be set");
    ASSERT_MSG(std::fabs(ctx.state.targetTiltDeg - TILT_WIND_SAFE_DEG) < 0.01f,
               "Target should be wind-safe (%.1f), got %.2f",
               TILT_WIND_SAFE_DEG, ctx.state.targetTiltDeg);
    ASSERT(ctx.state.moveInProgress, "Should be moving to stow position");

    // Run until stowed
    for (t += 50; t <= 10000; t += 50) {
        ctx.tick(t);
        if (!ctx.state.moveInProgress) break;
    }

    ASSERT_MSG(std::fabs(ctx.state.tiltAngleDeg - TILT_WIND_SAFE_DEG) < 1.0f,
               "Should be at wind-safe angle, got %.2f", ctx.state.tiltAngleDeg);

    printf("  PASS (stowed at %.2f after wind spike)\n", ctx.state.tiltAngleDeg);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// Test 4: Mode switch with hysteresis — irradiance oscillates
// ---------------------------------------------------------------------------
static void test_mode_hysteresis_oscillation() {
    printf("Test 4: Mode hysteresis under oscillating irradiance\n");
    MockSolarModel mock;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 30.0f;

    // Start in astronomical mode (default)
    ASSERT(ctx.state.trackingMode == TrackingMode::ASTRONOMICAL, "Should start ASTRONOMICAL");

    // Oscillate irradiance above/below threshold every 30 seconds
    // Mode should NOT switch because no sustained period exceeds MODE_HOLD_DURATION_MS
    for (uint32_t cycle = 0; cycle < 10; cycle++) {
        // Above threshold for 30s
        ctx.state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
        ctx.state.quadTop = 500; ctx.state.quadBottom = 500;
        uint32_t t_start = cycle * 60000;
        for (uint32_t t = t_start; t < t_start + 30000; t += TRACKING_INTERVAL_MS) {
            ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
            ctx.tick(t);
        }

        // Below threshold for 30s
        ctx.state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
        for (uint32_t t = t_start + 30000; t < t_start + 60000; t += TRACKING_INTERVAL_MS) {
            ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
            ctx.tick(t);
        }

        ASSERT(ctx.state.trackingMode == TrackingMode::ASTRONOMICAL,
               "Mode should NOT switch during rapid oscillation");
    }

    printf("  PASS (mode stable through 10 oscillation cycles)\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// Test 5: Full cycle — astronomical track → move → settle → idle
// Verify moveDuration_s and final state
// ---------------------------------------------------------------------------
static void test_full_move_lifecycle() {
    printf("Test 5: Full move lifecycle\n");
    MockSolarModel mock;
    mock.fixed = {45.0f, 60.0f, 190.0f};
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 0.0f;

    // Trigger tracking
    uint32_t t = 0;
    ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
    ctx.tick(t);

    ASSERT(ctx.state.moveInProgress, "Move should start");
    uint32_t move_started = t;

    // Run until settled or timeout
    bool settled = false;
    for (t = 50; t <= 15000; t += 50) {
        ctx.tick(t);
        if (!ctx.state.moveInProgress) {
            settled = true;
            break;
        }
    }

    ASSERT(settled, "PID should settle within 15 seconds");
    ASSERT(ctx.state.moveDuration_s > 0.0f, "Duration should be recorded");
    ASSERT_MSG(std::fabs(ctx.state.tiltAngleDeg - 45.0f) < 1.0f,
               "Should be near 45.0, got %.2f", ctx.state.tiltAngleDeg);

    // After settling: no more driving
    float angle_after = ctx.state.tiltAngleDeg;
    for (t += 50; t <= t + 2000; t += 50) {
        ctx.tick(t);
    }
    ASSERT_MSG(std::fabs(ctx.state.tiltAngleDeg - angle_after) < 0.01f,
               "Angle should not drift after settling (%.4f vs %.4f)",
               angle_after, ctx.state.tiltAngleDeg);

    printf("  PASS (0 -> 45 deg in %.1fs, stable after settle)\n",
           ctx.state.moveDuration_s);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// Test 6: Tracking blocked by safety flags — no target update
// ---------------------------------------------------------------------------
static void test_tracking_blocked_by_safety() {
    printf("Test 6: Tracking blocked by safety flags\n");
    MockSolarModel mock;
    mock.fixed = {50.0f, 40.0f, 200.0f};
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 20.0f;
    ctx.state.targetTiltDeg = 20.0f;

    // Set wind override
    ctx.state.windSpeedMs = WIND_MAX_MS + 5.0f;
    ctx.last_safety_ms = 0 - SAFETY_INTERVAL_MS;
    ctx.tick(0);  // safety fires, sets wind override

    // Try to trigger tracking
    float prev_target = ctx.state.targetTiltDeg;
    ctx.last_tracking_ms = 50 - TRACKING_INTERVAL_MS;
    ctx.tick(50);

    // Target should be wind-safe (set by safety), NOT the mock model's 50 deg
    ASSERT_MSG(std::fabs(ctx.state.targetTiltDeg - TILT_WIND_SAFE_DEG) < 0.01f,
               "Target should be wind-safe, got %.2f", ctx.state.targetTiltDeg);

    printf("  PASS (tracking correctly blocked during wind override)\n");
    tests_passed++;
}

int main() {
    printf("=== Integration Tests ===\n\n");

    test_astronomical_tracking_cycle();
    test_sensor_based_tracking();
    test_wind_interruption_mid_move();
    test_mode_hysteresis_oscillation();
    test_full_move_lifecycle();
    test_tracking_blocked_by_safety();

    printf("\n--- Results: %d passed, %d failed ---\n",
           tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
