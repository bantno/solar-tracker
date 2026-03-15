// Unity integration test -- wires all modules together simulating loop()
// Run: pio test -e native -f test_integration

#include <unity.h>
#include <cstdio>
#include <cmath>
#include "core/tilt_controller.h"
#include "core/hybrid_tracker.h"
#include "core/safety_monitor.h"
#include "core/solar_math.h"
#include "comms/system_state.h"
#include "config/config.h"
#include "core/solar_model.h"

void setUp(void) {}
void tearDown(void) {}

// Mock solar model
class MockSolarModel : public ISolarModel {
public:
    SolarModelOutput fixed_output;
    MockSolarModel() {
        fixed_output.optimal_tilt_deg    = 35.0f;
        fixed_output.solar_elevation_deg = 45.0f;
        fixed_output.solar_azimuth_deg   = 180.0f;
    }
    SolarModelOutput compute(const SolarModelInput&) override {
        return fixed_output;
    }
};

// Simulate main loop timing
struct SimContext {
    SystemState      state;
    TiltController   pid;
    HybridTracker    tracker;
    SafetyMonitor    safety;

    uint32_t last_loop_ms;
    uint32_t last_safety_ms;
    uint32_t last_tracking_ms;
    uint32_t move_start_ms;

    SimContext(ISolarModel* model)
        : pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
              TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
              TILT_PID_DEADBAND_DEG, TILT_PID_KAW),
          tracker(model),
          last_loop_ms(0), last_safety_ms(0),
          last_tracking_ms(0), move_start_ms(0)
    {
        state.latitude      = DEFAULT_LATITUDE;
        state.longitude     = DEFAULT_LONGITUDE;
        state.escArmed      = true;
        state.twilight      = 0.8f;
        state.irradianceWm2 = 100.0f;
    }

    void tick(uint32_t now_ms) {
        if (now_ms - last_loop_ms >= MAIN_LOOP_INTERVAL_MS) {
            last_loop_ms = now_ms;
            if (state.moveInProgress && state.escArmed) {
                float pulse = pid.update(
                    state.tiltAngleDeg, state.targetTiltDeg, now_ms);
                float drive = (pulse - 1500.0f) / 100.0f;
                state.tiltAngleDeg += drive;
                if (pid.isSettled()) {
                    state.moveInProgress = false;
                    state.moveDuration_s =
                        (float)(now_ms - move_start_ms) / 1000.0f;
                    pid.reset();
                }
            }
        }

        if (now_ms - last_safety_ms >= SAFETY_INTERVAL_MS) {
            last_safety_ms = now_ms;
            safety.update(state, now_ms);
            if (state.safetyWindOverride || state.safetyFlightLock ||
                state.safetyNightReset  || state.safetyEscNotArmed) {
                if (state.safetyFlightLock || state.safetyEscNotArmed) {
                    state.moveInProgress = false;
                }
                pid.reset();
            }
        }

        if (now_ms - last_tracking_ms >= TRACKING_INTERVAL_MS) {
            last_tracking_ms = now_ms;
            if (!state.safetyWindOverride && !state.safetyFlightLock &&
                !state.safetyNightReset  &&  state.escArmed) {
                float prev = state.targetTiltDeg;
                tracker.update(state, now_ms);
                if (std::fabs(state.targetTiltDeg - prev) >
                    TILT_PID_DEADBAND_DEG) {
                    pid.reset();
                    move_start_ms = now_ms;
                    state.moveInProgress = true;
                }
            }
        }
    }
};

// -----------------------------------------------------------------------
// Test 1: Astronomical tracking cycle
// -----------------------------------------------------------------------
void test_astronomical_tracking_cycle(void) {
    MockSolarModel mock;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 10.0f;

    ctx.last_tracking_ms = static_cast<uint32_t>(0) - TRACKING_INTERVAL_MS;
    ctx.tick(0);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 35.0f, ctx.state.targetTiltDeg);
    TEST_ASSERT_TRUE(ctx.state.moveInProgress);

    for (uint32_t t = 50; t <= 5000; t += 50) {
        ctx.tick(t);
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 35.0f, ctx.state.tiltAngleDeg);
    TEST_ASSERT_FALSE(ctx.state.moveInProgress);
    TEST_ASSERT_TRUE(ctx.state.moveDuration_s > 0.0f);
    printf("  INFO: settled at %.2f in %.1fs\n",
           ctx.state.tiltAngleDeg, ctx.state.moveDuration_s);
}

// -----------------------------------------------------------------------
// Test 2: Sensor-based tracking
// -----------------------------------------------------------------------
void test_sensor_based_tracking(void) {
    MockSolarModel mock;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg  = 20.0f;
    ctx.state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 200.0f;
    ctx.state.quadTop       = 600;
    ctx.state.quadBottom    = 400;

    ctx.last_tracking_ms = static_cast<uint32_t>(0) - TRACKING_INTERVAL_MS;
    ctx.tick(0);

    // Advance past hysteresis, reset to known position
    uint32_t t2 = MODE_HOLD_DURATION_MS + TRACKING_INTERVAL_MS + 1;
    ctx.state.tiltAngleDeg   = 20.0f;
    ctx.state.moveInProgress = false;
    ctx.last_tracking_ms     = t2 - TRACKING_INTERVAL_MS;
    ctx.tick(t2);

    TEST_ASSERT_EQUAL_INT(
        (int)TrackingMode::SENSOR_BASED,
        (int)ctx.state.trackingMode);

    float expected = 20.0f + 200.0f * SENSOR_TRACK_GAIN;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, ctx.state.targetTiltDeg);
    printf("  INFO: target=%.2f\n", ctx.state.targetTiltDeg);
}

// -----------------------------------------------------------------------
// Test 3: Wind interruption mid-move
// -----------------------------------------------------------------------
void test_wind_interruption_mid_move(void) {
    MockSolarModel mock;
    mock.fixed_output.optimal_tilt_deg = 40.0f;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 10.0f;
    ctx.state.windSpeedMs  = 2.0f;

    ctx.last_tracking_ms = static_cast<uint32_t>(0) - TRACKING_INTERVAL_MS;
    ctx.tick(0);
    TEST_ASSERT_TRUE(ctx.state.moveInProgress);

    uint32_t t = 0;
    for (t = 50; t <= 1000; t += 50) {
        ctx.tick(t);
    }
    TEST_ASSERT_TRUE(ctx.state.moveInProgress);
    TEST_ASSERT_TRUE(ctx.state.tiltAngleDeg > 10.0f);

    // Wind spike
    ctx.state.windSpeedMs = WIND_MAX_MS + 5.0f;
    t = 1050;
    ctx.last_safety_ms = t - SAFETY_INTERVAL_MS;
    ctx.tick(t);

    TEST_ASSERT_TRUE(ctx.state.safetyWindOverride);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_WIND_SAFE_DEG,
                              ctx.state.targetTiltDeg);

    for (t += 50; t <= 10000; t += 50) {
        ctx.tick(t);
        if (!ctx.state.moveInProgress) break;
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, TILT_WIND_SAFE_DEG,
                              ctx.state.tiltAngleDeg);
    printf("  INFO: stowed at %.2f\n", ctx.state.tiltAngleDeg);
}

// -----------------------------------------------------------------------
// Test 4: Mode hysteresis
// -----------------------------------------------------------------------
void test_mode_hysteresis_oscillation(void) {
    MockSolarModel mock;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 30.0f;

    for (uint32_t cycle = 0; cycle < 10; cycle++) {
        ctx.state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
        ctx.state.quadTop = 500;
        ctx.state.quadBottom = 500;
        uint32_t t_start = cycle * 60000;
        uint32_t t = t_start;
        ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
        ctx.tick(t);

        ctx.state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
        t = t_start + 30000;
        ctx.last_tracking_ms = t - TRACKING_INTERVAL_MS;
        ctx.tick(t);

        TEST_ASSERT_EQUAL_INT(
            (int)TrackingMode::ASTRONOMICAL,
            (int)ctx.state.trackingMode);
    }
    printf("  INFO: mode stable through 10 oscillation cycles\n");
}

// -----------------------------------------------------------------------
// Test 5: Full move lifecycle
// -----------------------------------------------------------------------
void test_full_move_lifecycle(void) {
    MockSolarModel mock;
    mock.fixed_output.optimal_tilt_deg = 45.0f;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg = 0.0f;

    ctx.last_tracking_ms = static_cast<uint32_t>(0) - TRACKING_INTERVAL_MS;
    ctx.tick(0);
    TEST_ASSERT_TRUE(ctx.state.moveInProgress);

    bool settled = false;
    uint32_t t = 0;
    for (t = 50; t <= 15000; t += 50) {
        ctx.tick(t);
        if (!ctx.state.moveInProgress) {
            settled = true;
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(settled, "Should settle within 15s");
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 45.0f, ctx.state.tiltAngleDeg);

    float angle_after = ctx.state.tiltAngleDeg;
    uint32_t t_end = t + 2000;
    for (t += 50; t <= t_end; t += 50) {
        ctx.tick(t);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, angle_after, ctx.state.tiltAngleDeg);
    printf("  INFO: 0->45 in %.1fs, stable\n", ctx.state.moveDuration_s);
}

// -----------------------------------------------------------------------
// Test 6: Tracking blocked by safety
// -----------------------------------------------------------------------
void test_tracking_blocked_by_safety(void) {
    MockSolarModel mock;
    mock.fixed_output.optimal_tilt_deg = 50.0f;
    SimContext ctx(&mock);
    ctx.state.tiltAngleDeg  = 20.0f;
    ctx.state.targetTiltDeg = 20.0f;

    ctx.state.windSpeedMs = WIND_MAX_MS + 5.0f;
    ctx.last_safety_ms = static_cast<uint32_t>(0) - SAFETY_INTERVAL_MS;
    ctx.tick(0);

    ctx.last_tracking_ms = static_cast<uint32_t>(50) - TRACKING_INTERVAL_MS;
    ctx.tick(50);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_WIND_SAFE_DEG,
                              ctx.state.targetTiltDeg);
    printf("  INFO: tracking blocked during wind override\n");
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_astronomical_tracking_cycle);
    RUN_TEST(test_sensor_based_tracking);
    RUN_TEST(test_wind_interruption_mid_move);
    RUN_TEST(test_mode_hysteresis_oscillation);
    RUN_TEST(test_full_move_lifecycle);
    RUN_TEST(test_tracking_blocked_by_safety);
    return UNITY_END();
}
