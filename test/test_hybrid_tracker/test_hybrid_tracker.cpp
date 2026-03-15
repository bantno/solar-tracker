// Unity test for HybridTracker
// Run: pio test -e native -f test_hybrid_tracker

#include <unity.h>
#include <cmath>
#include <cstdio>
#include "core/hybrid_tracker.h"
#include "core/solar_model.h"
#include "config/config.h"

void setUp(void) {}
void tearDown(void) {}

// Mock solar model returning configurable fixed output
class MockSolarModel : public ISolarModel {
public:
    SolarModelOutput fixed_output = {30.0f, 45.0f, 180.0f};

    SolarModelOutput compute(const SolarModelInput& /*input*/) override {
        return fixed_output;
    }
};

// 1. Mode selection based on irradiance
void test_mode_selection() {
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;

    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.tiltAngleDeg = 20.0f;
    state.quadTop = 600; state.quadBottom = 400;

    uint32_t now = 0;
    tracker.update(state, now);
    now = MODE_HOLD_DURATION_MS + 1;
    tracker.update(state, now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(TrackingMode::SENSOR_BASED),
                          static_cast<int>(state.trackingMode));

    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
    now += 1;
    tracker.update(state, now);
    now += MODE_HOLD_DURATION_MS + 1;
    tracker.update(state, now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(TrackingMode::ASTRONOMICAL),
                          static_cast<int>(state.trackingMode));
}

// 2. Mode hysteresis
void test_mode_hysteresis() {
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 20.0f;

    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.quadTop = 500; state.quadBottom = 500;

    for (uint32_t t = 0; t < MODE_HOLD_DURATION_MS; t += 10000) {
        tracker.update(state, t);
        TEST_ASSERT_EQUAL_INT_MESSAGE(
            static_cast<int>(TrackingMode::ASTRONOMICAL),
            static_cast<int>(state.trackingMode),
            "Mode should NOT switch before hold period elapses");
    }

    tracker.update(state, MODE_HOLD_DURATION_MS + 1);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(TrackingMode::SENSOR_BASED),
                          static_cast<int>(state.trackingMode));
}

// 3. Sensor-based target
void test_sensor_based_target() {
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 20.0f;

    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.quadTop = 600; state.quadBottom = 400;
    tracker.update(state, 0);
    tracker.update(state, MODE_HOLD_DURATION_MS + 1);

    state.tiltAngleDeg = 20.0f;
    state.quadTop = 600; state.quadBottom = 400;
    tracker.update(state, MODE_HOLD_DURATION_MS + 2);

    float expected = 20.0f + 200.0f * SENSOR_TRACK_GAIN;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, state.targetTiltDeg);
    printf("  INFO: target=%.2f\n", state.targetTiltDeg);
}

// 4. Astronomical target
void test_astronomical_target() {
    MockSolarModel mock;
    mock.fixed_output = {42.5f, 55.0f, 200.0f};
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 10.0f;

    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
    tracker.update(state, 0);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.5f, state.targetTiltDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 55.0f, state.solarElevationDeg);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, state.solarAzimuthDeg);
}

// 5. Clamping
void test_clamping() {
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 80.0f;

    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.quadTop = 4095; state.quadBottom = 0;
    tracker.update(state, 0);
    tracker.update(state, MODE_HOLD_DURATION_MS + 1);

    tracker.update(state, MODE_HOLD_DURATION_MS + 2);

    char buf[256];
    snprintf(buf, sizeof(buf), "Target %.2f exceeds max %.2f",
             state.targetTiltDeg, TILT_BETA_MAX_DEG);
    TEST_ASSERT_TRUE_MESSAGE(state.targetTiltDeg <= TILT_BETA_MAX_DEG, buf);

    snprintf(buf, sizeof(buf), "Target %.2f below min %.2f",
             state.targetTiltDeg, TILT_BETA_MIN_DEG);
    TEST_ASSERT_TRUE_MESSAGE(state.targetTiltDeg >= TILT_BETA_MIN_DEG, buf);

    printf("  INFO: target=%.2f, max=%.2f\n", state.targetTiltDeg, TILT_BETA_MAX_DEG);
}

// 6. moveInProgress trigger
void test_move_in_progress() {
    MockSolarModel mock;
    mock.fixed_output = {30.0f, 45.0f, 180.0f};
    HybridTracker tracker(&mock);
    SystemState state;

    state.tiltAngleDeg = 10.0f;
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
    state.moveInProgress = false;
    tracker.update(state, 0);
    TEST_ASSERT_TRUE_MESSAGE(state.moveInProgress,
        "moveInProgress should be true when target differs significantly");

    state.tiltAngleDeg = 29.8f;
    state.moveInProgress = false;
    mock.fixed_output.optimal_tilt_deg = 30.0f;
    tracker.update(state, 100);
    TEST_ASSERT_FALSE_MESSAGE(state.moveInProgress,
        "moveInProgress should remain false when within deadband");
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_mode_selection);
    RUN_TEST(test_mode_hysteresis);
    RUN_TEST(test_sensor_based_target);
    RUN_TEST(test_astronomical_target);
    RUN_TEST(test_clamping);
    RUN_TEST(test_move_in_progress);
    return UNITY_END();
}
