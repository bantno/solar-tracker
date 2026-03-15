// Desktop test for HybridTracker
// Compile: g++ -std=c++17 -I src test/test_hybrid_tracker.cpp
//          src/core/hybrid_tracker.cpp src/core/solar_math.cpp -o test_tracker

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "core/hybrid_tracker.h"
#include "core/solar_model.h"
#include "config/config.h"

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

// Mock solar model returning configurable fixed output
class MockSolarModel : public ISolarModel {
public:
    SolarModelOutput fixed_output = {30.0f, 45.0f, 180.0f};

    SolarModelOutput compute(const SolarModelInput& /*input*/) override {
        return fixed_output;
    }
};

// 1. Mode selection based on irradiance
static void test_mode_selection() {
    printf("Test 1: Mode selection\n");
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;

    // High irradiance → SENSOR_BASED (after hysteresis)
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.tiltAngleDeg = 20.0f;
    state.quadTop = 600; state.quadBottom = 400;

    // Need to wait for MODE_HOLD_DURATION_MS for mode switch
    uint32_t now = 0;
    tracker.update(state, now);
    // First call: still ASTRONOMICAL (hysteresis not elapsed)
    // Advance past hold duration
    now = MODE_HOLD_DURATION_MS + 1;
    tracker.update(state, now);
    ASSERT(state.trackingMode == TrackingMode::SENSOR_BASED,
           "High irradiance should switch to SENSOR_BASED after hold period");

    // Low irradiance → ASTRONOMICAL (after hysteresis)
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
    now += 1;
    tracker.update(state, now);
    // Still SENSOR_BASED — hysteresis
    now += MODE_HOLD_DURATION_MS + 1;
    tracker.update(state, now);
    ASSERT(state.trackingMode == TrackingMode::ASTRONOMICAL,
           "Low irradiance should switch to ASTRONOMICAL after hold period");

    printf("  PASS\n");
    tests_passed++;
}

// 2. Mode hysteresis: mode does not switch until hold period elapses
static void test_mode_hysteresis() {
    printf("Test 2: Mode hysteresis\n");
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 20.0f;

    // Start in ASTRONOMICAL (default), set high irradiance
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.quadTop = 500; state.quadBottom = 500;

    // Tick several times before hold period
    for (uint32_t t = 0; t < MODE_HOLD_DURATION_MS; t += 10000) {
        tracker.update(state, t);
        ASSERT(state.trackingMode == TrackingMode::ASTRONOMICAL,
               "Mode should NOT switch before hold period elapses");
    }

    // Now exceed hold period
    tracker.update(state, MODE_HOLD_DURATION_MS + 1);
    ASSERT(state.trackingMode == TrackingMode::SENSOR_BASED,
           "Mode should switch after hold period");

    printf("  PASS\n");
    tests_passed++;
}

// 3. Sensor-based target: quadTop > quadBottom → target increases
static void test_sensor_based_target() {
    printf("Test 3: Sensor-based target\n");
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 20.0f;

    // Force into SENSOR_BASED mode (switch + wait for hysteresis)
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.quadTop = 600; state.quadBottom = 400;
    tracker.update(state, 0);
    tracker.update(state, MODE_HOLD_DURATION_MS + 1);

    // Now test: quadTop=600, quadBottom=400 → positive vertical error
    float prev_target = state.targetTiltDeg;
    // Reset to known state
    state.tiltAngleDeg = 20.0f;
    state.quadTop = 600; state.quadBottom = 400;
    tracker.update(state, MODE_HOLD_DURATION_MS + 2);

    // vertical_error = 600 - 400 = 200, target = 20 + 200*0.02 = 24
    float expected = 20.0f + 200.0f * SENSOR_TRACK_GAIN;
    ASSERT_MSG(std::fabs(state.targetTiltDeg - expected) < 0.01f,
               "Expected target=%.2f, got %.2f", expected, state.targetTiltDeg);

    printf("  PASS (target=%.2f)\n", state.targetTiltDeg);
    tests_passed++;
}

// 4. Astronomical target: uses mock solar model output
static void test_astronomical_target() {
    printf("Test 4: Astronomical target\n");
    MockSolarModel mock;
    mock.fixed_output = {42.5f, 55.0f, 200.0f};
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 10.0f;

    // Low irradiance → ASTRONOMICAL (default mode)
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
    tracker.update(state, 0);

    ASSERT_MSG(std::fabs(state.targetTiltDeg - 42.5f) < 0.01f,
               "Expected target=42.5, got %.2f", state.targetTiltDeg);
    ASSERT_MSG(std::fabs(state.solarElevationDeg - 55.0f) < 0.01f,
               "Expected elevation=55.0, got %.2f", state.solarElevationDeg);
    ASSERT_MSG(std::fabs(state.solarAzimuthDeg - 200.0f) < 0.01f,
               "Expected azimuth=200.0, got %.2f", state.solarAzimuthDeg);

    printf("  PASS\n");
    tests_passed++;
}

// 5. Clamping: extreme sensor values don't exceed TILT_BETA_MAX_DEG
static void test_clamping() {
    printf("Test 5: Clamping\n");
    MockSolarModel mock;
    HybridTracker tracker(&mock);
    SystemState state;
    state.tiltAngleDeg = 80.0f;

    // Force SENSOR_BASED
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 + 100.0f;
    state.quadTop = 4095; state.quadBottom = 0;
    tracker.update(state, 0);
    tracker.update(state, MODE_HOLD_DURATION_MS + 1);

    // quadTop=4095, quadBottom=0 → vertical_error=4095
    // target = 80 + 4095*0.02 = 161.9 → clamped to 90
    tracker.update(state, MODE_HOLD_DURATION_MS + 2);

    ASSERT_MSG(state.targetTiltDeg <= TILT_BETA_MAX_DEG,
               "Target %.2f exceeds max %.2f", state.targetTiltDeg, TILT_BETA_MAX_DEG);
    ASSERT_MSG(state.targetTiltDeg >= TILT_BETA_MIN_DEG,
               "Target %.2f below min %.2f", state.targetTiltDeg, TILT_BETA_MIN_DEG);

    printf("  PASS (target=%.2f, max=%.2f)\n", state.targetTiltDeg, TILT_BETA_MAX_DEG);
    tests_passed++;
}

// 6. moveInProgress trigger
static void test_move_in_progress() {
    printf("Test 6: moveInProgress trigger\n");
    MockSolarModel mock;
    mock.fixed_output = {30.0f, 45.0f, 180.0f};
    HybridTracker tracker(&mock);
    SystemState state;

    // Case 1: target differs significantly from current → moveInProgress = true
    state.tiltAngleDeg = 10.0f;
    state.irradianceWm2 = IRRADIANCE_THRESHOLD_WM2 - 50.0f;
    state.moveInProgress = false;
    tracker.update(state, 0);
    // target=30 vs current=10: diff=20 > deadband
    ASSERT(state.moveInProgress == true,
           "moveInProgress should be true when target differs significantly");

    // Case 2: target within deadband of current → moveInProgress unchanged
    state.tiltAngleDeg = 29.8f;  // Within 0.5 of target 30
    state.moveInProgress = false;
    mock.fixed_output.optimal_tilt_deg = 30.0f;
    tracker.update(state, 100);
    ASSERT(state.moveInProgress == false,
           "moveInProgress should remain false when within deadband");

    printf("  PASS\n");
    tests_passed++;
}

int main() {
    printf("=== HybridTracker Tests ===\n\n");

    test_mode_selection();
    test_mode_hysteresis();
    test_sensor_based_target();
    test_astronomical_target();
    test_clamping();
    test_move_in_progress();

    printf("\n--- Results: %d passed, %d failed ---\n",
           tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
