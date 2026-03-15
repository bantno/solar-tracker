// Unity safety scenario tests -- end-to-end state transitions
// Run: pio test -e native -f test_safety_scenarios

#include <unity.h>
#include <cmath>
#include <cstdio>
#include "core/safety_monitor.h"
#include "comms/system_state.h"
#include "config/config.h"

void setUp(void) {}
void tearDown(void) {}

// Helper: clear all safety-related state
static void reset_state(SystemState& s) {
    s.windSpeedMs  = 2.0f;
    s.inFlight     = false;
    s.twilight     = 0.8f;
    s.escArmed     = true;
    s.safetyWindOverride = false;
    s.safetyFlightLock   = false;
    s.safetyNightReset   = false;
    s.safetyEscNotArmed  = false;
    s.moveInProgress     = false;
    s.targetTiltDeg      = 30.0f;
    s.tiltAngleDeg       = 30.0f;
}

// 1. Wind spike -> stow -> wind drops -> resume
void test_wind_spike_and_recovery() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    safety.update(state, 0);
    TEST_ASSERT_FALSE(state.safetyWindOverride);

    state.windSpeedMs = WIND_MAX_MS + 5.0f;
    safety.update(state, 1000);
    TEST_ASSERT_TRUE(state.safetyWindOverride);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_WIND_SAFE_DEG, state.targetTiltDeg);
    TEST_ASSERT_TRUE(state.moveInProgress);

    state.tiltAngleDeg = TILT_WIND_SAFE_DEG;
    state.moveInProgress = false;

    state.windSpeedMs = WIND_MAX_MS + 1.0f;
    safety.update(state, 2000);
    TEST_ASSERT_TRUE(state.safetyWindOverride);

    state.windSpeedMs = WIND_MAX_MS - 2.0f;
    safety.update(state, 3000);
    TEST_ASSERT_FALSE(state.safetyWindOverride);
}

// 2. Day -> twilight -> night reset -> dawn restart
void test_day_night_cycle() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);
    state.targetTiltDeg = 35.0f;

    state.twilight = 0.8f;
    safety.update(state, 0);
    TEST_ASSERT_FALSE(state.safetyNightReset);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 35.0f, state.targetTiltDeg);

    state.twilight = 0.15f;
    safety.update(state, 1000);
    TEST_ASSERT_FALSE(state.safetyNightReset);

    state.twilight = 0.05f;
    safety.update(state, 2000);
    TEST_ASSERT_TRUE(state.safetyNightReset);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_NIGHT_RESET_DEG, state.targetTiltDeg);
    TEST_ASSERT_TRUE(state.moveInProgress);

    state.moveInProgress = false;
    state.targetTiltDeg  = TILT_NIGHT_RESET_DEG;
    safety.update(state, 3000);
    TEST_ASSERT_FALSE_MESSAGE(state.moveInProgress,
        "Should NOT re-trigger move on subsequent night ticks (no edge)");

    state.twilight = 0.5f;
    safety.update(state, 4000);
    TEST_ASSERT_FALSE(state.safetyNightReset);

    state.twilight = 0.05f;
    safety.update(state, 5000);
    TEST_ASSERT_TRUE(state.safetyNightReset);
    TEST_ASSERT_TRUE_MESSAGE(state.moveInProgress, "Move triggered on new nightfall edge");
}

// 3. In-flight lock -> landing -> resume
void test_flight_lock() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    safety.update(state, 0);
    TEST_ASSERT_FALSE(state.safetyFlightLock);

    state.inFlight = true;
    safety.update(state, 1000);
    TEST_ASSERT_TRUE(state.safetyFlightLock);

    state.inFlight = false;
    safety.update(state, 2000);
    TEST_ASSERT_FALSE(state.safetyFlightLock);
}

// 4. Multiple simultaneous safety flags
void test_multiple_flags() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    state.windSpeedMs = WIND_MAX_MS + 10.0f;
    state.inFlight    = true;
    state.twilight    = 0.01f;
    state.escArmed    = false;

    safety.update(state, 0);

    TEST_ASSERT_TRUE(state.safetyWindOverride);
    TEST_ASSERT_TRUE(state.safetyFlightLock);
    TEST_ASSERT_TRUE(state.safetyNightReset);
    TEST_ASSERT_TRUE(state.safetyEscNotArmed);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_WIND_SAFE_DEG, state.targetTiltDeg);

    // Clear flags one by one
    state.windSpeedMs = 2.0f;
    safety.update(state, 1000);
    TEST_ASSERT_FALSE(state.safetyWindOverride);
    TEST_ASSERT_TRUE(state.safetyFlightLock);
    TEST_ASSERT_TRUE(state.safetyNightReset);
    TEST_ASSERT_TRUE(state.safetyEscNotArmed);

    state.inFlight = false;
    safety.update(state, 2000);
    TEST_ASSERT_FALSE(state.safetyFlightLock);

    state.escArmed = true;
    safety.update(state, 3000);
    TEST_ASSERT_FALSE(state.safetyEscNotArmed);

    state.twilight = 0.5f;
    safety.update(state, 4000);
    TEST_ASSERT_FALSE(state.safetyNightReset);

    TEST_ASSERT_FALSE(state.safetyWindOverride);
    TEST_ASSERT_FALSE(state.safetyFlightLock);
    TEST_ASSERT_FALSE(state.safetyNightReset);
    TEST_ASSERT_FALSE(state.safetyEscNotArmed);
}

// 5. ESC not armed -> armed transition
void test_esc_arming_transition() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);
    state.escArmed = false;

    safety.update(state, 0);
    TEST_ASSERT_TRUE(state.safetyEscNotArmed);

    state.escArmed = true;
    safety.update(state, 1000);
    TEST_ASSERT_FALSE(state.safetyEscNotArmed);

    state.escArmed = false;
    safety.update(state, 2000);
    TEST_ASSERT_TRUE(state.safetyEscNotArmed);
}

// 6. Night reset edge detection -- only triggers on transition
void test_night_reset_edge_detection() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    state.twilight = 0.05f;
    safety.update(state, 0);
    TEST_ASSERT_TRUE(state.safetyNightReset);
    TEST_ASSERT_TRUE(state.moveInProgress);

    state.moveInProgress = false;
    state.targetTiltDeg = 15.0f;

    for (int i = 0; i < 5; i++) {
        safety.update(state, 1000 + i * 1000);
        TEST_ASSERT_FALSE_MESSAGE(state.moveInProgress,
            "Should NOT re-trigger move during sustained night");
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, 15.0f, state.targetTiltDeg,
            "Target should NOT change during sustained night");
    }

    state.twilight = 0.5f;
    safety.update(state, 10000);
    TEST_ASSERT_FALSE(state.safetyNightReset);

    state.twilight = 0.05f;
    state.targetTiltDeg = 25.0f;
    safety.update(state, 11000);
    TEST_ASSERT_TRUE(state.moveInProgress);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_NIGHT_RESET_DEG, state.targetTiltDeg);
}

// 7. Wind override is continuous (not edge-triggered)
void test_wind_override_continuous() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    state.windSpeedMs = WIND_MAX_MS + 3.0f;

    for (int i = 0; i < 5; i++) {
        state.targetTiltDeg = 40.0f;
        state.moveInProgress = false;
        safety.update(state, i * 1000);
        TEST_ASSERT_TRUE(state.safetyWindOverride);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, TILT_WIND_SAFE_DEG, state.targetTiltDeg);
        TEST_ASSERT_TRUE_MESSAGE(state.moveInProgress,
            "moveInProgress set each tick during wind override");
    }
}

// 8. Boundary values -- exactly at thresholds
void test_boundary_values() {
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    state.windSpeedMs = WIND_MAX_MS;
    safety.update(state, 0);
    TEST_ASSERT_FALSE_MESSAGE(state.safetyWindOverride,
        "Wind exactly at threshold should NOT trigger override");

    state.windSpeedMs = WIND_MAX_MS + 0.001f;
    safety.update(state, 1000);
    TEST_ASSERT_TRUE_MESSAGE(state.safetyWindOverride,
        "Wind just above threshold should trigger");

    SafetyMonitor safety2;
    SystemState state2;
    reset_state(state2);
    state2.twilight = TWILIGHT_THRESHOLD;
    safety2.update(state2, 0);
    TEST_ASSERT_FALSE_MESSAGE(state2.safetyNightReset,
        "Twilight exactly at threshold should NOT trigger night reset");

    state2.twilight = TWILIGHT_THRESHOLD - 0.001f;
    safety2.update(state2, 1000);
    TEST_ASSERT_TRUE_MESSAGE(state2.safetyNightReset,
        "Twilight just below threshold should trigger night reset");
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_wind_spike_and_recovery);
    RUN_TEST(test_day_night_cycle);
    RUN_TEST(test_flight_lock);
    RUN_TEST(test_multiple_flags);
    RUN_TEST(test_esc_arming_transition);
    RUN_TEST(test_night_reset_edge_detection);
    RUN_TEST(test_wind_override_continuous);
    RUN_TEST(test_boundary_values);
    return UNITY_END();
}
