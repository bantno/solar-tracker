// Desktop safety scenario tests — end-to-end state transitions
// Compile: g++ -std=c++17 -I src test/test_safety_scenarios.cpp
//          src/core/safety_monitor.cpp -o test_safety_scenarios.exe

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "core/safety_monitor.h"
#include "comms/system_state.h"
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

// ---------------------------------------------------------------------------
// 1. Wind spike → stow → wind drops → resume
// ---------------------------------------------------------------------------
static void test_wind_spike_and_recovery() {
    printf("Test 1: Wind spike -> stow -> wind drops -> resume\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    // Normal conditions
    safety.update(state, 0);
    ASSERT(!state.safetyWindOverride, "No wind override initially");

    // Wind spike
    state.windSpeedMs = WIND_MAX_MS + 5.0f;
    safety.update(state, 1000);
    ASSERT(state.safetyWindOverride, "Wind override should be set");
    ASSERT_MSG(std::fabs(state.targetTiltDeg - TILT_WIND_SAFE_DEG) < 0.01f,
               "Target should be wind-safe (%.1f), got %.2f",
               TILT_WIND_SAFE_DEG, state.targetTiltDeg);
    ASSERT(state.moveInProgress, "Move should be initiated for stow");

    // Simulate stow complete
    state.tiltAngleDeg = TILT_WIND_SAFE_DEG;
    state.moveInProgress = false;

    // Wind drops — still above threshold one more tick
    state.windSpeedMs = WIND_MAX_MS + 1.0f;
    safety.update(state, 2000);
    ASSERT(state.safetyWindOverride, "Still overriding above threshold");

    // Wind drops below threshold
    state.windSpeedMs = WIND_MAX_MS - 2.0f;
    safety.update(state, 3000);
    ASSERT(!state.safetyWindOverride, "Override should clear when wind drops");
    // Tracking can now resume (moveInProgress not set by safety when clearing)

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 2. Day → twilight → night reset → dawn restart
// ---------------------------------------------------------------------------
static void test_day_night_cycle() {
    printf("Test 2: Day -> twilight -> night reset -> dawn\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);
    state.targetTiltDeg = 35.0f;

    // Daytime
    state.twilight = 0.8f;
    safety.update(state, 0);
    ASSERT(!state.safetyNightReset, "No night reset during day");
    ASSERT_MSG(std::fabs(state.targetTiltDeg - 35.0f) < 0.01f,
               "Target unchanged during day, got %.2f", state.targetTiltDeg);

    // Twilight fading
    state.twilight = 0.15f;
    safety.update(state, 1000);
    ASSERT(!state.safetyNightReset, "Twilight above threshold — no reset");

    // Night falls (below threshold) — should trigger night reset
    state.twilight = 0.05f;
    safety.update(state, 2000);
    ASSERT(state.safetyNightReset, "Night reset should be set");
    ASSERT_MSG(std::fabs(state.targetTiltDeg - TILT_NIGHT_RESET_DEG) < 0.01f,
               "Target should be night-reset angle (%.1f), got %.2f",
               TILT_NIGHT_RESET_DEG, state.targetTiltDeg);
    ASSERT(state.moveInProgress, "Move should start for night reset");

    // Still night — subsequent ticks should NOT re-trigger move
    state.moveInProgress = false;  // simulate move complete
    state.targetTiltDeg  = TILT_NIGHT_RESET_DEG;
    safety.update(state, 3000);
    ASSERT(!state.moveInProgress,
           "Should NOT re-trigger move on subsequent night ticks (no edge)");

    // Dawn — twilight rises above threshold
    state.twilight = 0.5f;
    safety.update(state, 4000);
    ASSERT(!state.safetyNightReset, "Night reset should clear at dawn");

    // Next night should trigger again (edge detection reset)
    state.twilight = 0.05f;
    safety.update(state, 5000);
    ASSERT(state.safetyNightReset, "Night reset on next nightfall");
    ASSERT(state.moveInProgress, "Move triggered on new nightfall edge");

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 3. In-flight lock → landing → resume
// ---------------------------------------------------------------------------
static void test_flight_lock() {
    printf("Test 3: In-flight lock -> landing -> resume\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    // On the ground
    safety.update(state, 0);
    ASSERT(!state.safetyFlightLock, "No flight lock on ground");

    // Take off
    state.inFlight = true;
    safety.update(state, 1000);
    ASSERT(state.safetyFlightLock, "Flight lock should be set");
    // Flight lock doesn't change target — BT handles freezing movement

    // Landing
    state.inFlight = false;
    safety.update(state, 2000);
    ASSERT(!state.safetyFlightLock, "Flight lock should clear on landing");

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 4. Multiple simultaneous safety flags
// ---------------------------------------------------------------------------
static void test_multiple_flags() {
    printf("Test 4: Multiple simultaneous safety flags\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    // Trigger all flags at once
    state.windSpeedMs = WIND_MAX_MS + 10.0f;
    state.inFlight    = true;
    state.twilight    = 0.01f;
    state.escArmed    = false;

    safety.update(state, 0);

    ASSERT(state.safetyWindOverride, "Wind override set");
    ASSERT(state.safetyFlightLock,   "Flight lock set");
    ASSERT(state.safetyNightReset,   "Night reset set");
    ASSERT(state.safetyEscNotArmed,  "ESC not armed set");

    // Wind override takes precedence on target (it runs after night reset in code)
    ASSERT_MSG(std::fabs(state.targetTiltDeg - TILT_WIND_SAFE_DEG) < 0.01f,
               "Wind override target should take precedence, got %.2f",
               state.targetTiltDeg);

    // Clear flags one by one
    state.windSpeedMs = 2.0f;
    safety.update(state, 1000);
    ASSERT(!state.safetyWindOverride, "Wind cleared");
    ASSERT(state.safetyFlightLock,    "Flight still set");
    ASSERT(state.safetyNightReset,    "Night still set");
    ASSERT(state.safetyEscNotArmed,   "ESC still not armed");

    state.inFlight = false;
    safety.update(state, 2000);
    ASSERT(!state.safetyFlightLock, "Flight cleared");

    state.escArmed = true;
    safety.update(state, 3000);
    ASSERT(!state.safetyEscNotArmed, "ESC armed cleared");

    state.twilight = 0.5f;
    safety.update(state, 4000);
    ASSERT(!state.safetyNightReset, "Night cleared");

    // All flags clear
    ASSERT(!state.safetyWindOverride && !state.safetyFlightLock &&
           !state.safetyNightReset   && !state.safetyEscNotArmed,
           "All flags should be clear");

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 5. ESC not armed → armed transition
// ---------------------------------------------------------------------------
static void test_esc_arming_transition() {
    printf("Test 5: ESC arming transition\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);
    state.escArmed = false;

    safety.update(state, 0);
    ASSERT(state.safetyEscNotArmed, "ESC not armed flag should be set");

    // ESC arms
    state.escArmed = true;
    safety.update(state, 1000);
    ASSERT(!state.safetyEscNotArmed, "Flag should clear when ESC arms");

    // ESC loses power / disarms
    state.escArmed = false;
    safety.update(state, 2000);
    ASSERT(state.safetyEscNotArmed, "Flag should re-set on disarm");

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 6. Night reset edge detection — only triggers on transition
// ---------------------------------------------------------------------------
static void test_night_reset_edge_detection() {
    printf("Test 6: Night reset edge detection\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    // Start in night (already below threshold)
    state.twilight = 0.05f;
    safety.update(state, 0);
    // First call with night=true when prev_night=false → should trigger
    ASSERT(state.safetyNightReset, "Night flag set");
    ASSERT(state.moveInProgress, "Move triggered on initial night detection");

    // Clear move, keep in night
    state.moveInProgress = false;
    state.targetTiltDeg = 15.0f;  // Set to non-reset value to detect changes

    // Repeated updates in same night state — should NOT re-trigger
    for (int i = 0; i < 5; i++) {
        safety.update(state, 1000 + i * 1000);
        ASSERT(!state.moveInProgress,
               "Should NOT re-trigger move during sustained night");
        ASSERT_MSG(std::fabs(state.targetTiltDeg - 15.0f) < 0.01f,
                   "Target should NOT change during sustained night, got %.2f",
                   state.targetTiltDeg);
    }

    // Dawn
    state.twilight = 0.5f;
    safety.update(state, 10000);
    ASSERT(!state.safetyNightReset, "Night flag cleared at dawn");

    // Next nightfall — edge should fire again
    state.twilight = 0.05f;
    state.targetTiltDeg = 25.0f;
    safety.update(state, 11000);
    ASSERT(state.moveInProgress, "Move triggered on second nightfall");
    ASSERT_MSG(std::fabs(state.targetTiltDeg - TILT_NIGHT_RESET_DEG) < 0.01f,
               "Target set to night reset on second nightfall, got %.2f",
               state.targetTiltDeg);

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 7. Wind override repeatedly fires each tick while active
// ---------------------------------------------------------------------------
static void test_wind_override_continuous() {
    printf("Test 7: Wind override is continuous (not edge-triggered)\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    state.windSpeedMs = WIND_MAX_MS + 3.0f;

    // Each safety tick while wind is high should keep target at wind-safe
    for (int i = 0; i < 5; i++) {
        state.targetTiltDeg = 40.0f;  // something else
        state.moveInProgress = false;
        safety.update(state, i * 1000);
        ASSERT(state.safetyWindOverride, "Wind override stays set");
        ASSERT_MSG(std::fabs(state.targetTiltDeg - TILT_WIND_SAFE_DEG) < 0.01f,
                   "Target reset to wind-safe each tick, got %.2f",
                   state.targetTiltDeg);
        ASSERT(state.moveInProgress,
               "moveInProgress set each tick during wind override");
    }

    printf("  PASS\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 8. Boundary values — exactly at thresholds
// ---------------------------------------------------------------------------
static void test_boundary_values() {
    printf("Test 8: Boundary values at exact thresholds\n");
    SafetyMonitor safety;
    SystemState state;
    reset_state(state);

    // Wind exactly at threshold — should NOT trigger (> not >=)
    state.windSpeedMs = WIND_MAX_MS;
    safety.update(state, 0);
    ASSERT(!state.safetyWindOverride,
           "Wind exactly at threshold should NOT trigger override");

    // Wind just above
    state.windSpeedMs = WIND_MAX_MS + 0.001f;
    safety.update(state, 1000);
    ASSERT(state.safetyWindOverride, "Wind just above threshold should trigger");

    // Twilight exactly at threshold — should NOT trigger (< not <=)
    // Need fresh monitor for clean edge detection
    SafetyMonitor safety2;
    SystemState state2;
    reset_state(state2);
    state2.twilight = TWILIGHT_THRESHOLD;
    safety2.update(state2, 0);
    ASSERT(!state2.safetyNightReset,
           "Twilight exactly at threshold should NOT trigger night reset");

    // Twilight just below
    state2.twilight = TWILIGHT_THRESHOLD - 0.001f;
    safety2.update(state2, 1000);
    ASSERT(state2.safetyNightReset,
           "Twilight just below threshold should trigger night reset");

    printf("  PASS\n");
    tests_passed++;
}

int main() {
    printf("=== Safety Scenario Tests ===\n\n");

    test_wind_spike_and_recovery();
    test_day_night_cycle();
    test_flight_lock();
    test_multiple_flags();
    test_esc_arming_transition();
    test_night_reset_edge_detection();
    test_wind_override_continuous();
    test_boundary_values();

    printf("\n--- Results: %d passed, %d failed ---\n",
           tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
