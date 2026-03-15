// Desktop test for TiltController
// Compile: g++ -std=c++17 -I src test/test_tilt_controller.cpp src/core/tilt_controller.cpp -o test_pid

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "core/tilt_controller.h"
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

// 1. Step response: target=45, current=0, run 200 ticks at 50ms
static void test_step_response() {
    printf("Test 1: Step response\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 0.0f;
    float target  = 45.0f;
    bool settled_within_100 = false;

    for (int tick = 0; tick < 200; tick++) {
        uint32_t now_ms = tick * 50;
        float output = pid.update(current, target, now_ms);

        // Verify output always in range
        ASSERT_MSG(output >= TILT_PID_OUT_MIN && output <= TILT_PID_OUT_MAX,
                   "Tick %d: output %.1f out of range [%d, %d]",
                   tick, output, TILT_PID_OUT_MIN, TILT_PID_OUT_MAX);

        // Simulate actuator: output > 1500 moves current toward target
        // Scale: full range (500us deviation) = ~5 deg/tick
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;

        if (pid.isSettled() && !settled_within_100 && tick < 100) {
            settled_within_100 = true;
        }
    }

    ASSERT(std::fabs(current - target) < 5.0f,
           "Should converge near target after 200 ticks");
    // Check that error is small by end
    ASSERT(std::fabs(pid.getError()) < TILT_PID_DEADBAND_DEG,
           "Final error should be within deadband");
    printf("  PASS (final current=%.2f, error=%.4f)\n", current, pid.getError());
    tests_passed++;
}

// 2. Deadband: |error| < deadband → output = 1500
static void test_deadband() {
    printf("Test 2: Deadband\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Error of 0.1 degrees, well within deadband of 0.5
    float output = pid.update(44.9f, 45.0f, 0);
    ASSERT(output == 1500.0f, "Output should be exactly 1500 within deadband");
    ASSERT(pid.isSettled(), "Should be settled within deadband");

    output = pid.update(45.3f, 45.0f, 50);
    ASSERT(output == 1500.0f, "Negative small error should also be 1500");

    printf("  PASS\n");
    tests_passed++;
}

// 3. Anti-windup: hold at saturation for 50 ticks, integral should stay bounded
static void test_anti_windup() {
    printf("Test 3: Anti-windup\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Large error that will saturate output
    float current = 0.0f;
    float target  = 90.0f;

    for (int tick = 0; tick < 50; tick++) {
        pid.update(current, target, tick * 50);
        // Don't move current — keep saturated
    }

    float integral = pid.getIntegral();
    float max_reasonable = 2.0f * (TILT_PID_OUT_MAX - TILT_PID_OUT_MIN) / TILT_PID_KI;
    ASSERT_MSG(std::fabs(integral) < max_reasonable,
               "Integral %.2f exceeds bound %.2f", integral, max_reasonable);
    printf("  PASS (integral=%.2f, bound=%.2f)\n", integral, max_reasonable);
    tests_passed++;
}

// 4. Reset: after reset(), integral and derivative are zero, no derivative kick
static void test_reset() {
    printf("Test 4: Reset\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Build up some state
    for (int tick = 0; tick < 20; tick++) {
        pid.update(0.0f, 45.0f, tick * 50);
    }

    pid.reset();
    ASSERT(pid.getIntegral() == 0.0f, "Integral should be zero after reset");
    ASSERT(pid.getLastDerivative() == 0.0f, "Derivative should be zero after reset");

    // First tick after reset: derivative should be zero (no kick)
    pid.update(20.0f, 45.0f, 2000);
    ASSERT(pid.getLastDerivative() == 0.0f,
           "Derivative should be zero on first tick after reset (no kick)");

    printf("  PASS\n");
    tests_passed++;
}

// 5. Direction: positive error → output > 1500, negative → output < 1500
static void test_direction() {
    printf("Test 5: Direction\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Positive error: target > current
    float output = pid.update(10.0f, 45.0f, 0);
    ASSERT(output > 1500.0f, "Positive error should produce output > 1500");

    pid.reset();

    // Negative error: target < current
    output = pid.update(45.0f, 10.0f, 100);
    ASSERT(output < 1500.0f, "Negative error should produce output < 1500");

    printf("  PASS\n");
    tests_passed++;
}

// 6. Move complete detection: isSettled() should not be true on first tick of large step
static void test_move_complete() {
    printf("Test 6: Move complete detection\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Large step: 0 → 45 degrees
    pid.update(0.0f, 45.0f, 0);
    ASSERT(!pid.isSettled(), "Should NOT be settled on first tick of 45° step");

    // Run until settled
    float current = 0.0f;
    bool ever_settled = false;
    for (int tick = 1; tick < 300; tick++) {
        float output = pid.update(current, 45.0f, tick * 50);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
        if (pid.isSettled()) {
            ever_settled = true;
            printf("  Settled at tick %d, current=%.2f\n", tick, current);
            break;
        }
    }
    ASSERT(ever_settled, "Should eventually settle");

    printf("  PASS\n");
    tests_passed++;
}

// Simulation trace
static void simulation_trace() {
    printf("\n=== Simulation Trace (60 ticks, 50ms each) ===\n");
    printf("Hardcoded: irr=250, wind=2, quadTop=550, quadBot=500, tilt=20, target=35\n\n");

    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float target  = 35.0f;

    printf("%-5s  %-8s  %-8s  %-10s  %-8s\n",
           "Tick", "Error", "Current", "Output(us)", "Settled");
    printf("-----  --------  --------  ----------  --------\n");

    for (int tick = 0; tick < 60; tick++) {
        uint32_t now_ms = tick * 50;
        float output = pid.update(current, target, now_ms);
        bool settled = pid.isSettled();

        printf("%-5d  %+7.3f  %8.3f  %10.1f  %-8s\n",
               tick, pid.getError(), current, output,
               settled ? "YES" : "no");

        if (settled) break;

        // Simulate actuator response
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }
}

int main() {
    printf("=== TiltController Tests ===\n\n");

    test_step_response();
    test_deadband();
    test_anti_windup();
    test_reset();
    test_direction();
    test_move_complete();

    printf("\n--- Results: %d passed, %d failed ---\n",
           tests_passed, tests_failed);

    simulation_trace();

    return tests_failed > 0 ? 1 : 0;
}
