// Desktop edge case tests for TiltController
// Compile: g++ -std=c++17 -I src test/test_edge_cases.cpp
//          src/core/tilt_controller.cpp -o test_edge_cases.exe

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <climits>
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

static bool output_in_range(float o) {
    return o >= TILT_PID_OUT_MIN && o <= TILT_PID_OUT_MAX;
}

static bool is_finite(float v) {
    return !std::isnan(v) && !std::isinf(v);
}

// ---------------------------------------------------------------------------
// 1. Negative target angle
// ---------------------------------------------------------------------------
static void test_negative_target() {
    printf("Test 1: Negative target angle\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Target below zero — current at 10 degrees
    float current = 10.0f;
    float target  = -5.0f;

    for (int tick = 0; tick < 100; tick++) {
        float output = pid.update(current, target, tick * 50);
        ASSERT(output_in_range(output), "Output must stay in range with negative target");
        ASSERT(is_finite(output), "Output must be finite");
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    // Should drive below zero toward target
    ASSERT_MSG(current < 5.0f,
               "Should drive toward negative target, got %.2f", current);
    ASSERT(is_finite(pid.getIntegral()), "Integral must be finite");

    printf("  PASS (current=%.2f with target=-5)\n", current);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 2. Zero-degree target from high angle
// ---------------------------------------------------------------------------
static void test_zero_target() {
    printf("Test 2: Zero-degree target from high angle\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 60.0f;
    float target  = 0.0f;

    for (int tick = 0; tick < 200; tick++) {
        float output = pid.update(current, target, tick * 50);
        ASSERT(output_in_range(output), "Output in range");
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    ASSERT_MSG(std::fabs(current - target) < 1.0f,
               "Should converge to 0, got %.2f", current);

    printf("  PASS (converged to %.2f)\n", current);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 3. millis() overflow (uint32_t wraps at ~49.7 days)
// ---------------------------------------------------------------------------
static void test_millis_overflow() {
    printf("Test 3: millis() overflow at uint32_t boundary\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float target  = 40.0f;

    // Start near the overflow boundary
    uint32_t start = UINT32_MAX - 500;  // 500ms before overflow

    // Run 20 ticks spanning the overflow
    for (int tick = 0; tick < 20; tick++) {
        uint32_t now = start + (uint32_t)(tick * 50);
        float output = pid.update(current, target, now);
        ASSERT_MSG(output_in_range(output),
                   "Tick %d (t=%u): output %.1f out of range", tick, now, output);
        ASSERT_MSG(is_finite(output),
                   "Tick %d: output is NaN/Inf", tick);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    ASSERT(is_finite(pid.getError()), "Error finite after overflow");
    ASSERT(is_finite(pid.getIntegral()), "Integral finite after overflow");
    ASSERT(is_finite(pid.getLastDerivative()), "Derivative finite after overflow");

    printf("  PASS (no NaN/Inf through uint32 overflow, current=%.2f)\n", current);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 4. Rapid target changes mid-move
// ---------------------------------------------------------------------------
static void test_rapid_target_changes() {
    printf("Test 4: Rapid target changes mid-move\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float targets[] = {45.0f, 10.0f, 60.0f, 25.0f, 80.0f};
    int num_targets = 5;

    for (int phase = 0; phase < num_targets; phase++) {
        float target = targets[phase];
        pid.reset();

        // Run 30 ticks per target (1.5 seconds — not enough to settle)
        for (int tick = 0; tick < 30; tick++) {
            uint32_t now = (phase * 30 + tick) * 50;
            float output = pid.update(current, target, now);
            ASSERT_MSG(output_in_range(output),
                       "Phase %d tick %d: output %.1f OOB", phase, tick, output);
            ASSERT_MSG(is_finite(output),
                       "Phase %d tick %d: NaN/Inf", phase, tick);

            // Check direction: output should move toward target
            if (tick > 0) {  // skip first tick (P-only)
                float error = target - current;
                if (std::fabs(error) > TILT_PID_DEADBAND_DEG) {
                    bool correct_dir = (error > 0 && output > 1500.0f) ||
                                       (error < 0 && output < 1500.0f);
                    // Allow some slack for integral wind-down
                    if (tick > 5) {
                        ASSERT_MSG(correct_dir,
                                   "Phase %d tick %d: wrong direction (err=%.1f, out=%.1f)",
                                   phase, tick, error, output);
                    }
                }
            }

            float drive = (output - 1500.0f) / 100.0f;
            current += drive;
        }
    }

    ASSERT(is_finite(current), "Final position is finite");
    printf("  PASS (survived 5 rapid target changes, final=%.2f)\n", current);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 5. Same timestamp twice (dt = 0)
// ---------------------------------------------------------------------------
static void test_zero_dt() {
    printf("Test 5: Same timestamp twice (dt=0)\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // First tick at t=0
    float o1 = pid.update(10.0f, 30.0f, 0);
    ASSERT(output_in_range(o1) && is_finite(o1), "First tick OK");

    // Second tick at same timestamp
    float o2 = pid.update(10.0f, 30.0f, 0);
    ASSERT(output_in_range(o2) && is_finite(o2), "Second tick at same time should not crash");

    // Third tick at t=0 again
    float o3 = pid.update(10.5f, 30.0f, 0);
    ASSERT(output_in_range(o3) && is_finite(o3), "Third tick at same time OK");

    printf("  PASS (no crash with duplicate timestamps)\n");
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 6. Very large dt (long pause between ticks)
// ---------------------------------------------------------------------------
static void test_large_dt() {
    printf("Test 6: Large dt (long pause between ticks)\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float target  = 40.0f;

    // Normal first tick
    float o1 = pid.update(current, target, 0);
    ASSERT(output_in_range(o1), "First tick OK");

    // 10-second gap (dt clamped to 100ms by design)
    float o2 = pid.update(current, target, 10000);
    ASSERT(output_in_range(o2) && is_finite(o2), "Output OK after large dt");

    // The integral shouldn't have exploded
    float integral = pid.getIntegral();
    ASSERT_MSG(std::fabs(integral) < 1000.0f,
               "Integral should be bounded after large dt, got %.2f", integral);

    printf("  PASS (integral=%.2f, clamped dt prevented explosion)\n", integral);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 7. Oscillating around target (noise rejection)
// ---------------------------------------------------------------------------
static void test_oscillating_measurement() {
    printf("Test 7: Oscillating measurement around target\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float target = 30.0f;
    int neutral_count = 0;
    int total_ticks = 100;

    for (int tick = 0; tick < total_ticks; tick++) {
        // Oscillate ±0.3 degrees around target (within deadband)
        float noise = (tick % 2 == 0) ? 0.3f : -0.3f;
        float current = target + noise;
        float output = pid.update(current, target, tick * 50);
        ASSERT(output_in_range(output), "Output in range");
        if (output == 1500.0f) neutral_count++;
    }

    ASSERT_MSG(neutral_count == total_ticks,
               "All ticks within deadband should output neutral, got %d/%d",
               neutral_count, total_ticks);

    printf("  PASS (%d/%d ticks returned neutral)\n", neutral_count, total_ticks);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 8. Extreme angles (very large error)
// ---------------------------------------------------------------------------
static void test_extreme_angles() {
    printf("Test 8: Extreme angles (0 -> 90 degrees)\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 0.0f;
    float target  = 90.0f;

    // First tick should saturate at max
    float output = pid.update(current, target, 0);
    ASSERT_MSG(output == TILT_PID_OUT_MAX,
               "90 deg error should saturate at max, got %.1f", output);

    // Run and verify convergence
    for (int tick = 1; tick < 500; tick++) {
        output = pid.update(current, target, tick * 50);
        ASSERT(output_in_range(output), "Output always in range");
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    ASSERT_MSG(std::fabs(current - 90.0f) < 2.0f,
               "Should converge near 90, got %.2f", current);

    printf("  PASS (converged to %.2f from 0 -> 90)\n", current);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 9. Reset mid-move then new target — no derivative kick
// ---------------------------------------------------------------------------
static void test_reset_mid_move_no_kick() {
    printf("Test 9: Reset mid-move then new target — no derivative kick\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 10.0f;

    // Drive toward 50 for 20 ticks
    for (int tick = 0; tick < 20; tick++) {
        float output = pid.update(current, 50.0f, tick * 50);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    // Reset and immediately issue new target
    pid.reset();
    float output_after_reset = pid.update(current, 10.0f, 1000);

    // First tick after reset: derivative should be zero (no kick)
    ASSERT_MSG(pid.getLastDerivative() == 0.0f,
               "Derivative should be 0 on first tick after reset, got %.4f",
               pid.getLastDerivative());
    ASSERT(output_in_range(output_after_reset), "Output in range after reset");

    // The output should reflect P-only (negative error, output < 1500)
    ASSERT(output_after_reset < 1500.0f,
           "New target below current should produce output < 1500");

    printf("  PASS (derivative=0 after reset, output=%.1f)\n", output_after_reset);
    tests_passed++;
}

// ---------------------------------------------------------------------------
// 10. Sustained saturation then direction reversal
// ---------------------------------------------------------------------------
static void test_saturation_then_reversal() {
    printf("Test 10: Sustained saturation then direction reversal\n");
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    // Saturate at max for 50 ticks (huge positive error, don't move current)
    for (int tick = 0; tick < 50; tick++) {
        pid.update(0.0f, 90.0f, tick * 50);
    }

    // Now reverse: target below current
    pid.reset();
    float current = 0.0f;

    // PID should respond in the correct (negative) direction quickly
    float output = pid.update(current, -10.0f, 3000);
    ASSERT(output < 1500.0f, "Should drive negative after reversal");

    // Run a few more ticks to confirm no integral windup carryover
    for (int tick = 1; tick < 10; tick++) {
        output = pid.update(current, -10.0f, 3000 + tick * 50);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
        ASSERT(output <= 1500.0f,
               "Should consistently drive negative after reset+reversal");
    }

    printf("  PASS (clean reversal after saturation, current=%.2f)\n", current);
    tests_passed++;
}

int main() {
    printf("=== Edge Case Tests ===\n\n");

    test_negative_target();
    test_zero_target();
    test_millis_overflow();
    test_rapid_target_changes();
    test_zero_dt();
    test_large_dt();
    test_oscillating_measurement();
    test_extreme_angles();
    test_reset_mid_move_no_kick();
    test_saturation_then_reversal();

    printf("\n--- Results: %d passed, %d failed ---\n",
           tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
