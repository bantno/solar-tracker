// Unity edge case tests for TiltController
// Run: pio test -e native -f test_edge_cases

#include <unity.h>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <climits>
#include "core/tilt_controller.h"
#include "config/config.h"

void setUp(void) {}
void tearDown(void) {}

static bool output_in_range(float o) {
    return o >= TILT_PID_OUT_MIN && o <= TILT_PID_OUT_MAX;
}

static bool is_finite(float v) {
    return !std::isnan(v) && !std::isinf(v);
}

// 1. Negative target angle
void test_negative_target() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 10.0f;
    float target  = -5.0f;

    for (int tick = 0; tick < 100; tick++) {
        float output = pid.update(current, target, tick * 50);
        TEST_ASSERT_TRUE_MESSAGE(output_in_range(output),
            "Output must stay in range with negative target");
        TEST_ASSERT_TRUE_MESSAGE(is_finite(output), "Output must be finite");
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "Should drive toward negative target, got %.2f", current);
    TEST_ASSERT_TRUE_MESSAGE(current < 5.0f, buf);
    TEST_ASSERT_TRUE_MESSAGE(is_finite(pid.getIntegral()), "Integral must be finite");

    printf("  INFO: current=%.2f with target=-5\n", current);
}

// 2. Zero-degree target from high angle
void test_zero_target() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 60.0f;
    float target  = 0.0f;

    for (int tick = 0; tick < 200; tick++) {
        float output = pid.update(current, target, tick * 50);
        TEST_ASSERT_TRUE_MESSAGE(output_in_range(output), "Output in range");
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    TEST_ASSERT_FLOAT_WITHIN(1.0f, target, current);
    printf("  INFO: converged to %.2f\n", current);
}

// 3. millis() overflow (uint32_t wraps at ~49.7 days)
void test_millis_overflow() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float target  = 40.0f;

    uint32_t start = UINT32_MAX - 500;

    for (int tick = 0; tick < 20; tick++) {
        uint32_t now = start + (uint32_t)(tick * 50);
        float output = pid.update(current, target, now);
        char buf[256];
        snprintf(buf, sizeof(buf), "Tick %d (t=%u): output %.1f out of range",
                 tick, now, output);
        TEST_ASSERT_TRUE_MESSAGE(output_in_range(output), buf);
        snprintf(buf, sizeof(buf), "Tick %d: output is NaN/Inf", tick);
        TEST_ASSERT_TRUE_MESSAGE(is_finite(output), buf);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    TEST_ASSERT_TRUE_MESSAGE(is_finite(pid.getError()), "Error finite after overflow");
    TEST_ASSERT_TRUE_MESSAGE(is_finite(pid.getIntegral()), "Integral finite after overflow");
    TEST_ASSERT_TRUE_MESSAGE(is_finite(pid.getLastDerivative()),
        "Derivative finite after overflow");

    printf("  INFO: no NaN/Inf through uint32 overflow, current=%.2f\n", current);
}

// 4. Rapid target changes mid-move
void test_rapid_target_changes() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float targets[] = {45.0f, 10.0f, 60.0f, 25.0f, 80.0f};
    int num_targets = 5;

    for (int phase = 0; phase < num_targets; phase++) {
        float target = targets[phase];
        pid.reset();

        for (int tick = 0; tick < 30; tick++) {
            uint32_t now = (phase * 30 + tick) * 50;
            float output = pid.update(current, target, now);

            char buf[256];
            snprintf(buf, sizeof(buf), "Phase %d tick %d: output %.1f OOB",
                     phase, tick, output);
            TEST_ASSERT_TRUE_MESSAGE(output_in_range(output), buf);

            snprintf(buf, sizeof(buf), "Phase %d tick %d: NaN/Inf", phase, tick);
            TEST_ASSERT_TRUE_MESSAGE(is_finite(output), buf);

            if (tick > 5) {
                float error = target - current;
                if (std::fabs(error) > TILT_PID_DEADBAND_DEG) {
                    bool correct_dir = (error > 0 && output > 1500.0f) ||
                                       (error < 0 && output < 1500.0f);
                    snprintf(buf, sizeof(buf),
                             "Phase %d tick %d: wrong direction (err=%.1f, out=%.1f)",
                             phase, tick, error, output);
                    TEST_ASSERT_TRUE_MESSAGE(correct_dir, buf);
                }
            }

            float drive = (output - 1500.0f) / 100.0f;
            current += drive;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(is_finite(current), "Final position is finite");
    printf("  INFO: survived 5 rapid target changes, final=%.2f\n", current);
}

// 5. Same timestamp twice (dt = 0)
void test_zero_dt() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float o1 = pid.update(10.0f, 30.0f, 0);
    TEST_ASSERT_TRUE(output_in_range(o1) && is_finite(o1));

    float o2 = pid.update(10.0f, 30.0f, 0);
    TEST_ASSERT_TRUE_MESSAGE(output_in_range(o2) && is_finite(o2),
        "Second tick at same time should not crash");

    float o3 = pid.update(10.5f, 30.0f, 0);
    TEST_ASSERT_TRUE_MESSAGE(output_in_range(o3) && is_finite(o3),
        "Third tick at same time OK");
}

// 6. Very large dt (long pause between ticks)
void test_large_dt() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 20.0f;
    float target  = 40.0f;

    float o1 = pid.update(current, target, 0);
    TEST_ASSERT_TRUE(output_in_range(o1));

    float o2 = pid.update(current, target, 10000);
    TEST_ASSERT_TRUE(output_in_range(o2) && is_finite(o2));

    float integral = pid.getIntegral();
    char buf[256];
    snprintf(buf, sizeof(buf),
             "Integral should be bounded after large dt, got %.2f", integral);
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(integral) < 1000.0f, buf);

    printf("  INFO: integral=%.2f, clamped dt prevented explosion\n", integral);
}

// 7. Oscillating around target (noise rejection)
void test_oscillating_measurement() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float target = 30.0f;
    int neutral_count = 0;
    int total_ticks = 100;

    for (int tick = 0; tick < total_ticks; tick++) {
        float noise = (tick % 2 == 0) ? 0.3f : -0.3f;
        float current = target + noise;
        float output = pid.update(current, target, tick * 50);
        TEST_ASSERT_TRUE(output_in_range(output));
        if (output == 1500.0f) neutral_count++;
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
             "All ticks within deadband should output neutral, got %d/%d",
             neutral_count, total_ticks);
    TEST_ASSERT_EQUAL_INT_MESSAGE(total_ticks, neutral_count, buf);

    printf("  INFO: %d/%d ticks returned neutral\n", neutral_count, total_ticks);
}

// 8. Extreme angles (very large error)
void test_extreme_angles() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 0.0f;
    float target  = 90.0f;

    float output = pid.update(current, target, 0);
    TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(TILT_PID_OUT_MAX), output);

    for (int tick = 1; tick < 500; tick++) {
        output = pid.update(current, target, tick * 50);
        TEST_ASSERT_TRUE(output_in_range(output));
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, current);
    printf("  INFO: converged to %.2f from 0 -> 90\n", current);
}

// 9. Reset mid-move then new target -- no derivative kick
void test_reset_mid_move_no_kick() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 10.0f;

    for (int tick = 0; tick < 20; tick++) {
        float output = pid.update(current, 50.0f, tick * 50);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    pid.reset();
    float output_after_reset = pid.update(current, 10.0f, 1000);

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, pid.getLastDerivative(),
        "Derivative should be 0 on first tick after reset");
    TEST_ASSERT_TRUE(output_in_range(output_after_reset));
    TEST_ASSERT_TRUE_MESSAGE(output_after_reset < 1500.0f,
        "New target below current should produce output < 1500");

    printf("  INFO: derivative=0 after reset, output=%.1f\n", output_after_reset);
}

// 10. Sustained saturation then direction reversal
void test_saturation_then_reversal() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    for (int tick = 0; tick < 50; tick++) {
        pid.update(0.0f, 90.0f, tick * 50);
    }

    pid.reset();
    float current = 0.0f;

    float output = pid.update(current, -10.0f, 3000);
    TEST_ASSERT_TRUE_MESSAGE(output < 1500.0f,
        "Should drive negative after reversal");

    for (int tick = 1; tick < 10; tick++) {
        output = pid.update(current, -10.0f, 3000 + tick * 50);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
        TEST_ASSERT_TRUE_MESSAGE(output <= 1500.0f,
            "Should consistently drive negative after reset+reversal");
    }

    printf("  INFO: clean reversal after saturation, current=%.2f\n", current);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_negative_target);
    RUN_TEST(test_zero_target);
    RUN_TEST(test_millis_overflow);
    RUN_TEST(test_rapid_target_changes);
    RUN_TEST(test_zero_dt);
    RUN_TEST(test_large_dt);
    RUN_TEST(test_oscillating_measurement);
    RUN_TEST(test_extreme_angles);
    RUN_TEST(test_reset_mid_move_no_kick);
    RUN_TEST(test_saturation_then_reversal);
    return UNITY_END();
}
