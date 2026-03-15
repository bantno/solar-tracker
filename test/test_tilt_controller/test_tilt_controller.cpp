// Unity test for TiltController
// Run: pio test -e native -f test_tilt_controller

#include <unity.h>
#include <cmath>
#include <cstdio>
#include "core/tilt_controller.h"
#include "config/config.h"

void setUp(void) {}
void tearDown(void) {}

// 1. Step response: target=45, current=0, run 200 ticks at 50ms
void test_step_response() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 0.0f;
    float target  = 45.0f;

    for (int tick = 0; tick < 200; tick++) {
        uint32_t now_ms = tick * 50;
        float output = pid.update(current, target, now_ms);

        // Verify output always in range
        char buf[256];
        snprintf(buf, sizeof(buf),
                 "Tick %d: output %.1f out of range [%d, %d]",
                 tick, output, TILT_PID_OUT_MIN, TILT_PID_OUT_MAX);
        TEST_ASSERT_TRUE_MESSAGE(
            output >= TILT_PID_OUT_MIN && output <= TILT_PID_OUT_MAX, buf);

        // Simulate actuator
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    TEST_ASSERT_TRUE_MESSAGE(std::fabs(current - target) < 5.0f,
                             "Should converge near target after 200 ticks");
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(pid.getError()) < TILT_PID_DEADBAND_DEG,
                             "Final error should be within deadband");
    printf("  INFO: final current=%.2f, error=%.4f\n", current, pid.getError());
}

// 2. Deadband: |error| < deadband -> output = 1500
void test_deadband() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float output = pid.update(44.9f, 45.0f, 0);
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, output);
    TEST_ASSERT_TRUE_MESSAGE(pid.isSettled(), "Should be settled within deadband");

    output = pid.update(45.3f, 45.0f, 50);
    TEST_ASSERT_EQUAL_FLOAT(1500.0f, output);
}

// 3. Anti-windup: hold at saturation for 50 ticks, integral should stay bounded
void test_anti_windup() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float current = 0.0f;
    float target  = 90.0f;

    for (int tick = 0; tick < 50; tick++) {
        pid.update(current, target, tick * 50);
    }

    float integral = pid.getIntegral();
    float max_reasonable = 2.0f * (TILT_PID_OUT_MAX - TILT_PID_OUT_MIN) / TILT_PID_KI;
    char buf[256];
    snprintf(buf, sizeof(buf), "Integral %.2f exceeds bound %.2f", integral, max_reasonable);
    TEST_ASSERT_TRUE_MESSAGE(std::fabs(integral) < max_reasonable, buf);
    printf("  INFO: integral=%.2f, bound=%.2f\n", integral, max_reasonable);
}

// 4. Reset: after reset(), integral and derivative are zero, no derivative kick
void test_reset() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    for (int tick = 0; tick < 20; tick++) {
        pid.update(0.0f, 45.0f, tick * 50);
    }

    pid.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getIntegral());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getLastDerivative());

    pid.update(20.0f, 45.0f, 2000);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, pid.getLastDerivative(),
        "Derivative should be zero on first tick after reset (no kick)");
}

// 5. Direction: positive error -> output > 1500, negative -> output < 1500
void test_direction() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    float output = pid.update(10.0f, 45.0f, 0);
    TEST_ASSERT_TRUE_MESSAGE(output > 1500.0f,
        "Positive error should produce output > 1500");

    pid.reset();

    output = pid.update(45.0f, 10.0f, 100);
    TEST_ASSERT_TRUE_MESSAGE(output < 1500.0f,
        "Negative error should produce output < 1500");
}

// 6. Move complete detection
void test_move_complete() {
    TiltController pid(TILT_PID_KP, TILT_PID_KI, TILT_PID_KD,
                       TILT_PID_OUT_MIN, TILT_PID_OUT_MAX,
                       TILT_PID_DEADBAND_DEG, TILT_PID_KAW);

    pid.update(0.0f, 45.0f, 0);
    TEST_ASSERT_FALSE_MESSAGE(pid.isSettled(),
        "Should NOT be settled on first tick of 45 deg step");

    float current = 0.0f;
    bool ever_settled = false;
    for (int tick = 1; tick < 300; tick++) {
        float output = pid.update(current, 45.0f, tick * 50);
        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
        if (pid.isSettled()) {
            ever_settled = true;
            printf("  INFO: Settled at tick %d, current=%.2f\n", tick, current);
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(ever_settled, "Should eventually settle");
}

// Simulation trace (informational — always passes)
void test_simulation_trace() {
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

        float drive = (output - 1500.0f) / 100.0f;
        current += drive;
    }

    TEST_ASSERT_TRUE_MESSAGE(true, "Simulation trace complete");
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_step_response);
    RUN_TEST(test_deadband);
    RUN_TEST(test_anti_windup);
    RUN_TEST(test_reset);
    RUN_TEST(test_direction);
    RUN_TEST(test_move_complete);
    RUN_TEST(test_simulation_trace);
    return UNITY_END();
}
