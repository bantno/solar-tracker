// Pure C++ — compile-time guard against accidental Arduino inclusion
#if defined(ARDUINO) && !defined(PLATFORMIO_BUILD)
#error "tilt_controller.cpp must not include Arduino.h or any hardware headers"
#endif

#include "tilt_controller.h"
#include <cmath>

TiltController::TiltController(float kp, float ki, float kd,
                               float output_min, float output_max,
                               float deadband_deg, float k_aw)
    : kp_(kp), ki_(ki), kd_(kd),
      output_min_(output_min), output_max_(output_max),
      deadband_deg_(deadband_deg), k_aw_(k_aw) {}

float TiltController::update(float current_deg, float target_deg, uint32_t now_ms) {
    float error = target_deg - current_deg;

    // Deadband: if settled, return neutral without updating state
    if (std::fabs(error) < deadband_deg_) {
        settled_ = true;
        last_error_ = error;
        last_output_ = NEUTRAL;
        return NEUTRAL;
    }

    settled_ = false;

    // Compute dt in seconds, clamped to [1 ms, 100 ms]
    float dt_ms = static_cast<float>(now_ms - last_time_);
    if (dt_ms < 1.0f) dt_ms = 1.0f;
    if (dt_ms > 100.0f) dt_ms = 100.0f;
    float dt = dt_ms / 1000.0f;

    last_time_ = now_ms;

    // On first tick after reset, skip integral and derivative
    if (first_tick_) {
        first_tick_ = false;
        prev_measure_ = current_deg;
        last_error_ = error;
        last_deriv_ = 0.0f;
        // P-only on first tick
        float output = NEUTRAL + kp_ * error;
        // Clamp
        if (output < output_min_) output = output_min_;
        if (output > output_max_) output = output_max_;
        last_output_ = output;
        return output;
    }

    // Integral term
    integral_ += ki_ * error * dt;

    // Derivative on measurement (not error) to avoid derivative kick
    float d_measurement = (current_deg - prev_measure_) / dt;
    prev_measure_ = current_deg;
    last_deriv_ = -d_measurement; // negative because derivative on measurement

    // PID output (offset from neutral)
    float raw_output = NEUTRAL + kp_ * error + integral_ - kd_ * d_measurement;

    // Clamp output
    float clamped_output = raw_output;
    if (clamped_output < output_min_) clamped_output = output_min_;
    if (clamped_output > output_max_) clamped_output = output_max_;

    // Anti-windup via back-calculation
    if (clamped_output != raw_output) {
        integral_ -= k_aw_ * (raw_output - clamped_output);
    }

    last_error_ = error;
    last_output_ = clamped_output;
    return clamped_output;
}

bool TiltController::isSettled() const {
    return settled_;
}

void TiltController::reset() {
    integral_ = 0.0f;
    prev_measure_ = 0.0f;
    last_error_ = 0.0f;
    last_deriv_ = 0.0f;
    last_output_ = NEUTRAL;
    last_time_ = 0;
    first_tick_ = true;
    settled_ = false;
}

float TiltController::getError() const { return last_error_; }
float TiltController::getIntegral() const { return integral_; }
float TiltController::getLastDerivative() const { return last_deriv_; }
float TiltController::getLastOutput() const { return last_output_; }
