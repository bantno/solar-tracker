#ifndef TILT_CONTROLLER_H
#define TILT_CONTROLLER_H

#include <cstdint>

class TiltController {
public:
    TiltController(float kp, float ki, float kd,
                   float output_min, float output_max,
                   float deadband_deg, float k_aw);

    // Called every 20 Hz tick while moveInProgress is true.
    // Returns pulse width in microseconds.
    float update(float current_deg, float target_deg, uint32_t now_ms);

    // Returns true when |error| < deadband_deg.
    bool isSettled() const;

    // Reset integrator, derivative state, and last_time.
    void reset();

    // Telemetry accessors
    float getError() const;
    float getIntegral() const;
    float getLastDerivative() const;
    float getLastOutput() const;

private:
    float kp_, ki_, kd_;
    float output_min_, output_max_;
    float deadband_deg_;
    float k_aw_;

    float integral_     = 0.0f;
    float prev_measure_ = 0.0f;
    float last_error_   = 0.0f;
    float last_deriv_   = 0.0f;
    float last_output_  = 1500.0f;
    uint32_t last_time_ = 0;
    bool  first_tick_   = true;
    bool  settled_      = false;

    static constexpr float NEUTRAL = 1500.0f;
};

#endif // TILT_CONTROLLER_H
