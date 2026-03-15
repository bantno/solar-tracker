// Pure C++ — compile-time guard against accidental Arduino inclusion
#if defined(ARDUINO) && !defined(PLATFORMIO_BUILD)
#error "hybrid_tracker.cpp must not include Arduino.h or any hardware headers"
#endif

#include "hybrid_tracker.h"
#include "../config/config.h"
#include <cmath>

HybridTracker::HybridTracker(ISolarModel* solar_model)
    : solar_model_(solar_model) {}

void HybridTracker::update(SystemState& state, uint32_t now_ms) {
    // Compute sensor errors for telemetry
    state.verticalSensorError   = static_cast<float>(state.quadTop) - static_cast<float>(state.quadBottom);
    state.horizontalSensorError = static_cast<float>(state.quadLeft) - static_cast<float>(state.quadRight);

    // Determine desired mode based on irradiance
    TrackingMode desired_mode;
    if (state.irradianceWm2 > IRRADIANCE_THRESHOLD_WM2) {
        desired_mode = TrackingMode::SENSOR_BASED;
    } else {
        desired_mode = TrackingMode::ASTRONOMICAL;
    }

    // Mode hysteresis: only switch if desired mode has been stable for MODE_HOLD_DURATION_MS
    if (desired_mode != state.trackingMode) {
        if (desired_mode != pending_mode_) {
            // New pending mode — start the hold timer
            pending_mode_ = desired_mode;
            mode_hold_until_ms_ = now_ms + MODE_HOLD_DURATION_MS;
        } else if (now_ms >= mode_hold_until_ms_) {
            // Hold period elapsed — commit the switch
            state.trackingMode = desired_mode;
        }
        // else: still waiting for hold period to elapse
    } else {
        // Current mode matches desired — reset pending
        pending_mode_ = state.trackingMode;
    }

    // Compute target based on current (possibly unchanged) mode
    float target = state.targetTiltDeg;

    if (state.trackingMode == TrackingMode::SENSOR_BASED) {
        float vertical_error = static_cast<float>(state.quadTop) - static_cast<float>(state.quadBottom);
        target = state.tiltAngleDeg + vertical_error * SENSOR_TRACK_GAIN;
    } else {
        // ASTRONOMICAL
        SolarModelInput input{};
        input.latitude  = state.latitude;
        input.longitude = state.longitude;
        input.year      = state.year;
        input.month     = state.month;
        input.day       = state.day;
        input.hour      = state.hour;
        input.minute    = state.minute;
        input.second    = state.second;

        SolarModelOutput output = solar_model_->compute(input);
        target = output.optimal_tilt_deg;
        state.solarElevationDeg = output.solar_elevation_deg;
        state.solarAzimuthDeg   = output.solar_azimuth_deg;
    }

    // Clamp target to valid range
    if (target < TILT_BETA_MIN_DEG) target = TILT_BETA_MIN_DEG;
    if (target > TILT_BETA_MAX_DEG) target = TILT_BETA_MAX_DEG;

    state.targetTiltDeg = target;

    // Trigger a move if target differs meaningfully from current position
    if (std::fabs(state.targetTiltDeg - state.tiltAngleDeg) > TILT_PID_DEADBAND_DEG) {
        state.moveInProgress = true;
    }
}
