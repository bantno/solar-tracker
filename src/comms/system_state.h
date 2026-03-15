#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <cstdint>

// Lock-pin mechanism state
enum class PinLockState : uint8_t {
    UNLOCKED,   // Pin fully retracted
    LOCKING,    // Pin extending into lock position
    LOCKED,     // Pin fully engaged
    UNLOCKING   // Pin retracting from lock position
};

// Active tracking mode
enum class TrackingMode : uint8_t {
    ASTRONOMICAL,   // Solar model drives tilt angle
    SENSOR_BASED    // Quad photo-sensor feedback drives tilt
};

// Central system state — single source of truth passed to all modules
struct SystemState {
    // --- Time & Position (populated by comms) ---
    uint16_t year   = 2026;
    uint8_t  month  = 1;
    uint8_t  day    = 1;
    uint8_t  hour   = 12;
    uint8_t  minute = 0;
    uint8_t  second = 0;
    float    latitude  = 0.0f;
    float    longitude = 0.0f;

    // --- Flight / Operational Mode (populated by comms) ---
    bool inFlight = false;          // True when aircraft is airborne

    // --- Encoder ---
    float    tiltAngleDeg  = 0.0f;  // Current wing tilt from AS5600 (0–360°)
    uint16_t encoderRaw    = 0;     // Raw AS5600 counts (0–4095)

    // --- Quad Sensor ---
    uint16_t quadTop    = 0;        // Raw ADC — top quadrant
    uint16_t quadBottom = 0;        // Raw ADC — bottom quadrant
    uint16_t quadLeft   = 0;        // Raw ADC — left quadrant
    uint16_t quadRight  = 0;        // Raw ADC — right quadrant

    // --- Environmental Sensors ---
    float    irradianceWm2 = 0.0f;  // Solar irradiance (W/m²)
    float    windSpeedMs   = 0.0f;  // Wind speed (m/s)
    float    twilight      = 0.0f;  // Ambient light level (0.0–1.0)

    // --- Actuator State ---
    uint16_t tiltPulseUs   = 1500;  // Current ESC pulse width (µs)
    bool     escArmed      = false; // ESC arm state

    // --- Tracking ---
    TrackingMode trackingMode = TrackingMode::ASTRONOMICAL;
    float targetTiltDeg       = 0.0f;  // Desired tilt angle from tracker

    // --- Move State ---
    bool  moveInProgress  = false;   // True while PID is driving toward target
    float moveDuration_s  = 0.0f;    // Duration of last completed move (seconds)

    // --- Sensor Error (telemetry) ---
    float verticalSensorError   = 0.0f;  // quadTop - quadBottom
    float horizontalSensorError = 0.0f;  // quadLeft - quadRight

    // --- Safety Flags ---
    bool safetyWindOverride = false;  // Wind speed exceeds WIND_MAX_MS
    bool safetyFlightLock   = false;  // Aircraft is in flight
    bool safetyNightReset   = false;  // Twilight below threshold
    bool safetyEscNotArmed  = false;  // ESC not armed

    // --- Pin Lock ---
    PinLockState pinLockState = PinLockState::UNLOCKED;

    // --- Solar Model Output (filled by tracker) ---
    float solarElevationDeg = 0.0f;
    float solarAzimuthDeg   = 0.0f;
    float optimalTiltDeg    = 0.0f;
};

#endif // SYSTEM_STATE_H
