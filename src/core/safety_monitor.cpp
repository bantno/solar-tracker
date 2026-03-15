// Pure C++ — compile-time guard against accidental Arduino inclusion
#if defined(ARDUINO) && !defined(PLATFORMIO_BUILD)
#error "safety_monitor.cpp must not include Arduino.h or any hardware headers"
#endif

#include "safety_monitor.h"
#include "../config/config.h"

void SafetyMonitor::update(SystemState& state, uint32_t now_ms) {
    (void)now_ms;  // Reserved for future use

    // Evaluate safety conditions
    state.safetyWindOverride = (state.windSpeedMs > WIND_MAX_MS);
    state.safetyFlightLock   = state.inFlight;
    state.safetyNightReset   = (state.twilight < TWILIGHT_THRESHOLD);
    state.safetyEscNotArmed  = !state.escArmed;

    // Night reset: on false→true transition, command rest position
    if (state.safetyNightReset && !prev_night_) {
        state.targetTiltDeg  = TILT_NIGHT_RESET_DEG;
        state.moveInProgress = true;
    }
    prev_night_ = state.safetyNightReset;

    // Wind override: when active, command wind-safe position
    if (state.safetyWindOverride) {
        state.targetTiltDeg  = TILT_WIND_SAFE_DEG;
        state.moveInProgress = true;
    }
}
