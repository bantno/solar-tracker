#ifndef HYBRID_TRACKER_H
#define HYBRID_TRACKER_H

#include <cstdint>
#include "../comms/system_state.h"
#include "solar_model.h"

class HybridTracker {
public:
    explicit HybridTracker(ISolarModel* solar_model);

    // Called once per TRACKING_INTERVAL.
    // Sets state.targetTiltDeg, state.trackingMode,
    // state.solarElevationDeg, state.solarAzimuthDeg.
    // Sets state.moveInProgress = true if new target differs
    // from current tilt by more than TILT_PID_DEADBAND_DEG.
    void update(SystemState& state, uint32_t now_ms);

private:
    ISolarModel* solar_model_;
    TrackingMode pending_mode_   = TrackingMode::ASTRONOMICAL;
    uint32_t     mode_hold_until_ms_ = 0;
};

#endif // HYBRID_TRACKER_H
