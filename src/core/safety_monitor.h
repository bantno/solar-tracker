#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <cstdint>
#include "../comms/system_state.h"

class SafetyMonitor {
public:
    void update(SystemState& state, uint32_t now_ms);

private:
    bool prev_night_ = false;  // Previous safetyNightReset state for edge detection
};

#endif // SAFETY_MONITOR_H
