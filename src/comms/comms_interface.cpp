#include "comms_interface.h"

void commsInit() {
    // TODO: replace with microROS node initialization
}

void commsUpdate(SystemState* state) {
    // TODO: replace with microROS subscriber callbacks
    // Hardcoded values for milestone 1 testing
    state->year      = 2026;
    state->month     = 3;
    state->day       = 14;
    state->hour      = 12;
    state->minute    = 0;
    state->second    = 0;
    state->latitude  = 48.8566f;
    state->longitude = 2.3522f;
    state->inFlight  = false;
}

void commsPublish(const SystemState* state) {
    // TODO: replace with microROS publishers
    (void)state; // suppress unused parameter warning
}
