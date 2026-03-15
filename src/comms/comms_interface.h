#ifndef COMMS_INTERFACE_H
#define COMMS_INTERFACE_H

#include "system_state.h"

// Initialize communications (serial, future: microROS)
void commsInit();

// Pull external data into system state (time, GPS, flight mode)
// TODO: replace with microROS subscriber callbacks
void commsUpdate(SystemState* state);

// Publish system state to external consumers
// TODO: replace with microROS publishers
void commsPublish(const SystemState* state);

#endif // COMMS_INTERFACE_H
