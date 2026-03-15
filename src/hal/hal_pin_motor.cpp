#include "hal_pin_motor.h"
#include <Arduino.h>

void HalPinMotor::init() {
    // H-bridge / motor driver IC not yet selected.
    // Pin modes will be configured here once hardware is finalized.
    Serial.println("[PinMotor] Stub initialized — no hardware driver selected");
}

void HalPinMotor::setPinMotor(uint8_t motorIndex, PinMotorDir dir, uint8_t speed) {
    // Stub: debug output only until driver IC is selected
    const char* dirStr = "STOP";
    if (dir == PinMotorDir::FORWARD) dirStr = "FWD";
    else if (dir == PinMotorDir::REVERSE) dirStr = "REV";

    Serial.print("[PinMotor] Motor ");
    Serial.print(motorIndex);
    Serial.print(" → ");
    Serial.print(dirStr);
    Serial.print(" @ speed ");
    Serial.println(speed);
}
