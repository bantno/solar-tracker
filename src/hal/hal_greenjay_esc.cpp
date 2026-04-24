#include "hal_greenjay_esc.h"
#include "../config/config.h"
#include <Arduino.h>

HalGreenJayEsc::HalGreenJayEsc(uint8_t pin) : pin_(pin) {}

void HalGreenJayEsc::init() {
    servo_.attach(pin_, PWM_PULSE_MIN_US, PWM_PULSE_MAX_US);
    servo_.writeMicroseconds(TILT_ARM_PULSE_HIGH_US);
    Serial.print("[GREENJAY] ESC on pin "); Serial.println(pin_);
}

void HalGreenJayEsc::arm(uint32_t now_ms) {
    switch (armState_) {
        case ArmState::IDLE:
            servo_.writeMicroseconds(TILT_ARM_PULSE_HIGH_US);
            phaseStartMs_ = now_ms;
            armState_     = ArmState::PHASE1;
            Serial.println("[GREENJAY] Arming phase 1 (1503 us)...");
            break;

        case ArmState::PHASE1:
            if ((now_ms - phaseStartMs_) >= TILT_ARM_PHASE1_MS) {
                servo_.writeMicroseconds(TILT_ARM_PULSE_LOW_US);
                phaseStartMs_ = now_ms;
                armState_     = ArmState::PHASE2;
                Serial.println("[GREENJAY] Arming phase 2 (1475 us)...");
            }
            break;

        case ArmState::PHASE2:
            if ((now_ms - phaseStartMs_) >= TILT_ARM_PHASE2_MS) {
                servo_.writeMicroseconds(TILT_MOTOR_PULSE_STOP_US);
                armState_ = ArmState::ARMED;
                Serial.println("[GREENJAY] Armed");
            }
            break;

        case ArmState::ARMED:
            break;
    }
}

bool HalGreenJayEsc::isArmed() {
    return armState_ == ArmState::ARMED;
}

void HalGreenJayEsc::setPulseWidth(uint16_t us) {
    servo_.writeMicroseconds(us);
}

void HalGreenJayEsc::stop() {
    servo_.writeMicroseconds(TILT_MOTOR_PULSE_STOP_US);
}
