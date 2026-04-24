#include "hal_pin_extender.h"
#include "../config/config.h"
#include <Arduino.h>

HalPinExtender::HalPinExtender(uint8_t pwmPin, uint8_t limitPin)
    : pwmPin_(pwmPin), limitSwitch_(limitPin) {}

void HalPinExtender::init() {
    servo_.attach(pwmPin_, PWM_PULSE_MIN_US, PWM_PULSE_MAX_US);
    servo_.writeMicroseconds(EXTENDER_PULSE_STOP_US);
    limitSwitch_.init();
    Serial.print("[EXTENDER] Servo on pin ");
    Serial.print(pwmPin_);
    Serial.print(", limit switch on pin ");
    Serial.println(limitSwitch_.pin());
}

void HalPinExtender::setPulseWidth(uint16_t us) {
    servo_.writeMicroseconds(us);
}

void HalPinExtender::stop() {
    servo_.writeMicroseconds(EXTENDER_PULSE_STOP_US);
}

void HalPinExtender::arm(uint32_t now_ms) {
    switch (armState_) {
        case ArmState::IDLE:
            servo_.writeMicroseconds(EXTENDER_PULSE_ARM_US);
            armStartMs_ = now_ms;
            armState_   = ArmState::ARMING;
            Serial.print("[EXTENDER] pin");
            Serial.print(pwmPin_);
            Serial.println(" arming ESC (1500 us)...");
            break;

        case ArmState::ARMING:
            if ((now_ms - armStartMs_) >= EXTENDER_ARM_MS) {
                armState_ = ArmState::ARMED;
                Serial.print("[EXTENDER] pin");
                Serial.print(pwmPin_);
                Serial.println(" ESC armed");
            }
            break;

        case ArmState::ARMED:
            break;
    }
}

bool HalPinExtender::isArmed() {
    return armState_ == ArmState::ARMED;
}

bool HalPinExtender::isExtended() {
    return limitSwitch_.isPressed();
}

void HalPinExtender::clearExtended() {
    limitSwitch_.clear();
}
