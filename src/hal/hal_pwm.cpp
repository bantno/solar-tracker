#include "hal_pwm.h"
#include "../config/config.h"
#include <Arduino.h>
#include <Servo.h>

static Servo tiltServo;

bool HalPwm::init() {
    tiltServo.attach(PIN_ESC_PWM, PWM_PULSE_MIN_US, PWM_PULSE_MAX_US);
    Serial.println("[PWM] Servo attached to pin, 50 Hz");
    return true;
}

void HalPwm::setTiltPulseWidth(uint16_t us) {
    // Clamp to safe range
    if (us < PWM_PULSE_MIN_US) us = PWM_PULSE_MIN_US;
    if (us > PWM_PULSE_MAX_US) us = PWM_PULSE_MAX_US;

    tiltServo.writeMicroseconds(us);
}

void HalPwm::setNeutral() {
    setTiltPulseWidth(PWM_PULSE_NEUTRAL);
}

void HalPwm::arm(uint32_t now_ms) {
    switch (armState_) {
        case ArmState::IDLE:
            // Begin arming: send minimum pulse
            setTiltPulseWidth(PWM_PULSE_MIN_US);
            armStartMs_ = now_ms;
            armState_ = ArmState::ARMING;
            Serial.println("[PWM] Arming ESC...");
            break;

        case ArmState::ARMING:
            // Wait for arm duration to elapse
            if ((now_ms - armStartMs_) >= ESC_ARM_DURATION_MS) {
                setNeutral();
                armState_ = ArmState::ARMED;
                Serial.println("[PWM] ESC armed");
            }
            break;

        case ArmState::ARMED:
            // Already armed — nothing to do
            break;
    }
}

bool HalPwm::isArmed() {
    return armState_ == ArmState::ARMED;
}
