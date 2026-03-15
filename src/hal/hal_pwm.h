#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <cstdint>

// Abstract PWM interface for tilt actuator control
class IPwm {
public:
    virtual ~IPwm() = default;
    virtual bool init()                        = 0;
    virtual void setTiltPulseWidth(uint16_t us) = 0;
    virtual void setNeutral()                   = 0;
    virtual void arm(uint32_t now_ms)           = 0;
    virtual bool isArmed()                      = 0;
};

// Concrete implementation using Servo library for 50 Hz PWM
class HalPwm : public IPwm {
public:
    bool init() override;
    void setTiltPulseWidth(uint16_t us) override;
    void setNeutral() override;
    void arm(uint32_t now_ms) override;
    bool isArmed() override;

private:
    enum class ArmState : uint8_t {
        IDLE,
        ARMING,
        ARMED
    };

    ArmState armState_    = ArmState::IDLE;
    uint32_t armStartMs_  = 0;
};

#endif // HAL_PWM_H
