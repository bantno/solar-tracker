#ifndef HAL_PIN_EXTENDER_H
#define HAL_PIN_EXTENDER_H

#include <cstdint>
#include <Servo.h>
#include "hal_button.h"

class IExtenderMotor {
public:
    virtual ~IExtenderMotor() = default;
    virtual void init()                     = 0;
    virtual void setPulseWidth(uint16_t us) = 0;
    virtual void stop()                     = 0;
    virtual void arm(uint32_t now_ms)       = 0;
    virtual bool isArmed()                  = 0;
    virtual bool isExtended()               = 0;
    virtual void clearExtended()            = 0;
};

class HalPinExtender : public IExtenderMotor {
public:
    HalPinExtender(uint8_t pwmPin, uint8_t limitPin);
    void init() override;
    void setPulseWidth(uint16_t us) override;
    void stop() override;
    void arm(uint32_t now_ms) override;
    bool isArmed() override;
    bool isExtended() override;
    void clearExtended() override;

private:
    enum class ArmState : uint8_t { IDLE, ARMING, ARMED };
    Servo     servo_;
    uint8_t   pwmPin_;
    ArmState  armState_   = ArmState::IDLE;
    uint32_t  armStartMs_ = 0;
    HalButton limitSwitch_;
};

#endif // HAL_PIN_EXTENDER_H
