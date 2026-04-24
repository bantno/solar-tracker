#ifndef HAL_GREENJAY_ESC_H
#define HAL_GREENJAY_ESC_H

#include <cstdint>
#include <Servo.h>

// Abstract ESC interface
class IEsc {
public:
    virtual ~IEsc() = default;
    virtual void init()                     = 0;
    virtual void arm(uint32_t now_ms)       = 0;
    virtual bool isArmed()                  = 0;
    virtual void setPulseWidth(uint16_t us) = 0;
    virtual void stop()                     = 0;
};

// GreenJay ESC arming sequence:
//   Phase 1 — send 1503 us for 2 s
//   Phase 2 — send 1475 us for 1.5 s
//   Armed   — return to neutral (1503 us)
class HalGreenJayEsc : public IEsc {
public:
    explicit HalGreenJayEsc(uint8_t pin);
    void init() override;
    void arm(uint32_t now_ms) override;
    bool isArmed() override;
    void setPulseWidth(uint16_t us) override;
    void stop() override;

private:
    enum class ArmState : uint8_t { IDLE, PHASE1, PHASE2, ARMED };

    Servo    servo_;
    uint8_t  pin_;
    ArmState armState_    = ArmState::IDLE;
    uint32_t phaseStartMs_ = 0;
};

#endif // HAL_GREENJAY_ESC_H
