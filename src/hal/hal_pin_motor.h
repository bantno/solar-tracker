#ifndef HAL_PIN_MOTOR_H
#define HAL_PIN_MOTOR_H

#include <cstdint>

// Pin motor direction commands
enum class PinMotorDir : uint8_t {
    FORWARD,
    REVERSE,
    STOP
};

// Abstract pin motor interface
class IPinMotor {
public:
    virtual ~IPinMotor() = default;
    virtual void init() = 0;
    virtual void setPinMotor(uint8_t motorIndex, PinMotorDir dir, uint8_t speed) = 0;
};

// Stub implementation — H-bridge / driver IC not yet selected.
// Prints commands to Serial for debugging until hardware is finalized.
class HalPinMotor : public IPinMotor {
public:
    void init() override;
    void setPinMotor(uint8_t motorIndex, PinMotorDir dir, uint8_t speed) override;
};

#endif // HAL_PIN_MOTOR_H
