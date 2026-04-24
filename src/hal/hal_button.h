#ifndef HAL_BUTTON_H
#define HAL_BUTTON_H

#include <cstdint>

class IButton {
public:
    virtual ~IButton() = default;
    virtual void init()      = 0;
    virtual bool isPressed() = 0;
    virtual void clear()     = 0;
};

// Active-low button with interrupt detection and glitch self-correction.
// Fires on FALLING edge (INPUT_PULLUP). isPressed() validates the pin is
// still LOW before returning true, so noise spikes auto-clear the flag.
class HalButton : public IButton {
public:
    static constexpr uint8_t MAX_INSTANCES = 4;

    explicit HalButton(uint8_t pin);
    void init() override;
    bool isPressed() override;
    void clear() override;

    uint8_t pin() const { return pin_; }
    void trigger(); // called by ISR dispatch — do not call directly

private:
    uint8_t       pin_;
    volatile bool triggered_ = false;
};

#endif // HAL_BUTTON_H
