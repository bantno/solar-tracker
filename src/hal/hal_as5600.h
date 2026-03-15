#ifndef HAL_AS5600_H
#define HAL_AS5600_H

#include <cstdint>

// Abstract encoder interface — enables mocking and driver swaps
class IEncoder {
public:
    virtual ~IEncoder() = default;
    virtual bool     init()             = 0;
    virtual float    readAngleDegrees() = 0;
    virtual uint16_t readRawCounts()    = 0;
};

// Concrete implementation using AS5600 magnetic rotary encoder over I2C
class HalAs5600 : public IEncoder {
public:
    bool     init() override;
    float    readAngleDegrees() override;
    uint16_t readRawCounts() override;

private:
    bool sensorPresent_ = false;
};

#endif // HAL_AS5600_H
