#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <cstdint>

// Quad photo-sensor channel identifiers
enum class QuadChannel : uint8_t {
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
};

// Abstract ADC interface — enables mocking for desktop tests
class IAdc {
public:
    virtual ~IAdc() = default;
    virtual void     init()                         = 0;
    virtual float    readIrradiance()               = 0;
    virtual float    readWindSpeed()                 = 0;
    virtual uint16_t readQuadSensor(QuadChannel ch) = 0;
    virtual float    readTwilight()                  = 0;
};

// Concrete implementation using Teensy 4.0 analog inputs
class HalAdc : public IAdc {
public:
    void     init() override;
    float    readIrradiance() override;
    float    readWindSpeed() override;
    uint16_t readQuadSensor(QuadChannel ch) override;
    float    readTwilight() override;
};

#endif // HAL_ADC_H
