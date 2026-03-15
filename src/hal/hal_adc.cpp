#include "hal_adc.h"
#include "../config/config.h"
#include <Arduino.h>

void HalAdc::init() {
    analogReadResolution(ADC_RESOLUTION_BITS);

    pinMode(PIN_QUAD_TOP,    INPUT);
    pinMode(PIN_QUAD_BOTTOM, INPUT);
    pinMode(PIN_QUAD_LEFT,   INPUT);
    pinMode(PIN_QUAD_RIGHT,  INPUT);
    pinMode(PIN_IRRADIANCE,  INPUT);
    pinMode(PIN_WIND,        INPUT);
    pinMode(PIN_TWILIGHT,    INPUT);

    Serial.println("[ADC] Initialized, 12-bit resolution");
}

float HalAdc::readIrradiance() {
    uint16_t raw = analogRead(PIN_IRRADIANCE);
    return static_cast<float>(raw) * IRRADIANCE_CAL_FACTOR;
}

float HalAdc::readWindSpeed() {
    uint16_t raw = analogRead(PIN_WIND);
    return static_cast<float>(raw) * WIND_CAL_FACTOR;
}

uint16_t HalAdc::readQuadSensor(QuadChannel ch) {
    switch (ch) {
        case QuadChannel::TOP:    return analogRead(PIN_QUAD_TOP);
        case QuadChannel::BOTTOM: return analogRead(PIN_QUAD_BOTTOM);
        case QuadChannel::LEFT:   return analogRead(PIN_QUAD_LEFT);
        case QuadChannel::RIGHT:  return analogRead(PIN_QUAD_RIGHT);
    }
    return 0;
}

float HalAdc::readTwilight() {
    uint16_t raw = analogRead(PIN_TWILIGHT);
    return static_cast<float>(raw) * TWILIGHT_CAL_FACTOR;
}
