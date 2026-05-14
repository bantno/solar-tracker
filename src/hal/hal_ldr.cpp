#include "hal_ldr.h"
#include <Arduino.h>
#include <cmath>

// Store the pin and calibration constants so every read() can use them.
HalLdr::HalLdr(uint8_t pin, float A, float gamma, float rFixed, int adcMax)
    : pin_(pin), A_(A), gamma_(gamma), rFixed_(rFixed), adcMax_(adcMax) {}

int HalLdr::read() {
    // Step 1 — sample the ADC (12-bit: 0 = 0 V, 4095 = 3.3 V)
    lastAdc_ = analogRead(pin_);

    // Step 2 — voltage divider → LDR resistance
    //   The circuit is: 3.3V ── LDR ── pin ── rFixed ── GND
    //   Rearranging the divider:  R_ldr = rFixed × (adcMax - adc) / adc
    //   Guard against adc == 0 (pin shorted to GND) by returning a huge resistance.
    lastR_ = (lastAdc_ <= 0) ? 1.0e9f
                             : rFixed_ * (adcMax_ - lastAdc_) / static_cast<float>(lastAdc_);

    // Step 3 — resistance → lux via calibrated power-law inversion
    //   Model fitted in MATLAB:  R = A * L^(-gamma)
    //   Solved for L:            L = (A / R)^(1/gamma)
    lastLux_ = (lastR_ > 0.0f) ? powf(A_ / lastR_, 1.0f / gamma_) : 0.0f;

    return lastAdc_;
}
