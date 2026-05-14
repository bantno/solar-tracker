#ifndef HAL_LDR_H
#define HAL_LDR_H

#include <cstdint>

// -----------------------------------------------------------------------------
// HalLdr — calibrated light-dependent resistor driver
//
// Each LDR sits in a voltage divider with a fixed 10 kΩ resistor:
//
//   3.3V ── LDR ── A_pin ── 10kΩ ── GND
//
// The Teensy reads the voltage at A_pin as a 12-bit ADC value (0–4095).
// HalLdr converts that raw number into:
//   1. Resistance (ohms) — using the voltage divider equation
//   2. Illuminance (lux) — using a sensor-specific power-law calibration
//
// Calibration model (fitted in MATLAB from measured lux vs. resistance data):
//   R = A * L^(-gamma)   →   L = (A / R)^(1/gamma)
//
// Usage — call read() once per loop, then use the getters:
//   int rawAdc = ldrA.read();          // triggers one analogRead, caches results
//   float r    = ldrA.resistanceOhms(); // last measured resistance
//   float l    = ldrA.lux();            // last calibration-corrected brightness
// -----------------------------------------------------------------------------
class HalLdr {
public:
    // pin    — Teensy analog pin the LDR is wired to (e.g. A0, A1)
    // A      — calibration constant from power-law fit  (units: Ω·lux^gamma)
    // gamma  — calibration exponent from power-law fit  (dimensionless)
    // rFixed — fixed resistor in the voltage divider    (default 10 kΩ)
    // adcMax — ADC full-scale value                     (default 4095 for 12-bit)
    HalLdr(uint8_t pin, float A, float gamma,
           float rFixed = 10000.0f, int adcMax = 4095);

    // Reads the ADC pin once, computes and caches resistance and lux.
    // Returns the raw 12-bit ADC value (0–4095).
    // Call this exactly once per loop iteration before using the getters below.
    int   read();

    // Returns the LDR resistance in ohms from the most recent read().
    float resistanceOhms() const { return lastR_; }

    // Returns the calibration-corrected illuminance in lux from the most recent read().
    float lux()            const { return lastLux_; }

private:
    uint8_t pin_;     // analog pin number
    float   A_;       // power-law calibration constant A
    float   gamma_;   // power-law calibration exponent γ
    float   rFixed_;  // fixed voltage-divider resistor (ohms)
    int     adcMax_;  // ADC full-scale (4095 for 12-bit)

    int   lastAdc_ = 0;      // most recent raw ADC reading
    float lastR_   = 1.0e9f; // most recent resistance (ohms); starts at ~∞ (no light)
    float lastLux_ = 0.0f;   // most recent illuminance (lux)
};

#endif // HAL_LDR_H
