// Solar wing tracker for Teensy 4.1
// Reads two LDRs, drives GreenJay ESC via PID to keep them balanced.
// Circuit: 3.3V --- LDR --- A_pin --- 10k --- GND (same as main_ldr_compare)
// Sign convention: A is the trailing-edge LDR, B is the leading-edge.
//   rawA > rawB  =>  error_deg > 0  =>  pulse > neutral  =>  tilt toward A
// Swap A/B wires or negate SENSOR_TRACK_GAIN in config if motor runs backwards.

#include <Arduino.h>
#include "config/config.h"
#include "hal/hal_greenjay_esc.h"

static constexpr uint8_t LDR_PIN_A   = A0;
static constexpr uint8_t LDR_PIN_B   = A1;
static constexpr float   LDR_R_FIXED = 10000.0f;
static constexpr int     LDR_ADC_MAX = 4095;

static HalGreenJayEsc wingEsc(PIN_ESC_PWM);

// ---------------------------------------------------------------------------
// LDR resistance helper (identical to main_ldr_compare)
// ---------------------------------------------------------------------------
static float ldrResistance(int adc) {
    if (adc == 0) return 1e9f;
    return LDR_R_FIXED * (LDR_ADC_MAX - adc) / static_cast<float>(adc);
}

// ---------------------------------------------------------------------------
// PID — error in degrees (ADC diff * SENSOR_TRACK_GAIN)
// Output is an absolute pulse width in microseconds.
// Anti-windup: back-calculation with TILT_PID_KAW.
// ---------------------------------------------------------------------------
static float s_integral  = 0.0f;
static float s_prevError = 0.0f;

static uint16_t pidUpdate(float error_deg, float dt_s) {
    if (fabsf(error_deg) < TILT_PID_DEADBAND_DEG) {
        s_integral *= 0.95f;  // bleed integral in deadband
        return PWM_PULSE_NEUTRAL;
    }

    float deriv     = (error_deg - s_prevError) / dt_s;
    s_prevError     = error_deg;

    float unclamped = static_cast<float>(PWM_PULSE_NEUTRAL)
                    + TILT_PID_KP * error_deg
                    + s_integral
                    + TILT_PID_KD * deriv;

    float clamped = constrain(unclamped,
                              static_cast<float>(TILT_PID_OUT_MIN),
                              static_cast<float>(TILT_PID_OUT_MAX));

    s_integral += (TILT_PID_KI * error_deg
                   + TILT_PID_KAW * (clamped - unclamped)) * dt_s;

    return static_cast<uint16_t>(clamped);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("=== LDR Solar Tracker (Teensy 4.1) ===");
    Serial.printf("  PID: Kp=%.2f Ki=%.2f Kd=%.2f  deadband=%.2f deg\n",
                  TILT_PID_KP, TILT_PID_KI, TILT_PID_KD, TILT_PID_DEADBAND_DEG);
    Serial.printf("  PWM: neutral=%d  range=[%d, %d] us\n",
                  PWM_PULSE_NEUTRAL, TILT_PID_OUT_MIN, TILT_PID_OUT_MAX);

    analogReadResolution(12);
    wingEsc.init();
}

void loop() {
    static uint32_t lastLoopMs  = 0;
    static uint32_t lastPrintMs = 0;
    static uint32_t prevMs      = 0;

    uint32_t now = millis();

    if (!wingEsc.isArmed()) {
        wingEsc.arm(now);
        return;
    }

    if (now - lastLoopMs < MAIN_LOOP_INTERVAL_MS) return;
    float dt_s  = (prevMs == 0) ? (MAIN_LOOP_INTERVAL_MS / 1000.0f)
                                : (now - prevMs) / 1000.0f;
    prevMs     = now;
    lastLoopMs = now;

    int   rawA      = analogRead(LDR_PIN_A);
    int   rawB      = analogRead(LDR_PIN_B);
    float error_deg = static_cast<float>(rawA - rawB) * SENSOR_TRACK_GAIN;

    uint16_t pulse = pidUpdate(error_deg, dt_s);
    wingEsc.setPulseWidth(pulse);

    if (now - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        float rA = ldrResistance(rawA);
        float rB = ldrResistance(rawB);
        const char* status = fabsf(error_deg) < TILT_PID_DEADBAND_DEG ? "BALANCED"
                           : error_deg > 0                             ? "=> tilt right"
                                                                       : "=> tilt left";
        Serial.printf("A:%4d(%.0f ohm)  B:%4d(%.0f ohm)  err:%+.2f deg  pulse:%d us  %s\n",
                      rawA, rA, rawB, rB, error_deg, pulse, status);
    }
}
