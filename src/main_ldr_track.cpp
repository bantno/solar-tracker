// Solar wing tracker for Teensy 4.1
// Reads two LDRs, drives GreenJay ESC via PID to keep them balanced.
// Circuit: 3.3V --- LDR --- A_pin --- 10k --- GND (same as main_ldr_compare)
// Sign convention: A is the trailing-edge LDR, B is the leading-edge.
//   rawA > rawB  =>  error_deg > 0  =>  pulse > neutral  =>  tilt toward A
// Swap A/B wires or negate SENSOR_TRACK_GAIN in config if motor runs backwards.

#include <Arduino.h>
#include <SD.h>
#include "config/config.h"
#include "hal/hal_greenjay_esc.h"
#include "hal/hal_as5600.h"
#include "hal/hal_ldr.h"

static constexpr uint32_t LOG_MS = 2000;

static bool sdReady  = false;
static char logFile[24];  // e.g. "log1.csv", "log99.csv"

static HalGreenJayEsc wingEsc(PIN_ESC_PWM);
static HalAs5600      encoder;

// ---------------------------------------------------------------------------
// LDR resistance helper — calibrated via power-law fit (R = A * L^-gamma)
// LDR 1 (A0, black wire): A=4.8641e5, gamma=0.5882  R²=0.9885
// LDR 2 (A1, red wire):   A=2.7794e5, gamma=0.4798  R²=0.9938
// ---------------------------------------------------------------------------
static HalLdr ldrA(A1, 4.8641e5f, 0.5882f);
static HalLdr ldrB(A2, 2.7794e5f, 0.4798f);

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
// SD logging helpers
// ---------------------------------------------------------------------------
static void logEncoderToSD(uint32_t t_ms,
                           uint16_t rawCounts,
                           float angleDeg,
                           float error_deg,
                           uint16_t pulse) {
    File f = SD.open(logFile, FILE_WRITE);
    if (!f) {
        Serial.println("[SD] open failed");
        return;
    }
    if constexpr (LOG_MS < 1000) {
        f.printf("%.2f,%u,%.2f,%+.2f,%u\n",
                 t_ms / 1000.0f,
                 rawCounts,
                 angleDeg,
                 error_deg,
                 pulse);
    } else {
        f.printf("%lu,%u,%.2f,%+.2f,%u\n",
                 t_ms / 1000,
                 rawCounts,
                 angleDeg,
                 error_deg,
                 pulse);
    }
    f.close();
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

    bool encoderReady = encoder.init();
    if (!encoderReady) {
        Serial.println("[AS5600] encoder init failed");
    }

    sdReady = SD.begin(BUILTIN_SDCARD);
    if (sdReady) {
        // Find the next unused log number (log1.csv, log2.csv, ...)
        int logNum = 1;
        do {
            snprintf(logFile, sizeof(logFile), "log%d.csv", logNum++);
        } while (SD.exists(logFile));

        // Create the new file with a header row
        File f = SD.open(logFile, FILE_WRITE);
        if (f) {
            f.println("t_s,raw_counts,angle_deg,error_deg,pulse_us");
            f.close();
        }
        Serial.printf("[SD] logging to %s\n", logFile);
    } else {
        Serial.println("[SD] not found — logging disabled");
    }
}

void loop() {
    static uint32_t lastLoopMs  = 0;
    static uint32_t lastPrintMs = 0;
    static uint32_t lastLogMs   = 0;
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

    int      rawA      = ldrA.read();
    int      rawB      = ldrB.read();
    float    error_deg = static_cast<float>(rawA - rawB) * 1; 
    float    angleDeg  = encoder.readAngleDegrees();
    uint16_t rawCounts = encoder.readRawCounts();

    uint16_t pulse = pidUpdate(error_deg, dt_s);
    wingEsc.setPulseWidth(pulse);

    if (now - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        const char* status = fabsf(error_deg) < TILT_PID_DEADBAND_DEG ? "BALANCED"
                           : error_deg > 0                             ? "=> tilt right"
                                                                       : "=> tilt left";
        Serial.printf("A:%4d(%.0f ohm / %.1f lux)  B:%4d(%.0f ohm / %.1f lux)  err:%+.2f deg  pulse:%d us  %s\n",
                      rawA, ldrA.resistanceOhms(), ldrA.lux(),
                      rawB, ldrB.resistanceOhms(), ldrB.lux(),
                      error_deg, pulse, status);
        Serial.printf("  Encoder: raw=%u  angle=%.2f deg\n", rawCounts, angleDeg);
    }

    if (sdReady && (now - lastLogMs >= LOG_MS)) {
        lastLogMs = now;
        logEncoderToSD(now, rawCounts, angleDeg, error_deg, pulse);
    }
}
