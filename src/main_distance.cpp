#include <Arduino.h>
#include "config/config.h"
#include "SparkFun_Qwiic_XM125_Arduino_Library.h"

static SparkFunXM125Distance radar;
static uint32_t lastSampleMs = 0;

// Onboard WS2812 RGB LED (IO18) used as a serial-independent status channel.
static const uint8_t RGB_PIN = 18;
static inline void rgb(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(RGB_PIN, r, g, b); }

static void printPeak(uint8_t idx, uint32_t distMm, int32_t strength) {
    if (distMm == 0) return;
    Serial.print("[DIST] peak="); Serial.print(idx);
    Serial.print("  dist=");      Serial.print(distMm); Serial.print("mm");
    Serial.print("  str=");       Serial.println(strength);
}

static void runDetector() {
    if (radar.start() != ksfTkErrOk) {
        Serial.println("[DIST] start error");
        return;
    }

    if (radar.busyWait() != ksfTkErrOk) {
        Serial.println("[DIST] busy wait error");
        return;
    }

    uint32_t measErr = 0;
    radar.getMeasureDistanceError(measErr);
    if (measErr == 1) {
        Serial.println("[DIST] measure distance error");
        return;
    }

    uint32_t calibNeeded = 0;
    radar.getCalibrationNeeded(calibNeeded);
    if (calibNeeded == 1) {
        Serial.println("[DIST] recalibrating...");
        radar.recalibrate();
        return;
    }

    uint32_t distMm  = 0;
    int32_t  strength = 0;
    for (uint8_t i = 0; i <= 9; i++) {
        radar.getPeakDistance(i, distMm);
        radar.getPeakStrength(i, strength);
        printPeak(i, distMm, strength);
    }
}

void setup() {
    // RGB status (works without serial): WHITE == reached setup().
    rgb(40, 40, 40);

    Serial.begin(115200);
    // ESP32-S2 native USB re-enumerates when the app's CDC comes up; give the
    // host time to (re)attach so the banner isn't lost during enumeration.
    while (!Serial && millis() < 8000) {}
    delay(200);
    Serial.println("=== XM125 Distance Node ===");

    // YELLOW == about to init I2C / talk to the radar.
    rgb(40, 40, 0);
    Wire.begin(DISTANCE_I2C_SDA, DISTANCE_I2C_SCL);

    if (!radar.begin()) {
        Serial.println("[DIST] ERROR: sensor not found — halting");
        // Solid RED forever == radar.begin() failed (I2C / wiring / pins).
        while (1) { rgb(80, 0, 0); delay(100); }
    }

    if (radar.distanceSetup(DISTANCE_BEGIN_MM, DISTANCE_END_MM) != ksfTkErrOk) {
        Serial.println("[DIST] ERROR: distance setup failed");
        // MAGENTA == sensor found but distanceSetup() failed.
        rgb(60, 0, 60);
    }

    Serial.print("[DIST] range=");
    Serial.print(DISTANCE_BEGIN_MM); Serial.print("-");
    Serial.print(DISTANCE_END_MM);   Serial.println("mm  ready");
}

void loop() {
    uint32_t now = millis();
    if ((now - lastSampleMs) < DISTANCE_SAMPLE_MS) return;
    lastSampleMs = now;
    // BLUE blip each loop == loop() is alive and scanning.
    rgb(0, 0, 60); delay(20); rgb(0, 10, 0);
    Serial.print("[DIST] t="); Serial.print(now); Serial.println("ms scanning...");
    runDetector();
}
