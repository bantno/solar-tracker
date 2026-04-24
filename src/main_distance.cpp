#include <Arduino.h>
#include "config/config.h"
#include "SparkFun_Qwiic_XM125_Arduino_Library.h"

static SparkFunXM125Distance radar;
static uint32_t lastSampleMs = 0;

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
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("=== XM125 Distance Node ===");

    Wire.begin();

    if (!radar.begin()) {
        Serial.println("[DIST] ERROR: sensor not found — halting");
        while (1);
    }

    if (radar.distanceSetup(DISTANCE_BEGIN_MM, DISTANCE_END_MM) != ksfTkErrOk) {
        Serial.println("[DIST] ERROR: distance setup failed");
    }

    Serial.print("[DIST] range=");
    Serial.print(DISTANCE_BEGIN_MM); Serial.print("-");
    Serial.print(DISTANCE_END_MM);   Serial.println("mm  ready");
}

void loop() {
    uint32_t now = millis();
    if ((now - lastSampleMs) < DISTANCE_SAMPLE_MS) return;
    lastSampleMs = now;
    runDetector();
}
