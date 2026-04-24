#include <Arduino.h>
#include "config/config.h"
#include "hal/hal_pin_extender.h"

enum class ExtenderState : uint8_t {
    ARMING,
    EXTENDING,
    RETRACTING,
    STOPPED,
    TIMEOUT_ERROR
};

static const char* stateName(ExtenderState s) {
    switch (s) {
        case ExtenderState::ARMING:        return "ARMING";
        case ExtenderState::EXTENDING:     return "EXTENDING";
        case ExtenderState::RETRACTING:    return "RETRACTING";
        case ExtenderState::STOPPED:       return "STOPPED";
        case ExtenderState::TIMEOUT_ERROR: return "TIMEOUT_ERROR";
    }
    return "?";
}

struct ExtenderCtx {
    HalPinExtender extender;
    ExtenderState  state      = ExtenderState::ARMING;
    uint32_t       stateStartMs = 0;
    const char*    label;
};

static ExtenderCtx ctx[2] = {
    { HalPinExtender(PIN_EXTENDER_PWM,  PIN_EXTENDER_LIMIT),  ExtenderState::ARMING, 0, "EXT1" },
    { HalPinExtender(PIN_EXTENDER2_PWM, PIN_EXTENDER2_LIMIT), ExtenderState::ARMING, 0, "EXT2" },
};

static uint32_t lastPrintMs = 0;

static void tickExtender(ExtenderCtx& c, uint32_t now) {
    switch (c.state) {
        case ExtenderState::ARMING:
            c.extender.arm(now);
            if (c.extender.isArmed()) {
                c.extender.clearExtended();
                c.extender.setPulseWidth(EXTENDER_PULSE_FWD_US);
                c.stateStartMs = now;
                c.state        = ExtenderState::EXTENDING;
                Serial.print("[");
                Serial.print(c.label);
                Serial.println("] Extending...");
            }
            break;

        case ExtenderState::EXTENDING:
            if (c.extender.isExtended()) {
                c.extender.setPulseWidth(EXTENDER_PULSE_REV_US);
                c.stateStartMs = now;
                c.state        = ExtenderState::RETRACTING;
                Serial.print("[");
                Serial.print(c.label);
                Serial.println("] Button triggered — retracting for 30 s");
            } else if ((now - c.stateStartMs) >= EXTENDER_TIMEOUT_MS) {
                c.extender.setPulseWidth(EXTENDER_PULSE_STOP_US);
                c.state = ExtenderState::TIMEOUT_ERROR;
                Serial.print("[");
                Serial.print(c.label);
                Serial.println("] ERROR: timeout — limit switch not reached");
            }
            break;

        case ExtenderState::RETRACTING:
            if ((now - c.stateStartMs) >= EXTENDER_RETRACT_MS) {
                c.extender.setPulseWidth(EXTENDER_PULSE_STOP_US);
                c.state = ExtenderState::STOPPED;
                Serial.print("[");
                Serial.print(c.label);
                Serial.println("] Retract complete — stopped");
            }
            break;

        case ExtenderState::STOPPED:
        case ExtenderState::TIMEOUT_ERROR:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("=== Pin Extender Node (2x) ===");
    for (auto& c : ctx) c.extender.init();
    delay(1000);
}

void loop() {
    uint32_t now = millis();

    for (auto& c : ctx) tickExtender(c, now);

    // 2 Hz telemetry
    if ((now - lastPrintMs) >= SERIAL_PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        for (auto& c : ctx) {
            Serial.print("[");
            Serial.print(c.label);
            Serial.print("] state=");
            Serial.print(stateName(c.state));
            if (c.state == ExtenderState::EXTENDING) {
                Serial.print("  elapsed=");
                Serial.print((now - c.stateStartMs) / 1000.0f, 1);
                Serial.print("s / ");
                Serial.print(EXTENDER_TIMEOUT_MS / 1000);
                Serial.print("s");
            } else if (c.state == ExtenderState::RETRACTING) {
                Serial.print("  retract=");
                Serial.print((now - c.stateStartMs) / 1000.0f, 1);
                Serial.print("s / ");
                Serial.print(EXTENDER_RETRACT_MS / 1000);
                Serial.print("s");
            }
            Serial.println();
        }
    }
}
