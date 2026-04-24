#include <Arduino.h>
#include "config/config.h"
#include "hal/hal_pin_extender.h"
#include "hal/hal_greenjay_esc.h"

// ---------------------------------------------------------------------------
// Serial commands — includes ANSI arrow key escape sequences
// ---------------------------------------------------------------------------
enum class Cmd : uint8_t {
    NONE,
    EXTEND, RETRACT, STOP,          // extender commands
    TILT_LEFT, TILT_RIGHT, TILT_STOP // tilt commands
};

static Cmd readSerial() {
    // Arrow keys send 3-byte sequences: ESC '[' A/B/C/D
    static enum class ParseState : uint8_t { NORMAL, ESC, CSI } ps = ParseState::NORMAL;

    while (Serial.available()) {
        char c = (char)Serial.read();
        switch (ps) {
            case ParseState::NORMAL:
                if (c == '\x1B') { ps = ParseState::ESC; break; }
                switch (c) {
                    case 'e': case 'E': Serial.println("[CMD] extend");     return Cmd::EXTEND;
                    case 'r': case 'R': Serial.println("[CMD] retract");    return Cmd::RETRACT;
                    case 's': case 'S': Serial.println("[CMD] stop");       return Cmd::STOP;
                    default: break;
                }
                break;
            case ParseState::ESC:
                ps = (c == '[') ? ParseState::CSI : ParseState::NORMAL;
                break;
            case ParseState::CSI:
                ps = ParseState::NORMAL;
                switch (c) {
                    case 'C': Serial.println("[CMD] tilt right"); return Cmd::TILT_RIGHT; // →
                    case 'D': Serial.println("[CMD] tilt left");  return Cmd::TILT_LEFT;  // ←
                    case 'A':                                                              // ↑
                    case 'B': Serial.println("[CMD] tilt stop");  return Cmd::TILT_STOP;  // ↓
                    default: break;
                }
                break;
        }
    }
    return Cmd::NONE;
}

// ---------------------------------------------------------------------------
// Extender state machine
// ---------------------------------------------------------------------------
enum class ExtState : uint8_t {
    IDLE,       // retracted, stopped — waiting for command
    EXTENDING,  // forward, waiting for limit switch
    EXTENDED,   // at limit, stopped — waiting for command
    RETRACTING  // reverse, timed
};

static const char* extStateName(ExtState s) {
    switch (s) {
        case ExtState::IDLE:       return "IDLE";
        case ExtState::EXTENDING:  return "EXTENDING";
        case ExtState::EXTENDED:   return "EXTENDED";
        case ExtState::RETRACTING: return "RETRACTING";
    }
    return "?";
}

struct ExtCtx {
    HalPinExtender extender;
    ExtState       state      = ExtState::IDLE;
    uint32_t       stateStart = 0;
    const char*    label;
};

static ExtCtx exts[2] = {
    { HalPinExtender(PIN_EXTENDER_PWM,  PIN_EXTENDER_LIMIT),  ExtState::IDLE, 0, "EXT1" },
    { HalPinExtender(PIN_EXTENDER2_PWM, PIN_EXTENDER2_LIMIT), ExtState::IDLE, 0, "EXT2" },
};

// ---------------------------------------------------------------------------
// Tilt motor (GreenJay ESC)
// ---------------------------------------------------------------------------
static HalGreenJayEsc tiltEsc(PIN_TILT_MOTOR_PWM);

static void tickTilt(Cmd cmd) {
    switch (cmd) {
        case Cmd::TILT_LEFT:  tiltEsc.setPulseWidth(TILT_MOTOR_PULSE_LEFT_US);  break;
        case Cmd::TILT_RIGHT: tiltEsc.setPulseWidth(TILT_MOTOR_PULSE_RIGHT_US); break;
        case Cmd::TILT_STOP:  tiltEsc.stop();                                   break;
        case Cmd::STOP:       tiltEsc.stop();                                   break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// System arming
// ---------------------------------------------------------------------------
static bool systemArmed() {
    return exts[0].extender.isArmed() &&
           exts[1].extender.isArmed() &&
           tiltEsc.isArmed();
}

// ---------------------------------------------------------------------------
// Extender tick
// ---------------------------------------------------------------------------
static void tickExtender(ExtCtx& c, uint32_t now, Cmd cmd) {
    // STOP and RETRACT override any state immediately
    if (cmd == Cmd::STOP) {
        c.extender.setPulseWidth(EXTENDER_PULSE_STOP_US);
        c.state = ExtState::IDLE;
        Serial.print("["); Serial.print(c.label); Serial.println("] Stopped");
        return;
    }
    if (cmd == Cmd::RETRACT) {
        c.extender.setPulseWidth(EXTENDER_PULSE_REV_US);
        c.stateStart = now;
        c.state      = ExtState::RETRACTING;
        Serial.print("["); Serial.print(c.label); Serial.println("] Retracting...");
        return;
    }

    switch (c.state) {
        case ExtState::IDLE:
            if (cmd == Cmd::EXTEND) {
                c.extender.clearExtended();
                c.extender.setPulseWidth(EXTENDER_PULSE_FWD_US);
                c.stateStart = now;
                c.state      = ExtState::EXTENDING;
                Serial.print("["); Serial.print(c.label); Serial.println("] Extending...");
            }
            break;

        case ExtState::EXTENDING:
            if (c.extender.isExtended()) {
                c.extender.setPulseWidth(EXTENDER_PULSE_STOP_US);
                c.state = ExtState::EXTENDED;
                Serial.print("["); Serial.print(c.label); Serial.println("] Limit reached — idle extended");
            } else if ((now - c.stateStart) >= EXTENDER_TIMEOUT_MS) {
                c.extender.setPulseWidth(EXTENDER_PULSE_STOP_US);
                c.state = ExtState::IDLE;
                Serial.print("["); Serial.print(c.label); Serial.println("] WARN: extend timeout");
            }
            break;

        case ExtState::EXTENDED:
            break;

        case ExtState::RETRACTING:
            if ((now - c.stateStart) >= EXTENDER_RETRACT_MS) {
                c.extender.setPulseWidth(EXTENDER_PULSE_STOP_US);
                c.state = ExtState::IDLE;
                Serial.print("["); Serial.print(c.label); Serial.println("] Retract complete — idle");
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
static uint32_t lastPrintMs  = 0;
static Cmd      lastTiltCmd  = Cmd::TILT_STOP;

static const char* tiltStateName(Cmd c) {
    switch (c) {
        case Cmd::TILT_LEFT:  return "LEFT";
        case Cmd::TILT_RIGHT: return "RIGHT";
        default:              return "STOP";
    }
}

static void printTelemetry(uint32_t now) {
    if ((now - lastPrintMs) < SERIAL_PRINT_INTERVAL_MS) return;
    lastPrintMs = now;

    for (auto& c : exts) {
        Serial.print("["); Serial.print(c.label); Serial.print("] ");
        Serial.print("state="); Serial.print(extStateName(c.state));
        Serial.print("  limit=");
        Serial.print(c.extender.isExtended() ? "TRIGGERED" : "open");
        if (c.state == ExtState::EXTENDING) {
            Serial.print("  elapsed=");
            Serial.print((now - c.stateStart) / 1000.0f, 1);
            Serial.print("s / "); Serial.print(EXTENDER_TIMEOUT_MS / 1000); Serial.print("s");
        } else if (c.state == ExtState::RETRACTING) {
            Serial.print("  retract=");
            Serial.print((now - c.stateStart) / 1000.0f, 1);
            Serial.print("s / "); Serial.print(EXTENDER_RETRACT_MS / 1000); Serial.print("s");
        }
        Serial.println();
    }

    Serial.print("[TILT]  armed="); Serial.print(tiltArmed ? "YES" : "NO");
    Serial.print("  dir="); Serial.println(tiltStateName(lastTiltCmd));
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("=== Wing Deploy Node ===");
    Serial.println("Extenders: e=extend  r=retract  s=stop all");
    Serial.println("Tilt:      ←/→=direction  ↑/↓/s=stop");

    for (auto& c : exts) c.extender.init();
    tiltServo.attach(PIN_TILT_MOTOR_PWM, PWM_PULSE_MIN_US, PWM_PULSE_MAX_US);

    delay(1000);
}

void loop() {
    uint32_t now = millis();
    Cmd      cmd = readSerial();

    if (!systemArmed()) {
        for (auto& c : exts) c.extender.arm(now);
        tiltEsc.arm(now);
        printTelemetry(now);
        return;
    }

    // Track last tilt direction for telemetry
    if (cmd == Cmd::TILT_LEFT || cmd == Cmd::TILT_RIGHT ||
        cmd == Cmd::TILT_STOP || cmd == Cmd::STOP) {
        lastTiltCmd = cmd;
    }

    tickTilt(cmd);
    for (auto& c : exts) tickExtender(c, now, cmd);

    printTelemetry(now);
}
