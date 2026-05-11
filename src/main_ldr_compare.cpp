// LDR voltage divider comparison on Teensy 4.1
// Circuit: 3.3V --- LDR --- A_pin --- 10k --- GND

#include <Arduino.h>
#include <SD.h>

static constexpr uint8_t  LDR_PIN_A   = A0;
static constexpr uint8_t  LDR_PIN_B   = A1;
static constexpr float    R_FIXED     = 10000.0f; // 10k pull-down
static constexpr int      ADC_MAX     = 4095;
static constexpr uint32_t PRINT_MS    = 200;
static constexpr uint32_t LOG_MS      = 30000;
static constexpr char     LOG_FILE[]  = "ldr_log.csv";

static bool sdReady = false;

static float ldrResistance(int adc) {
    if (adc == 0) return 1e9f;
    return R_FIXED * (ADC_MAX - adc) / static_cast<float>(adc);
}

static void logToSD(uint32_t t_ms, int rawA, int rawB, int diff) {
    File f = SD.open(LOG_FILE, FILE_WRITE);
    if (!f) {
        Serial.println("[SD] open failed");
        return;
    }
    f.printf("%lu,%d,%d,%d\n", t_ms / 1000, rawA, rawB, diff);
    f.close();
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    sdReady = SD.begin(BUILTIN_SDCARD);
    if (sdReady) {
        // Write header if file is new
        if (!SD.exists(LOG_FILE)) {
            File f = SD.open(LOG_FILE, FILE_WRITE);
            if (f) { f.println("t_s,raw_a,raw_b,diff"); f.close(); }
        }
        Serial.println("[SD] ready");
    } else {
        Serial.println("[SD] not found — logging disabled");
    }
}

void loop() {
    static uint32_t lastPrint = 0;
    static uint32_t lastLog   = 0;
    uint32_t now = millis();

    int rawA = analogRead(LDR_PIN_A);
    int rawB = analogRead(LDR_PIN_B);
    int diff = rawA - rawB;

    if (now - lastPrint >= PRINT_MS) {
        lastPrint = now;
        float rA = ldrResistance(rawA);
        float rB = ldrResistance(rawB);
        Serial.printf("A: %4d (%.0f ohm)  B: %4d (%.0f ohm)  diff: %+d  ",
                      rawA, rA, rawB, rB, diff);
        if (abs(diff) < 20)     Serial.println("=> balanced");
        else if (diff > 0)      Serial.println("=> A brighter");
        else                    Serial.println("=> B brighter");
    }

    if (sdReady && (now - lastLog >= LOG_MS)) {
        lastLog = now;
        logToSD(now, rawA, rawB, diff);
    }
}
