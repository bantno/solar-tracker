#include "hal_button.h"
#include <Arduino.h>

static HalButton* s_instances[HalButton::MAX_INSTANCES] = {};
static uint8_t    s_count = 0;

// One ISR per slot — Teensy requires plain function pointers, not lambdas
static void isr0() { s_instances[0]->trigger(); }
static void isr1() { s_instances[1]->trigger(); }
static void isr2() { s_instances[2]->trigger(); }
static void isr3() { s_instances[3]->trigger(); }
static void (*s_isrs[HalButton::MAX_INSTANCES])() = { isr0, isr1, isr2, isr3 };

HalButton::HalButton(uint8_t pin) : pin_(pin) {}

void HalButton::init() {
    pinMode(pin_, INPUT_PULLUP);
    if (s_count < MAX_INSTANCES) {
        s_instances[s_count] = this;
        attachInterrupt(digitalPinToInterrupt(pin_), s_isrs[s_count], FALLING);
        s_count++;
    }
}

void HalButton::trigger() {
    triggered_ = true;
}

bool HalButton::isPressed() {
    if (!triggered_) return false;
    // Confirm pin is still LOW — if it recovered, the trigger was a glitch
    if (digitalRead(pin_) == HIGH) {
        triggered_ = false;
        return false;
    }
    return true;
}

void HalButton::clear() {
    triggered_ = false;
}
