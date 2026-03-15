#include "hal_as5600.h"
#include "../config/config.h"
#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

static AS5600 sensor;

bool HalAs5600::init() {
    Wire.begin();

    // Check if AS5600 is present on the I2C bus
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    uint8_t error = Wire.endTransmission();

    sensorPresent_ = (error == 0);
    if (!sensorPresent_) {
        Serial.println("[AS5600] Sensor not found on I2C bus");
    } else {
        Serial.println("[AS5600] Sensor detected");
    }
    return sensorPresent_;
}

float HalAs5600::readAngleDegrees() {
    if (!sensorPresent_) {
        return -1.0f;
    }
    // AS5600 library returns angle in degrees (0.0–360.0)
    return sensor.readAngle() * (360.0f / 4096.0f);
}

uint16_t HalAs5600::readRawCounts() {
    if (!sensorPresent_) {
        return 0;
    }
    // Raw 12-bit value (0–4095)
    return sensor.readAngle();
}
