#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ============================================================================
// Pin Assignments
// ============================================================================

// ESC / Servo PWM output for wing tilt actuator
constexpr uint8_t PIN_ESC_PWM = 2;

// AS5600 rotary encoder I2C (Teensy 4.0 default I2C0)
constexpr uint8_t PIN_I2C_SDA = 18;       // Teensy 4.0 Wire SDA
constexpr uint8_t PIN_I2C_SCL = 19;       // Teensy 4.0 Wire SCL

// Quad photo-sensor analog inputs (4-quadrant sun sensor)
constexpr uint8_t PIN_QUAD_TOP    = 14;   // Analog A0
constexpr uint8_t PIN_QUAD_BOTTOM = 15;   // Analog A1
constexpr uint8_t PIN_QUAD_LEFT   = 16;   // Analog A2
constexpr uint8_t PIN_QUAD_RIGHT  = 17;   // Analog A3

// Irradiance sensor analog input
constexpr uint8_t PIN_IRRADIANCE = 20;    // Analog A6

// Wind speed sensor analog input
constexpr uint8_t PIN_WIND = 21;          // Analog A7

// Twilight / ambient light sensor analog input
constexpr uint8_t PIN_TWILIGHT = 22;      // Analog A8

// Pin motor H-bridge direction + enable pins (4 motors)
constexpr uint8_t PIN_MOTOR_DIR_A_0 = 3;  // Motor 0 direction A
constexpr uint8_t PIN_MOTOR_DIR_B_0 = 4;  // Motor 0 direction B
constexpr uint8_t PIN_MOTOR_EN_0    = 5;  // Motor 0 enable / PWM speed
constexpr uint8_t PIN_MOTOR_DIR_A_1 = 6;  // Motor 1 direction A
constexpr uint8_t PIN_MOTOR_DIR_B_1 = 7;  // Motor 1 direction B
constexpr uint8_t PIN_MOTOR_EN_1    = 8;  // Motor 1 enable / PWM speed
constexpr uint8_t PIN_MOTOR_DIR_A_2 = 9;  // Motor 2 direction A
constexpr uint8_t PIN_MOTOR_DIR_B_2 = 10; // Motor 2 direction B
constexpr uint8_t PIN_MOTOR_EN_2    = 11; // Motor 2 enable / PWM speed
constexpr uint8_t PIN_MOTOR_DIR_A_3 = 12; // Motor 3 direction A
constexpr uint8_t PIN_MOTOR_DIR_B_3 = 24; // Motor 3 direction B
constexpr uint8_t PIN_MOTOR_EN_3    = 25; // Motor 3 enable / PWM speed

// ============================================================================
// PWM / ESC Configuration
// ============================================================================

constexpr uint16_t PWM_PULSE_MIN_US  = 1000;  // Full reverse / minimum pulse
constexpr uint16_t PWM_PULSE_MAX_US  = 2000;  // Full forward / maximum pulse
constexpr uint16_t PWM_PULSE_NEUTRAL = 1500;  // Neutral / center position
constexpr uint16_t PWM_FREQUENCY_HZ  = 50;    // Standard servo frequency

// ============================================================================
// AS5600 Encoder
// ============================================================================

constexpr uint8_t AS5600_I2C_ADDRESS = 0x36;  // Default AS5600 I2C address

// ============================================================================
// Sensor Thresholds
// ============================================================================

// Minimum irradiance to engage solar tracking (W/m²)
constexpr float IRRADIANCE_THRESHOLD_WM2 = 200.0f;

// Maximum wind speed before stowing wing (m/s)
constexpr float WIND_MAX_MS = 15.0f;

// Quad sensor balance threshold — below this, sensor is "balanced" (ADC counts)
constexpr uint16_t SENSOR_BALANCE_THRESHOLD = 5;

// ============================================================================
// ADC Calibration Factors
// ============================================================================

// Teensy 4.0 ADC: 12-bit → 0–4095 counts, 3.3 V reference
constexpr uint16_t ADC_RESOLUTION_BITS = 12;
constexpr uint16_t ADC_MAX_COUNTS = 4095;
constexpr float    ADC_REF_VOLTAGE = 3.3f;

// Irradiance sensor: linear calibration (counts → W/m²)
// Assumes sensor outputs 0–3.3 V for 0–1500 W/m²
constexpr float IRRADIANCE_CAL_FACTOR = 1500.0f / ADC_MAX_COUNTS;

// Wind speed sensor: linear calibration (counts → m/s)
// Assumes sensor outputs 0–3.3 V for 0–50 m/s
constexpr float WIND_CAL_FACTOR = 50.0f / ADC_MAX_COUNTS;

// Twilight normalization: raw ADC → 0.0–1.0
constexpr float TWILIGHT_CAL_FACTOR = 1.0f / ADC_MAX_COUNTS;

// ============================================================================
// Location Stub (for solar model)
// ============================================================================

constexpr float DEFAULT_LATITUDE  = 48.8566f;   // Paris, France (placeholder)
constexpr float DEFAULT_LONGITUDE = 2.3522f;     // Paris, France (placeholder)

// ============================================================================
// Loop Timing
// ============================================================================

constexpr uint32_t MAIN_LOOP_INTERVAL_MS   = 50;   // 20 Hz main control loop
constexpr uint32_t SERIAL_PRINT_INTERVAL_MS = 500;  // 2 Hz serial output

// ============================================================================
// Solar Radiation Model Parameters
// ============================================================================

// Tilt angle search bounds (degrees)
constexpr float TILT_BETA_MIN_DEG = 0.0f;
constexpr float TILT_BETA_MAX_DEG = 90.0f;

// Ground reflectivity (albedo), dimensionless
constexpr float GROUND_REFLECTIVITY = 0.2f;

// Default clearness index K_T (0.0–1.0)
constexpr float DEFAULT_CLEARNESS_INDEX = 0.5f;

// ============================================================================
// Seasonal Diffuse Model Parameters (Frydrychowicz-Jastrzębska & Bugała)
// ============================================================================

// October–January: Bugler anisotropic diffuse model (p=0, q=1)
constexpr float SEASONAL_P_WINTER = 0.0f;
constexpr float SEASONAL_Q_WINTER = 1.0f;

// February–September: isotropic diffuse model (p=1, q=0)
constexpr float SEASONAL_P_SUMMER = 1.0f;
constexpr float SEASONAL_Q_SUMMER = 0.0f;

// ============================================================================
// ESC Arming
// ============================================================================

constexpr uint32_t ESC_ARM_DURATION_MS = 2000;  // Hold low throttle for 2 s

// ============================================================================
// Loop Timing — Safety & Tracking
// ============================================================================

constexpr uint32_t SAFETY_INTERVAL_MS   = 1000;     // 1 Hz safety evaluation
constexpr uint32_t TRACKING_INTERVAL_MS = 900000;   // 15 min target computation

// ============================================================================
// Tilt PID Controller
// ============================================================================

constexpr float    TILT_PID_KP          = 8.0f;
constexpr float    TILT_PID_KI          = 0.5f;
constexpr float    TILT_PID_KD          = 1.2f;
constexpr float    TILT_PID_KAW         = 0.1f;     // Anti-windup gain
constexpr uint16_t TILT_PID_OUT_MIN     = 1000;     // = PWM_PULSE_MIN_US
constexpr uint16_t TILT_PID_OUT_MAX     = 2000;     // = PWM_PULSE_MAX_US
constexpr float    TILT_PID_DEADBAND_DEG = 0.5f;

// ============================================================================
// Hybrid Tracker
// ============================================================================

constexpr float    SENSOR_TRACK_GAIN     = 0.02f;   // degrees per ADC count
constexpr uint32_t MODE_HOLD_DURATION_MS = 120000;  // 2 min hysteresis

// ============================================================================
// Safety Monitor
// ============================================================================

constexpr float TWILIGHT_THRESHOLD    = 0.1f;
constexpr float TILT_NIGHT_RESET_DEG  = 0.0f;
constexpr float TILT_WIND_SAFE_DEG    = 0.0f;

#endif // CONFIG_H
