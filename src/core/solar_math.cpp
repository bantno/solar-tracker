// Pure C++ — compile-time guard against accidental Arduino inclusion
// In PlatformIO builds, ARDUINO is defined globally but these files must not
// depend on Arduino.h. The guard fires only outside PlatformIO (e.g. Arduino IDE).
#if defined(ARDUINO) && !defined(PLATFORMIO_BUILD)
#error "solar_math.cpp must not include Arduino.h or any hardware headers"
#endif

#include "solar_math.h"
#include "../config/config.h"
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Physical constants
// ============================================================================

/// Solar constant (W/m²)
static constexpr float G_SC = 1367.0f;

/// Degrees-to-radians conversion factor
static constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;

/// Radians-to-degrees conversion factor
static constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

/// Bugler coefficients for anisotropic diffuse model
static constexpr float BUGLER_A = 0.51f;
static constexpr float BUGLER_B = -0.18f;
static constexpr float BUGLER_C = 0.145f;

/// Tilt search step size (degrees)
static constexpr float TILT_STEP_DEG = 0.5f;

// ============================================================================
// Solar Geometry
// ============================================================================

/// Day-of-year number N from calendar date (1 = Jan 1).
int SolarMath::dayNumber(unsigned year, unsigned month, unsigned day) {
    static constexpr int cumDays[] = {
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
    };
    int N = cumDays[month - 1] + static_cast<int>(day);

    // Leap year adjustment: add 1 if past February
    bool isLeap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    if (isLeap && month > 2) {
        N += 1;
    }
    return N;
}

/// Solar declination δ(N) in radians.
/// δ = 23.45° · sin(360/365 · (N − 81))
float SolarMath::solarDeclinationRad(int N) {
    float argument_deg = (360.0f / 365.0f) * static_cast<float>(N - 81);
    float argument_rad = argument_deg * DEG2RAD;
    float declination_deg = 23.45f * std::sin(argument_rad);
    float declination_rad = declination_deg * DEG2RAD;
    return declination_rad;
}

/// Equation-of-time intermediate B(N) in degrees.
/// B = (360/364) · (N − 81)
float SolarMath::equationOfTimeBDeg(int N) {
    float B_deg = (360.0f / 364.0f) * static_cast<float>(N - 81);
    return B_deg;
}

/// Equation of time in minutes.
/// EoT = 9.87·sin(2B) − 7.53·cos(B) − 1.5·sin(B)
float SolarMath::equationOfTimeMinutes(float B_deg) {
    float B_rad = B_deg * DEG2RAD;
    float sin_B = std::sin(B_rad);
    float cos_B = std::cos(B_rad);
    float sin_2B = std::sin(2.0f * B_rad);
    float EoT_minutes = 9.87f * sin_2B - 7.53f * cos_B - 1.5f * sin_B;
    return EoT_minutes;
}

/// Hour angle ω in radians from UTC time, longitude, and equation of time.
/// Solar time = UTC + longitude/15 + EoT/60  (in hours)
/// ω = 15° · (solar_time − 12)
float SolarMath::hourAngleRad(unsigned hour, unsigned minute, unsigned second,
                              float longitude_deg, float B_deg) {
    float utc_hours = static_cast<float>(hour)
                    + static_cast<float>(minute) / 60.0f
                    + static_cast<float>(second) / 3600.0f;
    float EoT_minutes = equationOfTimeMinutes(B_deg);
    float solar_time_hours = utc_hours + longitude_deg / 15.0f + EoT_minutes / 60.0f;
    float omega_deg = 15.0f * (solar_time_hours - 12.0f);
    float omega_rad = omega_deg * DEG2RAD;
    return omega_rad;
}

/// Solar altitude angle α in radians (above horizon).
/// sin(α) = sin(φ)·sin(δ) + cos(φ)·cos(δ)·cos(ω)
float SolarMath::solarAltitudeRad(float latitude_rad, float declination_rad,
                                  float hour_angle_rad) {
    float sin_lat = std::sin(latitude_rad);
    float cos_lat = std::cos(latitude_rad);
    float sin_decl = std::sin(declination_rad);
    float cos_decl = std::cos(declination_rad);
    float cos_omega = std::cos(hour_angle_rad);
    float sin_alpha = sin_lat * sin_decl + cos_lat * cos_decl * cos_omega;
    // Clamp to [-1, 1] to guard against floating-point drift
    sin_alpha = std::fmax(-1.0f, std::fmin(1.0f, sin_alpha));
    float alpha_rad = std::asin(sin_alpha);
    return alpha_rad;
}

/// Solar azimuth angle γ in radians from north, clockwise.
/// Computed via atan2 of azimuth-from-south components, then rotated +π.
float SolarMath::solarAzimuthRad(float latitude_rad, float declination_rad,
                                 float hour_angle_rad, float altitude_rad) {
    float sin_decl = std::sin(declination_rad);
    float cos_decl = std::cos(declination_rad);
    float sin_lat = std::sin(latitude_rad);
    float cos_lat = std::cos(latitude_rad);
    float cos_alpha = std::cos(altitude_rad);
    float sin_omega = std::sin(hour_angle_rad);

    // Azimuth from south: sin and cos components
    float sin_gamma_s = -cos_decl * sin_omega / cos_alpha;
    float cos_gamma_s = (sin_decl * cos_lat - cos_decl * sin_lat * std::cos(hour_angle_rad))
                        / cos_alpha;

    // Azimuth from south (positive westward)
    float gamma_south_rad = std::atan2(sin_gamma_s, cos_gamma_s);

    // Convert to from-north, clockwise
    float gamma_north_rad = gamma_south_rad + static_cast<float>(M_PI);

    // Normalize to [0, 2π)
    float two_pi = 2.0f * static_cast<float>(M_PI);
    gamma_north_rad = std::fmod(gamma_north_rad, two_pi);
    if (gamma_north_rad < 0.0f) {
        gamma_north_rad += two_pi;
    }
    return gamma_north_rad;
}

/// Zenith angle θ_z = π/2 − α
float SolarMath::zenithAngleRad(float altitude_rad) {
    return static_cast<float>(M_PI) / 2.0f - altitude_rad;
}

// ============================================================================
// Radiation Model
// ============================================================================

/// Extraterrestrial radiation G_on (W/m²).
/// G_on = G_sc · (1 + 0.033 · cos(360·N/365))
float SolarMath::extraterrestrialRadiation(int N) {
    float argument_deg = 360.0f * static_cast<float>(N) / 365.0f;
    float argument_rad = argument_deg * DEG2RAD;
    float G_on = G_SC * (1.0f + 0.033f * std::cos(argument_rad));
    return G_on;
}

/// Diffuse fraction G_d/G_t from the Orgill-Hollands correlation.
float SolarMath::diffuseFraction(float K_T) {
    if (K_T <= 0.35f) {
        return 1.0f - 0.249f * K_T;
    } else if (K_T < 0.75f) {
        return 1.557f - 1.84f * K_T;
    } else {
        return 0.177f;
    }
}

/// Angle of incidence θ_i on a south-facing tilted surface, in radians.
/// cos(θ_i) = sin(δ)·sin(φ − β) + cos(δ)·cos(φ − β)·cos(ω)
float SolarMath::angleOfIncidenceRad(float latitude_rad, float declination_rad,
                                     float hour_angle_rad, float beta_rad) {
    float lat_minus_beta = latitude_rad - beta_rad;
    float sin_decl = std::sin(declination_rad);
    float cos_decl = std::cos(declination_rad);
    float sin_lmb = std::sin(lat_minus_beta);
    float cos_lmb = std::cos(lat_minus_beta);
    float cos_omega = std::cos(hour_angle_rad);
    float cos_theta_i = sin_decl * sin_lmb + cos_decl * cos_lmb * cos_omega;
    cos_theta_i = std::fmax(-1.0f, std::fmin(1.0f, cos_theta_i));
    float theta_i_rad = std::acos(cos_theta_i);
    return theta_i_rad;
}

/// Geometric factor R_b = cos(θ_i) / cos(θ_z).
/// Returns 0 if sun is near horizon or beam is behind panel.
float SolarMath::geometricFactorRb(float theta_i_rad, float theta_z_rad) {
    float cos_theta_z = std::cos(theta_z_rad);
    float cos_theta_i = std::cos(theta_i_rad);

    // Guard: sun too close to horizon or beam behind panel
    if (cos_theta_z < 0.01f || cos_theta_i < 0.0f) {
        return 0.0f;
    }
    return cos_theta_i / cos_theta_z;
}

/// Diffuse correction factor R_d using seasonal p/q parameters and Bugler
/// coefficients. October–January uses winter (Bugler anisotropic) model;
/// February–September uses summer (isotropic) model.
float SolarMath::diffuseCorrectionRd(float beta_rad, unsigned month) {
    float p, q;
    if (month >= 10 || month <= 1) {
        // October–January (winter)
        p = SEASONAL_P_WINTER;
        q = SEASONAL_Q_WINTER;
    } else {
        // February–September (summer)
        p = SEASONAL_P_SUMMER;
        q = SEASONAL_Q_SUMMER;
    }

    float cos_beta = std::cos(beta_rad);
    float isotropic_term = (1.0f + cos_beta) / 2.0f;
    float bugler_term = BUGLER_A + BUGLER_B * cos_beta + BUGLER_C * cos_beta * cos_beta;

    float R_d = p * isotropic_term + q * bugler_term;
    return R_d;
}

/// Total radiation on tilted plane G_β (W/m²).
/// G_β = G_b·R_b + G_d·R_d + (G_b + G_d)·ρ_o·(1 − cos β)/2
float SolarMath::totalTiltedRadiation(float G_b, float G_d, float R_b,
                                      float R_d, float beta_rad) {
    float cos_beta = std::cos(beta_rad);
    float beam_component = G_b * R_b;
    float diffuse_component = G_d * R_d;
    float ground_reflected = (G_b + G_d) * GROUND_REFLECTIVITY * (1.0f - cos_beta) / 2.0f;
    float G_beta = beam_component + diffuse_component + ground_reflected;
    return G_beta;
}

// ============================================================================
// Public Interface
// ============================================================================

float SolarMath::solarAltitudeDeg(const SolarModelInput& input) {
    int N = dayNumber(input.year, input.month, input.day);
    float declination_rad = solarDeclinationRad(N);
    float B_deg = equationOfTimeBDeg(N);
    float latitude_rad = input.latitude * DEG2RAD;
    float omega_rad = hourAngleRad(input.hour, input.minute, input.second,
                                   input.longitude, B_deg);
    float alpha_rad = solarAltitudeRad(latitude_rad, declination_rad, omega_rad);
    return alpha_rad * RAD2DEG;
}

float SolarMath::solarAzimuthDeg(const SolarModelInput& input) {
    int N = dayNumber(input.year, input.month, input.day);
    float declination_rad = solarDeclinationRad(N);
    float B_deg = equationOfTimeBDeg(N);
    float latitude_rad = input.latitude * DEG2RAD;
    float omega_rad = hourAngleRad(input.hour, input.minute, input.second,
                                   input.longitude, B_deg);
    float alpha_rad = solarAltitudeRad(latitude_rad, declination_rad, omega_rad);

    // Guard: if sun is at or below horizon, azimuth is undefined; return 0
    if (alpha_rad <= 0.0f) {
        return 0.0f;
    }

    float gamma_rad = solarAzimuthRad(latitude_rad, declination_rad,
                                      omega_rad, alpha_rad);
    return gamma_rad * RAD2DEG;
}

float SolarMath::optimumTiltAngle(const SolarModelInput& input) {
    int N = dayNumber(input.year, input.month, input.day);
    float declination_rad = solarDeclinationRad(N);
    float B_deg = equationOfTimeBDeg(N);
    float latitude_rad = input.latitude * DEG2RAD;
    float omega_rad = hourAngleRad(input.hour, input.minute, input.second,
                                   input.longitude, B_deg);
    float alpha_rad = solarAltitudeRad(latitude_rad, declination_rad, omega_rad);

    // Sun below horizon — return flat position
    if (alpha_rad <= 0.0f) {
        return 0.0f;
    }

    float theta_z_rad = zenithAngleRad(alpha_rad);

    // Compute horizontal irradiance components
    float G_on = extraterrestrialRadiation(N);
    float sin_alpha = std::sin(alpha_rad);
    float G_0_horizontal = G_on * sin_alpha;
    float G_t = DEFAULT_CLEARNESS_INDEX * G_0_horizontal;
    float diffuse_frac = diffuseFraction(DEFAULT_CLEARNESS_INDEX);
    float G_d = diffuse_frac * G_t;
    float G_b = G_t - G_d;

    // Sweep tilt angles to find optimum
    float best_beta_deg = TILT_BETA_MIN_DEG;
    float best_G_beta = -1.0f;

    for (float beta_deg = TILT_BETA_MIN_DEG; beta_deg <= TILT_BETA_MAX_DEG;
         beta_deg += TILT_STEP_DEG) {
        float beta_rad = beta_deg * DEG2RAD;
        float theta_i_rad = angleOfIncidenceRad(latitude_rad, declination_rad,
                                                omega_rad, beta_rad);
        float R_b = geometricFactorRb(theta_i_rad, theta_z_rad);
        float R_d = diffuseCorrectionRd(beta_rad, input.month);
        float G_beta = totalTiltedRadiation(G_b, G_d, R_b, R_d, beta_rad);

        if (G_beta > best_G_beta) {
            best_G_beta = G_beta;
            best_beta_deg = beta_deg;
        }
    }

    return best_beta_deg;
}

SolarModelOutput SolarMath::compute(const SolarModelInput& input) {
    SolarModelOutput out{};
    out.optimal_tilt_deg = optimumTiltAngle(input);
    out.solar_elevation_deg = solarAltitudeDeg(input);
    out.solar_azimuth_deg = solarAzimuthDeg(input);
    return out;
}
