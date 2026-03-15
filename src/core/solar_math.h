#ifndef SOLAR_MATH_H
#define SOLAR_MATH_H

// Pure C++ — no hardware dependencies
#include "solar_model.h"

/// Concrete solar position and tilt optimization model.
/// Implements Frydrychowicz-Jastrzębska & Bugała modified Liu-Jordan isotropic
/// radiation model with Orgill-Hollands diffuse correlation.
class SolarMath : public ISolarModel {
public:
    /// Compute solar position and optimum tilt for given input.
    SolarModelOutput compute(const SolarModelInput& input) override;

    /// Returns optimum panel tilt angle in degrees [0–90] that maximizes G_β.
    /// Returns 0° when sun is below horizon.
    float optimumTiltAngle(const SolarModelInput& input);

    /// Returns solar altitude angle in degrees above the horizon.
    float solarAltitudeDeg(const SolarModelInput& input);

    /// Returns solar azimuth angle in degrees measured clockwise from north.
    float solarAzimuthDeg(const SolarModelInput& input);

private:
    // ---- Solar Geometry ----

    /// Day-of-year number N from calendar date (1 = Jan 1).
    static int dayNumber(unsigned year, unsigned month, unsigned day);

    /// Solar declination δ(N) in radians.
    static float solarDeclinationRad(int N);

    /// Equation-of-time intermediate B(N) in degrees.
    static float equationOfTimeBDeg(int N);

    /// Equation of time in minutes, from B in degrees.
    static float equationOfTimeMinutes(float B_deg);

    /// Hour angle ω in radians.  Inputs: UTC time, longitude (°E), B (°).
    static float hourAngleRad(unsigned hour, unsigned minute, unsigned second,
                              float longitude_deg, float B_deg);

    /// Solar altitude angle α in radians (above horizon).
    static float solarAltitudeRad(float latitude_rad, float declination_rad,
                                  float hour_angle_rad);

    /// Solar azimuth angle γ in radians from north, clockwise.
    static float solarAzimuthRad(float latitude_rad, float declination_rad,
                                 float hour_angle_rad, float altitude_rad);

    /// Zenith angle θ_z = π/2 − α, in radians.
    static float zenithAngleRad(float altitude_rad);

    // ---- Radiation Model ----

    /// Extraterrestrial radiation G_on (W/m²) for day N.
    static float extraterrestrialRadiation(int N);

    /// Diffuse fraction G_d / G_t from the Orgill-Hollands correlation.
    static float diffuseFraction(float K_T);

    /// Angle of incidence θ_i on a south-facing tilted surface, in radians.
    static float angleOfIncidenceRad(float latitude_rad, float declination_rad,
                                     float hour_angle_rad, float beta_rad);

    /// Geometric factor R_b = cos(θ_i) / cos(θ_z).
    static float geometricFactorRb(float theta_i_rad, float theta_z_rad);

    /// Diffuse correction factor R_d using seasonal p/q and Bugler coefficients.
    static float diffuseCorrectionRd(float beta_rad, unsigned month);

    /// Total radiation on tilted plane G_β (W/m²).
    static float totalTiltedRadiation(float G_b, float G_d, float R_b,
                                      float R_d, float beta_rad);
};

#endif // SOLAR_MATH_H
