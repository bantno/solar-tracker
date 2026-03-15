#ifndef SOLAR_MODEL_H
#define SOLAR_MODEL_H

// Pure C++ — no hardware dependencies, no Arduino.h
// This is the swappable seam: any solar position model implements ISolarModel.
// The hybrid_tracker (future) holds an ISolarModel* pointer, injected at startup.
// Candidate implementations: Liu-Jordan, SPA, Grena, Frydrychowicz-Jastrzębska, etc.

struct SolarModelInput {
    float    latitude;     // Decimal degrees, positive north
    float    longitude;    // Decimal degrees, positive east
    unsigned year;
    unsigned month;        // 1–12
    unsigned day;          // 1–31
    unsigned hour;         // 0–23 UTC
    unsigned minute;       // 0–59
    unsigned second;       // 0–59
};

struct SolarModelOutput {
    float optimal_tilt_deg;     // Recommended panel tilt angle (degrees)
    float solar_elevation_deg;  // Sun elevation above horizon (degrees)
    float solar_azimuth_deg;    // Sun azimuth from north (degrees)
};

// Abstract solar model interface
class ISolarModel {
public:
    virtual ~ISolarModel() = default;
    virtual SolarModelOutput compute(const SolarModelInput& input) = 0;
};

#endif // SOLAR_MODEL_H
