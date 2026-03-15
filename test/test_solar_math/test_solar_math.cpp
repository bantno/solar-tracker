// Unity test for SolarMath
// Run: pio test -e native -f test_solar_math

#include <unity.h>
#include "core/solar_math.h"
#include <cmath>
#include <cstdio>

void setUp(void) {}
void tearDown(void) {}

// Reference location: Poznan, Poland
static constexpr float LAT = 52.4f;    // deg N
static constexpr float LON = 16.93f;   // deg E

static SolarModelInput makeInput(unsigned year, unsigned month, unsigned day,
                                 unsigned hour, unsigned minute = 0,
                                 unsigned second = 0) {
    SolarModelInput in{};
    in.latitude  = LAT;
    in.longitude = LON;
    in.year   = year;
    in.month  = month;
    in.day    = day;
    in.hour   = hour;
    in.minute = minute;
    in.second = second;
    return in;
}

// 1. Day number / equinox altitude
void test_equinox_noon_altitude() {
    SolarMath model;
    SolarModelInput equinox = makeInput(2024, 3, 21, 12);
    float alt_equinox = model.solarAltitudeDeg(equinox);
    // At equinox at noon, altitude approx 90 - |lat - decl| approx 90 - 52.4 approx 37.6
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 37.6f, alt_equinox);
}

// 2. Solar declination via summer solstice altitude
void test_summer_solstice_altitude() {
    SolarMath model;
    SolarModelInput summer = makeInput(2024, 6, 21, 12);
    float alt_summer = model.solarAltitudeDeg(summer);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 59.0f, alt_summer);
}

// 3. Winter solstice altitude
void test_winter_solstice_altitude() {
    SolarMath model;
    SolarModelInput winter = makeInput(2024, 12, 21, 12);
    float alt_winter = model.solarAltitudeDeg(winter);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 14.15f, alt_winter);
}

// 4. Solar altitude (redundant with above, kept for parity)
void test_solar_altitude_summer() {
    SolarMath model;
    SolarModelInput summer_noon = makeInput(2024, 6, 21, 12);
    float alt = model.solarAltitudeDeg(summer_noon);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 59.0f, alt);
}

// 5. Optimum tilt summer
void test_optimum_tilt_summer() {
    SolarMath model;
    SolarModelInput summer = makeInput(2024, 6, 21, 12);
    float tilt_summer = model.optimumTiltAngle(summer);
    TEST_ASSERT_TRUE_MESSAGE(tilt_summer >= 15.0f && tilt_summer <= 40.0f,
        "Summer optimum tilt should be in [15, 40]");
    printf("  INFO: summer optimum tilt=%.2f\n", tilt_summer);
}

// 6. Optimum tilt winter
void test_optimum_tilt_winter() {
    SolarMath model;
    SolarModelInput winter = makeInput(2024, 12, 21, 12);
    float tilt_winter = model.optimumTiltAngle(winter);
    TEST_ASSERT_TRUE_MESSAGE(tilt_winter >= 55.0f && tilt_winter <= 85.0f,
        "Winter optimum tilt should be in [55, 85]");
    printf("  INFO: winter optimum tilt=%.2f\n", tilt_winter);
}

// 7. G_beta sanity: altitude > 0 on summer noon
void test_gbeta_sanity_altitude_positive() {
    SolarMath model;
    SolarModelInput summer = makeInput(2024, 6, 21, 12);
    float alt = model.solarAltitudeDeg(summer);
    TEST_ASSERT_TRUE_MESSAGE(alt > 0.0f, "Summer noon: altitude > 0 (sun is up)");
}

// 8. G_beta sanity: optimum tilt > 0 implies G_beta was computed
void test_gbeta_sanity_tilt_positive() {
    SolarMath model;
    SolarModelInput summer = makeInput(2024, 6, 21, 12);
    float tilt = model.optimumTiltAngle(summer);
    TEST_ASSERT_TRUE_MESSAGE(tilt > 0.0f,
        "Summer noon: optimum tilt > 0 (G_beta > 0)");
}

// 9. Night / below-horizon guard: winter midnight
void test_night_guard_winter_midnight() {
    SolarMath model;
    SolarModelInput midnight = makeInput(2024, 12, 21, 0);
    float tilt = model.optimumTiltAngle(midnight);
    TEST_ASSERT_TRUE(tilt >= 0.0f && tilt <= 90.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, tilt);
    TEST_ASSERT_FALSE_MESSAGE(std::isnan(tilt), "Night tilt is not NaN");
    TEST_ASSERT_FALSE_MESSAGE(std::isinf(tilt), "Night tilt is not Inf");
}

// 10. Night guard: summer midnight -- no NaN/Inf
void test_night_guard_summer_midnight() {
    SolarMath model;
    SolarModelInput summer_midnight = makeInput(2024, 6, 21, 0);
    float tilt2 = model.optimumTiltAngle(summer_midnight);
    TEST_ASSERT_FALSE_MESSAGE(std::isnan(tilt2), "Summer midnight tilt is not NaN");
    TEST_ASSERT_FALSE_MESSAGE(std::isinf(tilt2), "Summer midnight tilt is not Inf");
    TEST_ASSERT_TRUE(tilt2 >= 0.0f && tilt2 <= 90.0f);
}

// 11. Monthly tilt table (informational -- always passes)
void test_monthly_tilt_table() {
    SolarMath model;
    printf("\n--- Monthly Optimum Tilt at Solar Noon (52.4N, 16.93E) ---\n");
    printf("%-10s  %12s  %12s  %12s\n",
           "Month", "Altitude(deg)", "Azimuth(deg)", "Tilt(deg)");
    printf("------------------------------------------------------\n");
    for (unsigned m = 1; m <= 12; m++) {
        SolarModelInput in = makeInput(2024, m, 21, 11);
        float alt = model.solarAltitudeDeg(in);
        float az  = model.solarAzimuthDeg(in);
        float tilt = model.optimumTiltAngle(in);
        printf("  %2u/21      %8.2f      %8.2f      %8.2f\n",
               m, alt, az, tilt);
    }
    printf("\n");
    TEST_ASSERT_TRUE_MESSAGE(true, "Monthly table printed");
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_equinox_noon_altitude);
    RUN_TEST(test_summer_solstice_altitude);
    RUN_TEST(test_winter_solstice_altitude);
    RUN_TEST(test_solar_altitude_summer);
    RUN_TEST(test_optimum_tilt_summer);
    RUN_TEST(test_optimum_tilt_winter);
    RUN_TEST(test_gbeta_sanity_altitude_positive);
    RUN_TEST(test_gbeta_sanity_tilt_positive);
    RUN_TEST(test_night_guard_winter_midnight);
    RUN_TEST(test_night_guard_summer_midnight);
    RUN_TEST(test_monthly_tilt_table);
    return UNITY_END();
}
