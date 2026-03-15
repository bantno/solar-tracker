# Desktop Test Suite Summary

## Running Tests

```
pio test -e native
```

All tests use the Unity framework and run in PlatformIO's `[env:native]` environment.
The `[env:teensy40]` environment skips tests (`test_ignore = *`) since it requires
hardware.

Standalone (non-Unity) versions of the original tests are preserved in
`test/_standalone/` for manual `g++` compilation if needed.

## Test Suites

**6 suites, 48 test cases total**

---

### test_tilt_controller (7 tests)

PID position controller: step response, deadband, anti-windup, reset, direction.

| Test | Verifies |
|---|---|
| `test_step_response` | Converges from 0 to 45 deg within 200 ticks, output always in [1000, 2000] us |
| `test_deadband` | Output exactly 1500 us when error < 0.5 deg |
| `test_anti_windup` | Integral stays bounded under sustained saturation |
| `test_reset` | Integral and derivative zeroed after reset, no derivative kick on first tick |
| `test_direction` | Positive error produces output > 1500, negative produces < 1500 |
| `test_move_complete` | `isSettled()` false on first tick of 45 deg step, true when converged |
| `test_simulation_trace` | Prints 60-tick trace from 20 to 35 deg (informational) |

---

### test_hybrid_tracker (6 tests)

Mode switching, sensor vs astronomical targeting, hysteresis, clamping.

| Test | Verifies |
|---|---|
| `test_mode_selection` | SENSOR_BASED when irradiance > threshold, ASTRONOMICAL when below |
| `test_mode_hysteresis` | Mode does not switch before MODE_HOLD_DURATION_MS (2 min) elapses |
| `test_sensor_based_target` | Target = current + (quadTop - quadBottom) * SENSOR_TRACK_GAIN |
| `test_astronomical_target` | Target matches mock solar model output |
| `test_clamping` | Target clamped to [TILT_BETA_MIN_DEG, TILT_BETA_MAX_DEG] |
| `test_move_in_progress` | moveInProgress set when target differs > deadband, unchanged within |

---

### test_edge_cases (10 tests)

PID robustness under boundary and stress conditions.

| Test | Verifies |
|---|---|
| `test_negative_target` | Drives toward negative target (-5 deg), output in range |
| `test_zero_target` | Converges from 60 deg to 0 deg |
| `test_millis_overflow` | No NaN/Inf through uint32_t wraparound (~49.7 days) |
| `test_rapid_target_changes` | 5 target changes mid-move, output always in range with correct direction |
| `test_zero_dt` | Same timestamp twice — no crash or NaN |
| `test_large_dt` | 10-second gap between ticks — integral bounded (dt clamp to 100 ms) |
| `test_oscillating_measurement` | Noise within deadband — all 100 ticks return neutral (1500 us) |
| `test_extreme_angles` | 0 to 90 deg step — saturates initially, converges |
| `test_reset_mid_move_no_kick` | Reset mid-move then new target — derivative is zero (no kick) |
| `test_saturation_then_reversal` | Sustained saturation then direction reversal — clean recovery |

---

### test_safety_scenarios (8 tests)

SafetyMonitor state transitions and edge detection.

| Test | Verifies |
|---|---|
| `test_wind_spike_and_recovery` | Wind > threshold stows to safe angle, clears when wind drops |
| `test_day_night_cycle` | Twilight drop triggers night reset (edge-triggered), re-triggers on next nightfall |
| `test_flight_lock` | inFlight sets safetyFlightLock, clears on landing |
| `test_multiple_flags` | All 4 flags set simultaneously, cleared individually |
| `test_esc_arming_transition` | safetyEscNotArmed tracks ESC armed/disarmed state |
| `test_night_reset_edge_detection` | Night reset fires once per false-to-true edge, no re-trigger during sustained night |
| `test_wind_override_continuous` | Wind override re-applies target every tick while active |
| `test_boundary_values` | Exactly at threshold is safe, just above/below triggers |

---

### test_solar_math (11 tests)

Solar position and tilt optimization (Poznan, Poland — 52.4 N, 16.93 E).

| Test | Verifies |
|---|---|
| `test_equinox_noon_altitude` | Spring equinox noon altitude ~37.6 deg |
| `test_summer_solstice_altitude` | Summer solstice noon altitude ~59 deg |
| `test_winter_solstice_altitude` | Winter solstice noon altitude ~14.15 deg |
| `test_solar_altitude_summer` | Summer altitude cross-check |
| `test_optimum_tilt_summer` | Summer optimal tilt in [15, 40] deg |
| `test_optimum_tilt_winter` | Winter optimal tilt in [55, 85] deg |
| `test_gbeta_sanity_altitude_positive` | Sun altitude > 0 at summer noon |
| `test_gbeta_sanity_tilt_positive` | Optimal tilt > 0 when sun is up |
| `test_night_guard_winter_midnight` | Midnight tilt = 0, no NaN/Inf |
| `test_night_guard_summer_midnight` | Summer midnight tilt finite and in [0, 90] deg |
| `test_monthly_tilt_table` | Prints monthly altitude/azimuth/tilt table (informational) |

---

### test_integration (6 tests)

Full system integration — SimContext wires TiltController, HybridTracker,
SafetyMonitor, and MockSolarModel together in a simulated main loop.

| Test | Verifies |
|---|---|
| `test_astronomical_tracking_cycle` | 10 to 35 deg move under astronomical mode, settles in ~2.3s |
| `test_sensor_based_tracking` | High irradiance switches to SENSOR_BASED, target tracks quad sensor delta |
| `test_wind_interruption_mid_move` | Mid-move wind spike stows to safe angle |
| `test_mode_hysteresis_oscillation` | 10 cycles of irradiance oscillation — mode stays ASTRONOMICAL |
| `test_full_move_lifecycle` | 0 to 45 deg move settles and holds stable with no drift |
| `test_tracking_blocked_by_safety` | Wind override blocks tracking update, target forced to safe stow |

---

## Test Architecture

All core modules (`TiltController`, `HybridTracker`, `SafetyMonitor`, `SolarMath`)
have zero Arduino dependencies and compile on desktop with standard C++17.
Tests use a `MockSolarModel` implementing `ISolarModel` for deterministic control
of solar position output. The integration test's `SimContext` struct replicates the
`loop()` timing architecture (20 Hz main loop, 1 Hz safety, 15-min tracking) to
validate module interplay without hardware.
