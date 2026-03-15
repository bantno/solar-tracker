#!/usr/bin/env pwsh
# Run all desktop tests for the solar-tracker project
# Usage: powershell -ExecutionPolicy Bypass -File test/run_all_tests.ps1

$ErrorActionPreference = "Stop"
$project = Split-Path -Parent (Split-Path -Parent $PSCommandPath)
Set-Location $project

$tests = @(
    @{
        name    = "test_pid"
        sources = "test/test_tilt_controller.cpp src/core/tilt_controller.cpp"
        exe     = "test_pid.exe"
    },
    @{
        name    = "test_tracker"
        sources = "test/test_hybrid_tracker.cpp src/core/hybrid_tracker.cpp src/core/solar_math.cpp"
        exe     = "test_tracker.exe"
    },
    @{
        name    = "test_integration"
        sources = "test/test_integration.cpp src/core/tilt_controller.cpp src/core/hybrid_tracker.cpp src/core/safety_monitor.cpp src/core/solar_math.cpp"
        exe     = "test_integration.exe"
    },
    @{
        name    = "test_edge_cases"
        sources = "test/test_edge_cases.cpp src/core/tilt_controller.cpp"
        exe     = "test_edge_cases.exe"
    },
    @{
        name    = "test_safety_scenarios"
        sources = "test/test_safety_scenarios.cpp src/core/safety_monitor.cpp"
        exe     = "test_safety_scenarios.exe"
    }
)

$total_pass = 0
$total_fail = 0
$failed_suites = @()

foreach ($t in $tests) {
    Write-Host "`n============================================================" -ForegroundColor Cyan
    Write-Host "Building $($t.name)..." -ForegroundColor Cyan
    Write-Host "============================================================" -ForegroundColor Cyan

    $cmd = "g++ -std=c++17 -I src $($t.sources) -o $($t.exe) 2>&1"
    $build_output = Invoke-Expression $cmd
    if ($LASTEXITCODE -ne 0) {
        Write-Host "BUILD FAILED: $($t.name)" -ForegroundColor Red
        Write-Host $build_output
        $total_fail++
        $failed_suites += $t.name
        continue
    }

    Write-Host "Running $($t.name)..." -ForegroundColor Green
    $run_output = & ".\$($t.exe)" 2>&1
    $exit_code = $LASTEXITCODE

    Write-Host ($run_output -join "`n")

    if ($exit_code -ne 0) {
        $total_fail++
        $failed_suites += $t.name
    } else {
        $total_pass++
    }
}

Write-Host "`n============================================================" -ForegroundColor Cyan
Write-Host "SUMMARY: $total_pass suites passed, $total_fail suites failed" -ForegroundColor $(if ($total_fail -gt 0) { "Red" } else { "Green" })
if ($failed_suites.Count -gt 0) {
    Write-Host "Failed: $($failed_suites -join ', ')" -ForegroundColor Red
}
Write-Host "============================================================" -ForegroundColor Cyan

exit $total_fail
