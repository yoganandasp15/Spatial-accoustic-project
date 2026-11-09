@echo off
@chcp 65001 >nul
title Spatial Multimodal Edge-AI Minor Project Master Suite
color 0B

:MENU
cls
echo ╔══════════════════════════════════════════════════════════════════════════════╗
echo ║    SPATIAL MULTIMODAL EDGE-AI FOR ROTATING MACHINERY (MINOR PROJECT SUITE)   ║
echo ║    4-Mic ULA Array + ADXL345 Vibration + FC-03 Tachometer + Multimodal Fusion║
echo ╚══════════════════════════════════════════════════════════════════════════════╝
echo.
echo    [1] Launch Live Real-Time Radar GUI (Acoustic Camera + Motor Diagnostics)
echo    [2] Record HEALTHY Motor Run (10s @ 400Hz Vibration + Tach + Audio)
echo    [3] Record DEFECTIVE Motor Run (10s @ 400Hz Vibration + Tach + Audio)
echo    [4] Record ACOUSTIC DISTURBANCE Test (Proves Acoustic Noise != Motor Fault)
echo    [5] Compare Motor Runs & Generate Final Academic Report + Figures
echo    [6] Calibrate 4-Mic Array Accuracy (0°, ±30°, ±45° Ground Truth)
echo    [7] Run Complete Automated Test Suite (55 Tests Across All 4 Stages)
echo    [8] Exit Suite
echo.
echo ────────────────────────────────────────────────────────────────────────────────
set /p M="Select an option (1-8): "

if "%M%"=="1" (
    call Launch_Spatial_Acoustic_Radar.bat
    goto MENU
)
if "%M%"=="2" (
    call RECORD_HEALTHY_MOTOR.bat
    goto MENU
)
if "%M%"=="3" (
    call RECORD_DEFECTIVE_MOTOR.bat
    goto MENU
)
if "%M%"=="4" (
    call RECORD_ACOUSTIC_DISTURBANCE.bat
    goto MENU
)
if "%M%"=="5" (
    call GENERATE_EXPERIMENT_REPORT.bat
    goto MENU
)
if "%M%"=="6" (
    call CALIBRATE_PHYSICAL_ARRAY.bat
    goto MENU
)
if "%M%"=="7" (
    call RUN_UNIT_TESTS.bat
    goto MENU
)
if "%M%"=="8" exit /b 0
goto MENU
