@echo off
title Calibrate 4-Microphone INMP441 Acoustic Array (Ground Truth)
color 0D
echo =====================================================================
echo    PHYSICAL 4-MICROPHONE ACOUSTIC LOCALIZATION CALIBRATOR
echo    Tests Array Accuracy Across Ground Truth Angles: 0, ±30, ±45 deg
echo =====================================================================
echo.
echo Starting acoustic localization benchmark...
echo.

cd /d "%~dp0"
set PYTHONUNBUFFERED=1
set PYTHON_BIN=C:\Python314\python.exe
if not exist "%PYTHON_BIN%" set PYTHON_BIN=python

"%PYTHON_BIN%" tools\calibrate_physical_array.py --angles 0 30 -30 45 -45 --port COM8
if errorlevel 1 (
    echo.
    echo [ERROR] Calibration exited with error code %ERRORLEVEL%
    pause
) else (
    echo.
    echo =====================================================================
    echo [SUCCESS] Physical Acoustic Benchmark Complete!
    echo   - Plot: docs\evidence\physical_acoustic_localization_benchmark.png
    echo   - JSON: stage2_acoustic_localization\evidence\physical_acoustic_calibration.json
    echo =====================================================================
    echo.
    pause
)
