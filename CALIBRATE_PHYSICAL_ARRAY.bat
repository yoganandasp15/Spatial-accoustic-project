@echo off
@chcp 65001 >nul
title Physical 4-Mic Array Ground Truth Acoustic Calibration
color 0B
cls
echo ================================================================================
echo          PHYSICAL 4-MIC ARRAY GROUND TRUTH ACOUSTIC CALIBRATION
echo          Measuring SRP-PHAT Angular Precision Across 0 deg, +-30 deg, +-45 deg
echo ================================================================================
echo.
echo [INSTRUCTION] Place acoustic test tone/speaker at marked angles relative to array.
echo               Close any open radar GUI (COM8 will be used exclusively).
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\calibrate_physical_array.py --port COM8 --baud 921600
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Calibration aborted or failed with exit code %ERRORLEVEL%.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Physical array calibration verified and saved!
    echo ================================================================================
    echo.
    pause
)