@echo off
@chcp 65001 >nul
title Record Healthy Motor Physical Experiment (30 Seconds)
color 0B
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL MOTOR RECORDER: HEALTHY MOTOR RUN (30s)
echo          400Hz ADXL345 Vibration + FC-03 Tachometer + 4-Mic INMP441 Audio
echo ================================================================================
echo.
echo [INSTRUCTION] Ensure healthy DC motor is securely mounted, powered, and spinning.
echo               Close any open radar GUI (COM8 will be used exclusively).
echo               Duration: 30.0s (Captures ~12,000 vibration and ~300,000 audio samples).
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\record_motor_experiment.py --motor-id motor_A --health-label healthy --environment quiet --fault-type none --duration 30.0 --port COM8 %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Acquisition aborted or failed with exit code %ERRORLEVEL%.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Healthy motor run recorded to data\dataset\ and registered in manifest!
    echo ================================================================================
    echo.
    pause
)
