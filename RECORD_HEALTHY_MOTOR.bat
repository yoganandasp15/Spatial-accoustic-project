@echo off
@chcp 65001 >nul
title Record Healthy Motor Physical Experiment (30 Seconds)
color 0B
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL MOTOR RECORDER: HEALTHY MOTOR RUN (30s)
echo          400Hz ADXL345 Vibration (+/-2g) + FC-03 Tachometer + 4-Mic Audio
echo ================================================================================
echo.
echo [INSTRUCTION] Ensure healthy DC motor (motor_A) is securely mounted and running.
echo               Close any open radar GUI (COM8 will be used exclusively).
echo               Duration: 30.0s (Captures ~12,000 vibration and ~300,000 audio samples).
echo.

set "PROJECT_DIR="
if exist "%~dp0tools\record_motor_experiment.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\tools\record_motor_experiment.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\tools\record_motor_experiment.py" (
    set "PROJECT_DIR=%USERPROFILE%\Downloads\Spatial-accoustic-project"
)

if not defined PROJECT_DIR (
    color 0C
    echo [ERROR] Could not locate Spatial-accoustic-project repository.
    pause
    exit /b 1
)

cd /d "%PROJECT_DIR%"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\record_motor_experiment.py --motor-id motor_A --health-label healthy --environment quiet --fault-type none --duration 30.0 --port COM8 %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Acquisition aborted, interrupted, or incomplete! Check metadata/log.
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
