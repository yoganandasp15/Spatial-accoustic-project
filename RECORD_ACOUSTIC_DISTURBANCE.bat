@echo off
@chcp 65001 >nul
title Record Acoustic Disturbance Control Experiment (30 Seconds)
color 0E
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL MOTOR RECORDER: ACOUSTIC DISTURBANCE (30s)
echo          400Hz ADXL345 Vibration (+/-2g) + FC-03 Tachometer + 4-Mic Audio
echo ================================================================================
echo.
echo [INSTRUCTION] Run healthy DC motor (motor_A) while introducing external noise:
echo               (e.g., play loud audio from phone, clap, or speak nearby).
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
if not exist "%PYTHON_BIN%" (
    for /f "delims=" %%I in ('where python 2^>nul') do (
        if not defined PYTHON_BIN set "PYTHON_BIN=%%I"
    )
)
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

set "PORT=COM8"
if exist "tools\find_port.py" (
    for /f "delims=" %%P in ('"%PYTHON_BIN%" "tools\find_port.py" 2^>nul') do (
        set "PORT=%%P"
    )
)

echo [*] Target Serial Port : %PORT% (Auto-Detected)
echo.

"%PYTHON_BIN%" tools\record_motor_experiment.py --motor-id motor_A --health-label healthy --environment disturbance --fault-type none --disturbance-desc "External acoustic disturbance" --duration 30.0 --port %PORT% %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Acquisition aborted, interrupted, or incomplete! Check metadata/log.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Acoustic Disturbance run recorded to data\dataset\ and registered!
    echo ================================================================================
    echo.
    pause
)
