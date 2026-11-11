@echo off
@chcp 65001 >nul
title Record Defective Motor Physical Experiment (30 Seconds)
color 0C
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL MOTOR RECORDER: DEFECTIVE MOTOR RUN (30s)
echo          400Hz ADXL345 Vibration (+/-2g) + FC-03 Tachometer + 4-Mic Audio
echo ================================================================================
echo.
echo [INSTRUCTION] Mount defective/imbalanced DC motor (motor_B) firmly to testbed.
echo               Close any open radar GUI (Serial port will be used exclusively).
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

"%PYTHON_BIN%" tools\record_motor_experiment.py --motor-id motor_B --health-label defective --environment quiet --fault-type unknown --duration 30.0 --port %PORT% %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Acquisition aborted, interrupted, or incomplete! Check metadata/log.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Defective motor run recorded to data\dataset\ and registered in manifest!
    echo ================================================================================
    echo.
    pause
)
