@echo off
setlocal EnableExtensions
@chcp 65001 >nul
title Physical 4-Mic Array Ground Truth Acoustic Calibration
color 0B
cls
echo ================================================================================
echo          PHYSICAL 4-MIC ARRAY GROUND TRUTH ACOUSTIC CALIBRATION
echo          Measuring SRP-PHAT Angular Precision Across 0 deg, +-30 deg, +-45 deg
echo ================================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0tools\calibrate_physical_array.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\tools\calibrate_physical_array.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\tools\calibrate_physical_array.py" (
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

:: Auto-detect active ESP32 COM port
set "DETECTED_PORT=COM8"
if exist "tools\find_port.py" (
    for /f "delims=" %%P in ('"%PYTHON_BIN%" "tools\find_port.py" 2^>nul') do (
        set "DETECTED_PORT=%%P"
    )
)

echo [INSTRUCTION] Place acoustic test tone/speaker at marked angles relative to array.
echo               Close any open radar GUI (%DETECTED_PORT% will be used exclusively).
echo.
echo [*] Target Serial Port: %DETECTED_PORT% @ 921,600 Baud
echo.

"%PYTHON_BIN%" tools\calibrate_physical_array.py --port %DETECTED_PORT% --baud 921600 %*
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
