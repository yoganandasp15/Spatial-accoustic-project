@echo off
@chcp 65001 >nul
title Spatial Multimodal Acoustic Radar Pro - Live 4-Mic and Motor Diagnostic Suite
color 0B
cls
echo ================================================================================
echo           SPATIAL MULTIMODAL ACOUSTIC RADAR ENGINE (PRO V6)
echo     4-Mic INMP441 ULA Array + ADXL345 Vibration + FC-03 Optical Tachometer
echo ================================================================================
echo.

:: 1. Locate repository root directory
set "PROJECT_DIR="
if exist "%~dp0stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" (
    set "PROJECT_DIR=%USERPROFILE%\Downloads\Spatial-accoustic-project"
)

if not defined PROJECT_DIR (
    echo [ERROR] Could not find stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py.
    echo Please make sure the project exists in:
    echo C:\Users\Yogi\Downloads\Spatial-accoustic-project
    echo.
    pause
    exit /b 1
)

cd /d "%PROJECT_DIR%"
set "PYTHONUNBUFFERED=1"

:: 2. Locate Python binary
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" (
    for /f "delims=" %%I in ('where python 2^>nul') do (
        if not defined PYTHON_BIN set "PYTHON_BIN=%%I"
    )
)
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

echo [*] Workspace   : %CD%
echo [*] Python Core : %PYTHON_BIN%
echo [*] ESP32 Serial: COM8 @ 921,600 Baud (Dual-I2S + 400Hz FIFO)
echo.

:: 3. Scan for serial ports
powershell -NoProfile -Command "$ports = [System.IO.Ports.SerialPort]::GetPortNames(); if ($ports) { Write-Host ('[+] Active Serial Ports Detected: ' + ($ports -join ', ')) -ForegroundColor Green } else { Write-Host '[!] NOTE: No active COM ports detected. (Running in Standby / Synthetic mode)' -ForegroundColor Yellow }"

:: 4. Verify core dependencies
"%PYTHON_BIN%" -c "import numpy, scipy, matplotlib, sounddevice" >nul 2>&1
if errorlevel 1 (
    echo.
    echo [ERROR] Required Python packages [numpy, scipy, matplotlib, sounddevice] are missing.
    echo Please run: "%PYTHON_BIN%" -m pip install numpy scipy matplotlib sounddevice pyserial
    echo.
    pause
    exit /b 1
)

echo.
echo --------------------------------------------------------------------------------
echo  KEYBOARD CONTROLS:
echo    [B] Learn Motor Baseline      [C] Clear Baseline      [P] Cycle Tach PPR
echo    [T] Cycle Motor Test Demo     [K] Toggle Phase Calib  [I] Invert Mic Polarity
echo    [1] Laptop Microphone Array   [2] ESP32 4-Mic Array   [Q] Exit Application
echo --------------------------------------------------------------------------------
echo.
echo [+] Initializing Real-Time Interactive Radar HUD...
echo.

"%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py"

if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Application terminated with exit code %ERRORLEVEL%.
    echo Check console messages above for diagnostic details.
    echo ================================================================================
    pause
)