@echo off
@chcp 65001 >nul
title Spatial Multimodal Acoustic Radar Pro | Live 4-Mic & Motor Diagnostic Suite
color 0B
cls
echo ╔══════════════════════════════════════════════════════════════════════════════╗
echo ║          SPATIAL MULTIMODAL ACOUSTIC RADAR ENGINE (PRO V6)                   ║
echo ║    4-Mic INMP441 ULA Array + ADXL345 Vibration + FC-03 Optical Tachometer    ║
echo ╚══════════════════════════════════════════════════════════════════════════════╝
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"

set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

echo [*] Workspace   : %CD%
echo [*] Python Core : %PYTHON_BIN%
echo [*] ESP32 Serial: COM8 @ 921,600 Baud (Dual-I2S + 400Hz FIFO)
echo.

powershell -NoProfile -Command "$ports = [System.IO.Ports.SerialPort]::GetPortNames(); if ($ports) { Write-Host ('[+] Active Serial Ports Detected: ' + ($ports -join ', ')) -ForegroundColor Green } else { Write-Host '[!] NOTE: No active COM ports detected. (Running in Standby / Synthetic mode)' -ForegroundColor Yellow }"

echo.
echo ────────────────────────────────────────────────────────────────────────────────
echo  KEYBOARD CONTROLS:
echo    [B] Learn Motor Baseline      [C] Clear Baseline      [P] Cycle Tach PPR
echo    [T] Cycle Motor Test Demo     [K] Toggle Phase Calib  [I] Invert Mic Polarity
echo    [1] Laptop Microphone Array   [2] ESP32 4-Mic Array   [Q] Exit Application
echo ────────────────────────────────────────────────────────────────────────────────
echo.
echo [+] Initializing Real-Time Interactive Radar HUD...
echo.

"%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py"

if errorlevel 1 (
    echo.
    echo ════════════════════════════════════════════════════════════════════════════
    echo [ERROR] Application terminated with exit code %ERRORLEVEL%.
    echo Check console messages above for diagnostic details.
    echo ════════════════════════════════════════════════════════════════════════════
    pause
)
