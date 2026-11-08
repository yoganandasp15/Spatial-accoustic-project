@echo off
title Spatial Multimodal Acoustic Radar Pro (4-Mic Array + DC Motor Diagnostics)
color 0A
echo =====================================================================
echo    SPATIAL MULTIMODAL ACOUSTIC RADAR ENGINE (PRO V6)
echo    4-Microphone Array + ADXL345 Vibration + FC-03 Tachometer
echo =====================================================================
echo.
echo Starting Real-Time Acoustic Camera and Multimodal Telemetry Suite...
echo.

cd /d "%~dp0"
set PYTHONUNBUFFERED=1

set PYTHON_BIN=C:\Python314\python.exe
if not exist "%PYTHON_BIN%" set PYTHON_BIN=python

"%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py"
if errorlevel 1 (
    echo.
    echo [ERROR] Application exited with error code %ERRORLEVEL%
    pause
)
