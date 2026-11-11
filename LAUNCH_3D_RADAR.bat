@echo off
setlocal EnableExtensions
chcp 65001 >nul
title Spatial Multimodal Acoustic Radar 3D (Pro V7) - WebGL Twin
color 0B
cls

echo ===============================================================================
echo     SPATIAL MULTIMODAL ACOUSTIC RADAR 3D - WEBGL DIGITAL TWIN (PRO V7)
echo ===============================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0stage4_realtime_fusion_radar\src\telemetry_streamer_3d.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\stage4_realtime_fusion_radar\src\telemetry_streamer_3d.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\stage4_realtime_fusion_radar\src\telemetry_streamer_3d.py" (
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

echo [*] Starting local 3D telemetry server on port 8080...
start "" "%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\telemetry_streamer_3d.py" 8080
timeout /t 2 /nobreak >nul

echo [*] Opening 3D Acoustic Camera in default web browser...
start "" http://localhost:8080/

echo.
echo Server active at http://localhost:8080/
echo Close this window or press Ctrl+C to terminate server.
pause
