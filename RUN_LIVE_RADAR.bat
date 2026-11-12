@echo off
setlocal EnableExtensions
chcp 65001 >nul
title Spatial Multimodal Edge-AI - Rotating Machinery Fault Detection and Localization
color 0B
cls

echo +==============================================================================+
echo ^|        SPATIAL MULTIMODAL EDGE-AI FOR ROTATING MACHINERY DIAGNOSTICS         ^|
echo ^|      ECE Minor Project Evaluation  -  Live Multimodal Sensor Fusion HUD      ^|
echo +==============================================================================+
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
    color 0C
    echo [ERROR] Could not locate spatial_acoustic_radar_pro.py
    pause
    exit /b 1
)

cd /d "%PROJECT_DIR%"
set "PYTHONUNBUFFERED=1"

:: 2. Locate Python binary
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" (
    set "PYTHON_BIN="
    for /f "delims=" %%I in ('where python 2^>nul') do (
        if not defined PYTHON_BIN set "PYTHON_BIN=%%I"
    )
)
if not defined PYTHON_BIN set "PYTHON_BIN=python"

:: 3. Scan genuine streaming ESP32 COM port
set "STREAMING_PORT=NONE"
if exist "tools\find_port.py" (
    for /f "delims=" %%P in ('"%PYTHON_BIN%" "tools\find_port.py" --check-streaming 2^>nul') do (
        set "STREAMING_PORT=%%P"
    )
)

echo  Workspace : %CD%
echo  Python    : %PYTHON_BIN%
if "%STREAMING_PORT%"=="NONE" (
    echo  Hardware  : ESP32 not streaming telemetry ^(Simulation ^& Laptop Mic active^)
) else (
    echo  Hardware  : ESP32 telemetry active on %STREAMING_PORT% @ 921600 Baud
)
echo.

:: 4. Display serial ports via PowerShell
powershell -NoProfile -Command "$p=[System.IO.Ports.SerialPort]::GetPortNames(); if($p){Write-Host ('  [+] Detected COM Ports: '+($p -join ', ')) -ForegroundColor Green}else{Write-Host '  [!] No COM Ports - Demo/Mic modes available' -ForegroundColor Yellow}"

:: 5. Check dependencies
"%PYTHON_BIN%" -c "import numpy, scipy, matplotlib, sounddevice, serial" >nul 2>&1
if errorlevel 1 (
    color 0C
    echo [ERROR] Missing required packages.
    pause
    exit /b 1
)

echo.
echo  +----------------------------------------------------------------------------+
echo  ^|  SELECT INPUT SOURCE ^& OPERATING MODE                                       ^|
echo  +----------------------------------------------------------------------------+
echo  ^|  [1]  ACTIVE RADAR HUD    - Auto-Select (Uses Simulator or ESP32 if live)   ^|
echo  ^|  [2]  PC MICROPHONE MODE  - Live Audio from Laptop Mic (Real Sound Input)   ^|
echo  ^|  [3]  MOTOR SIMULATOR     - High-Speed Test (10k-100k RPM with [UP/DN] keys)^|
echo  ^|  [4]  DIAGNOSTIC HUD      - Compact Right-Panel (Speed Dial + Fault + DOA)  ^|
echo  +----------------------------------------------------------------------------+
echo.
set /p "LAUNCH_CHOICE=  Enter choice (1, 2, 3, or 4) [default=1]: "
if "%LAUNCH_CHOICE%"=="" set "LAUNCH_CHOICE=1"

echo.
echo  Controls: [1] PC Mic  [2] ESP32  [3] Sim  [UP/DN] Sim RPM  [P] PPR  [B] Baseline  [Q] Quit
echo.

if "%LAUNCH_CHOICE%"=="2" goto :launch_mic
if "%LAUNCH_CHOICE%"=="3" goto :launch_sim
if "%LAUNCH_CHOICE%"=="4" goto :launch_diag

:launch_auto
if "%STREAMING_PORT%"=="NONE" (
    echo  [*] Launching Radar in High-Speed Motor Simulator Mode (Active Inputs)...
    echo      (All 7 panels active! Press [UP]/[DOWN] or [+][-] to change RPM)
    echo      (Press '1' anytime inside the GUI to switch to Laptop Microphone)
    echo.
    "%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" --source synthetic --rpm 24000 %*
) else (
    echo  [+] Launching Live Hardware Radar on %STREAMING_PORT%...
    echo.
    "%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" --source esp32 --port %STREAMING_PORT% %*
)
goto :check_exit

:launch_mic
echo  [+] Launching Radar in PC / Laptop Microphone Input Mode...
echo      (Speak, whistle, or run motor near laptop mic to see live acoustic inputs)
echo.
"%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" --source laptop %*
goto :check_exit

:launch_sim
echo  [+] Launching High-Speed Motor Simulator Mode (24,000 RPM baseline)...
echo      (Press [UP]/[DOWN] or [+][-] to dynamically change RPM from 1k to 100k+ RPM)
echo.
"%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" --source synthetic --rpm 24000 %*
goto :check_exit

:launch_diag
echo  [+] Launching Compact Diagnostic HUD...
echo.
if "%STREAMING_PORT%"=="NONE" (
    "%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\diagnostic_hud.py" --source synthetic --rpm 24000 %*
) else (
    "%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\diagnostic_hud.py" --source esp32 --port %STREAMING_PORT% %*
)
goto :check_exit

:check_exit
if errorlevel 1 (
    color 0C
    echo [ERROR] GUI exited with an error.
    pause
)
