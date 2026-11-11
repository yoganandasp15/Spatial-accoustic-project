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
echo ^|                                                                              ^|
echo ^|  CORE MINOR PROJECT VISION:                                                  ^|
echo ^|    Single sensors are blind: Vibration has zero spatial awareness, while     ^|
echo ^|    an Acoustic hotspot is not automatically a mechanical fault.              ^|
echo ^|                                                                              ^|
echo ^|    By co-fusing 3 physical modalities in real time:                          ^|
echo ^|      [1] 4x INMP441 Mic ULA  --^>  SRP-PHAT 2D Acoustic Camera (WHERE IS IT?) ^|
echo ^|      [2] ADXL345 3-Axis Vib  --^>  400 Hz Dynamic g and ISO Velocity (Zone)   ^|
echo ^|      [3] FC-03 Optical Tach  --^>  Exact Shaft RPM and 1x/2x/3x Orders        ^|
echo ^|                                                                              ^|
echo ^|    The system answers BOTH:                                                  ^|
echo ^|      * WHAT IS WRONG?   (Unbalance 1x, Misalignment 2x, Impact, or Noise)    ^|
echo ^|      * WHERE IS IT?     (Real-Time DOA Bearing Angle [-90 to +90 deg])       ^|
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
    echo [ERROR] Could not locate stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py
    echo         Please verify the repository is in:
    echo         C:\Users\Yogi\Downloads\Spatial-accoustic-project
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

:: 3. Auto-apply GUI layout fix if fix_radar_ui.py is present
if exist "fix_radar_ui.py" (
    "%PYTHON_BIN%" "fix_radar_ui.py" >nul 2>&1
)

:: 4. Auto-detect active ESP32 COM port
set "DETECTED_PORT=COM8"
if exist "tools\find_port.py" (
    for /f "delims=" %%P in ('"%PYTHON_BIN%" "tools\find_port.py" 2^>nul') do (
        set "DETECTED_PORT=%%P"
    )
)

echo  +----------------------------------------------------------------------------+
echo  ^| SYSTEM TELEMETRY CONFIGURATION                                             ^|
echo  +----------------------------------------------------------------------------+
echo    Workspace Root : %CD%
echo    Python Engine  : %PYTHON_BIN%
echo    ESP32 Link     : %DETECTED_PORT% @ 921,600 Baud (Auto-Detected Type 0x05 Streamer)
echo    Array Geometry : 4-Mic INMP441 ULA (Pitch: 20.32 mm, Span: 60.96 mm)
echo  +----------------------------------------------------------------------------+
echo.

:: 5. Scan and display all active COM ports
powershell -NoProfile -Command "$ports = [System.IO.Ports.SerialPort]::GetPortNames(); if ($ports) { Write-Host ('  [+] Active Serial Ports Detected : ' + ($ports -join ', ')) -ForegroundColor Green } else { Write-Host '  [!] No COM Ports Detected        : Launching in Standby / Demo Mode (Press T)' -ForegroundColor Yellow }"

:: 6. Verify core Python dependencies
"%PYTHON_BIN%" -c "import numpy, scipy, matplotlib, sounddevice" >nul 2>&1
if errorlevel 1 (
    color 0C
    echo.
    echo  [ERROR] Missing required Python packages: numpy, scipy, matplotlib, sounddevice
    echo  Run: "%PYTHON_BIN%" -m pip install numpy scipy matplotlib sounddevice pyserial
    echo.
    pause
    exit /b 1
)

echo.
echo  +----------------------------------------------------------------------------+
echo  ^| INTERACTIVE LIVE DEMONSTRATION CONTROLS                                    ^|
echo  +----------------------------------------------------------------------------+
echo  ^|  [B] Lock Golden Baseline      [C] Clear Baseline      [P] Cycle Tach PPR  ^|
echo  ^|  [T] Cycle Motor Fault Demos   [I] Flip Mic Polarity   [K] Phase Calib     ^|
echo  ^|  [1] Laptop Internal Mic       [2] ESP32 4-Mic Array   [Q] Quit Radar      ^|
echo  +----------------------------------------------------------------------------+
echo.
echo  [+] Launching Spatial Multimodal Acoustic Radar Pro HUD on %DETECTED_PORT%...
echo.

"%PYTHON_BIN%" "stage4_realtime_fusion_radar\src\spatial_acoustic_radar_pro.py" %*

if errorlevel 1 (
    color 0C
    echo.
    echo +==============================================================================+
    echo ^| [ERROR] Radar GUI exited with code %ERRORLEVEL%. See diagnostic output above.         ^|
    echo +==============================================================================+
    pause
)
