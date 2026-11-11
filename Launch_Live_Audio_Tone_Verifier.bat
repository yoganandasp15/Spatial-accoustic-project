@echo off
setlocal EnableExtensions
chcp 65001 >nul
title ESP32 Live 4-Mic Audio Tone & Multimodal Probe
color 0B
cls
echo =====================================================================
echo    ESP32 LIVE 4-MIC AUDIO TONE & MULTIMODAL PROBE (921,600 BAUD)
echo =====================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0tools\live_frequency_tone_probe.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\tools\live_frequency_tone_probe.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\tools\live_frequency_tone_probe.py" (
    set "PROJECT_DIR=%USERPROFILE%\Downloads\Spatial-accoustic-project"
)

set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" (
    for /f "delims=" %%I in ('where python 2^>nul') do (
        if not defined PYTHON_BIN set "PYTHON_BIN=%%I"
    )
)
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

:: Auto-detect active ESP32 COM port
set "DETECTED_PORT=COM8"
if defined PROJECT_DIR (
    if exist "%PROJECT_DIR%\tools\find_port.py" (
        for /f "delims=" %%P in ('"%PYTHON_BIN%" "%PROJECT_DIR%\tools\find_port.py" 2^>nul') do (
            set "DETECTED_PORT=%%P"
        )
    )
)

set "SCRIPT_PATH="
if defined PROJECT_DIR (
    if exist "%PROJECT_DIR%\tools\live_frequency_tone_probe.py" set "SCRIPT_PATH=%PROJECT_DIR%\tools\live_frequency_tone_probe.py"
)
if not defined SCRIPT_PATH (
    if exist "C:\Users\Yogi\Downloads\live_frequency_tone_probe.py" set "SCRIPT_PATH=C:\Users\Yogi\Downloads\live_frequency_tone_probe.py"
)

if not defined SCRIPT_PATH (
    color 0C
    echo [ERROR] Could not locate live_frequency_tone_probe.py.
    pause
    exit /b 1
)

echo Target Hardware Port : %DETECTED_PORT% @ 921,600 Baud
echo Tachometer Engine    : Zero-Debounce Direct ISR (Supports >100,000+ RPM)
echo Audio Script Path    : %SCRIPT_PATH%
echo.
echo Connecting to ESP32 on %DETECTED_PORT%...
echo Play a 1000 Hz, 500 Hz, or 2000 Hz tone from your phone near the breadboard!
echo (Press 'P' or pass --ppr N to change tachometer pulses per revolution)
echo.

"%PYTHON_BIN%" "%SCRIPT_PATH%" --port %DETECTED_PORT% %*
if errorlevel 1 (
    echo.
    echo [ERROR] Process exited with code %ERRORLEVEL%
    pause
)
