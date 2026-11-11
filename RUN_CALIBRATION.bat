@echo off
setlocal EnableExtensions
@chcp 65001 >nul
title Spatial Multimodal Edge-AI - Dual-I2S Phase Audit and Calibrator
color 0B
cls
echo ================================================================================
echo   SPATIAL MULTIMODAL EDGE-AI -- DUAL-I2S PHASE AUDIT AND CALIBRATOR
echo   Measures inter-DMA clock skew between Pair A (I2S0) and Pair B (I2S1)
echo ================================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0stage2_acoustic_localization\src\calibrate_i2s_phase.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\stage2_acoustic_localization\src\calibrate_i2s_phase.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\stage2_acoustic_localization\src\calibrate_i2s_phase.py" (
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

"%PYTHON_BIN%" stage2_acoustic_localization\src\calibrate_i2s_phase.py %*

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ================================================================================
    echo [SUCCESS] Dual-I2S calibration completed!
    echo [CONFIG] stage2_acoustic_localization\evidence\i2s_phase_calibration.json
    echo [PLOT]   stage2_acoustic_localization\evidence\dual_i2s_phase_skew_audit.png
    echo ================================================================================
) else (
    echo.
    echo ================================================================================
    echo [ERROR] Calibration execution failed with exit code %ERRORLEVEL%.
    echo ================================================================================
)

echo.
pause
