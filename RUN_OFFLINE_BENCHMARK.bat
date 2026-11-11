@echo off
setlocal EnableExtensions
@chcp 65001 >nul
title Spatial Multimodal Edge-AI - Offline Evaluation Benchmark
color 0B
cls
echo ================================================================================
echo   SPATIAL MULTIMODAL EDGE-AI -- OFFLINE EVALUATION BENCHMARK
echo   4-Microphone Acoustic Localization + ISO 10816 Vibration Co-Fusion
echo ================================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0stage4_realtime_fusion_radar\src\offline_multimodal_evaluator.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\stage4_realtime_fusion_radar\src\offline_multimodal_evaluator.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\stage4_realtime_fusion_radar\src\offline_multimodal_evaluator.py" (
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

echo [*] Running full multimodal co-fusion on physical burst records...
"%PYTHON_BIN%" stage4_realtime_fusion_radar\src\offline_multimodal_evaluator.py %*

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ================================================================================
    echo [SUCCESS] Evaluation completed!
    echo [ARTIFACT 1] stage4_realtime_fusion_radar\evidence\offline_multimodal_diagnostic_report.png
    echo [ARTIFACT 2] stage4_realtime_fusion_radar\evidence\offline_multimodal_summary.json
    echo ================================================================================
) else (
    echo.
    echo ================================================================================
    echo [ERROR] Benchmark execution failed with exit code %ERRORLEVEL%.
    echo ================================================================================
)

echo.
pause
