@echo off
@chcp 65001 >nul
title Spatial Multimodal Edge-AI - Offline Evaluation Benchmark
color 0B
setlocal
cls
echo ================================================================================
echo   SPATIAL MULTIMODAL EDGE-AI -- OFFLINE EVALUATION BENCHMARK
echo   4-Microphone Acoustic Localization + ISO 10816 Vibration Co-Fusion
echo ================================================================================
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

echo [*] Running full multimodal co-fusion on physical burst records...
"%PYTHON_BIN%" stage4_realtime_fusion_radar\src\offline_multimodal_evaluator.py

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