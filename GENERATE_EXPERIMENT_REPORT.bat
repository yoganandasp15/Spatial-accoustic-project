@echo off
title Compare Motor Experiments and Generate Academic Report
color 0B
echo =====================================================================
echo    SPATIAL MULTIMODAL MOTOR EXPERIMENT REPORT GENERATOR
echo    Vibration FFT + Rotational Order Tracking + Disturbance Proof
echo =====================================================================
echo.
echo Processing recorded healthy and defective motor runs...
echo.

cd /d "%~dp0"
set PYTHONUNBUFFERED=1
set PYTHON_BIN=C:\Python314\python.exe
if not exist "%PYTHON_BIN%" set PYTHON_BIN=python

"%PYTHON_BIN%" tools\compare_motor_experiments.py
if errorlevel 1 (
    echo.
    echo [ERROR] Report generation exited with error code %ERRORLEVEL%
    pause
) else (
    echo.
    echo =====================================================================
    echo [SUCCESS] Complete Experiment Report Generated!
    echo   - Report: docs\evidence\PHYSICAL_EXPERIMENT_REPORT.md
    echo   - Plots : docs\evidence\physical_motor_vibration_comparison.png
    echo   - Proof : docs\evidence\multimodal_acoustic_disturbance_rejection.png
    echo =====================================================================
    echo.
    pause
)
