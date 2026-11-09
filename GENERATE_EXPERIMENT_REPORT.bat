@echo off
@chcp 65001 >nul
title Compare Motor Runs & Generate Minor Project Academic Report
color 0B
cls
echo ╔══════════════════════════════════════════════════════════════════════════════╗
echo ║         MULTIMODAL MOTOR RUN COMPARISON & ACADEMIC REPORT GENERATOR          ║
echo ║         Rotational Orders (1X/2X) + ISO Velocity RMS + Disturbance Proof     ║
echo ╚══════════════════════════════════════════════════════════════════════════════╝
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\compare_motor_experiments.py
if errorlevel 1 (
    echo.
    echo ════════════════════════════════════════════════════════════════════════════
    echo [ERROR] Report generation exited with error code %ERRORLEVEL%.
    echo ════════════════════════════════════════════════════════════════════════════
    pause
) else (
    echo.
    echo ════════════════════════════════════════════════════════════════════════════
    echo [SUCCESS] Report generated: docs\evidence\PHYSICAL_EXPERIMENT_REPORT.md
    echo ════════════════════════════════════════════════════════════════════════════
    echo.
    pause
)
