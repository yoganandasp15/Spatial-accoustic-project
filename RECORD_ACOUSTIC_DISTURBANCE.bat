@echo off
@chcp 65001 >nul
title Record Acoustic Disturbance Rejection Experiment (10 Seconds)
color 0B
cls
echo ╔══════════════════════════════════════════════════════════════════════════════╗
echo ║         SPATIAL MULTIMODAL MOTOR RECORDER: ACOUSTIC DISTURBANCE TEST         ║
echo ║         Proves Environmental Acoustic Noise != Internal Mechanical Fault     ║
echo ╚══════════════════════════════════════════════════════════════════════════════╝
echo.
echo [INSTRUCTION] Run healthy DC motor while introducing external noise (clap/speech).
echo               Close any open radar GUI (COM8 will be used exclusively).
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\record_motor_experiment.py --mode disturbance --duration 10.0 --port COM8
if errorlevel 1 (
    echo.
    echo ════════════════════════════════════════════════════════════════════════════
    echo [ERROR] Acquisition aborted or failed with exit code %ERRORLEVEL%.
    echo ════════════════════════════════════════════════════════════════════════════
    pause
) else (
    echo.
    echo ════════════════════════════════════════════════════════════════════════════
    echo [SUCCESS] Disturbance run recorded to data\experiments\motor_disturbance_run.csv!
    echo ════════════════════════════════════════════════════════════════════════════
    echo.
    pause
)
