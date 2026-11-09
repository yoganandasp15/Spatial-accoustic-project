@echo off
@chcp 65001 >nul
title Record Acoustic Disturbance Rejection Experiment (30 Seconds)
color 0E
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL MOTOR RECORDER: ACOUSTIC DISTURBANCE TEST (30s)
echo          Proves Environmental Acoustic Noise != Internal Mechanical Fault
echo ================================================================================
echo.
echo [INSTRUCTION] Run healthy DC motor while introducing external noise (speech/music/claps).
echo               Close any open radar GUI (COM8 will be used exclusively).
echo               Duration: 30.0s (Captures ~12,000 vibration and ~300,000 audio samples).
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\record_motor_experiment.py --motor-id motor_A --health-label healthy --environment disturbance --fault-type none --disturbance-desc "ambient acoustic noise" --duration 30.0 --port COM8 %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Acquisition aborted or failed with exit code %ERRORLEVEL%.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Disturbance run recorded to data\dataset\ and registered in manifest!
    echo ================================================================================
    echo.
    pause
)
