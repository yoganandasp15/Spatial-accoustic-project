@echo off
@chcp 65001 >nul
title Record Defective Motor + Acoustic Disturbance (30 Seconds)
color 0D
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL MOTOR RECORDER: DEFECTIVE + DISTURBANCE (30s)
echo          Verifies Defective State Persists During External Acoustic Noise
echo ================================================================================
echo.
echo [INSTRUCTION] Run defective DC motor (motor_B) while introducing external sound
echo               (phone speech/music/claps) 30-50cm from the microphone array.
echo               Close any open radar GUI (COM8 will be used exclusively).
echo               Duration: 30.0s (Captures ~12,000 vibration and ~300,000 audio samples).
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\record_motor_experiment.py --motor-id motor_B --health-label defective --environment disturbance --fault-type unknown --disturbance-desc "ambient acoustic noise" --duration 30.0 --port COM8 %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Acquisition aborted or failed with exit code %ERRORLEVEL%.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Defective disturbance run recorded to data\dataset\ and registered in manifest!
    echo ================================================================================
    echo.
    pause
)
