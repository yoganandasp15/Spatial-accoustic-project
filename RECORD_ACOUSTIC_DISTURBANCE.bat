@echo off
title Record Acoustic Disturbance Rejection Experiment (10 Seconds)
color 0E
echo =====================================================================
echo    SPATIAL MULTIMODAL MOTOR RECORDER: ACOUSTIC DISTURBANCE RUN
echo    Demonstrating PDF Page 13: Acoustic Disturbance != Mechanical Fault
echo =====================================================================
echo.
echo [INSTRUCTION] Run the Healthy Motor, and CLAP or SPEAK near the mics!
echo               The system will prove the motor vibration stays smooth!
echo.

cd /d "%~dp0"
set PYTHONUNBUFFERED=1
set PYTHON_BIN=C:\Python314\python.exe
if not exist "%PYTHON_BIN%" set PYTHON_BIN=python

"%PYTHON_BIN%" tools\record_motor_experiment.py --mode disturbance --duration 10.0 --port COM8
if errorlevel 1 (
    echo.
    echo [ERROR] Acquisition exited with error code %ERRORLEVEL%
    pause
) else (
    echo.
    echo [SUCCESS] Disturbance test recorded to data\experiments\motor_disturbance_run.csv!
    echo.
    pause
)
