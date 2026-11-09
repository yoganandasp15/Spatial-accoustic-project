@echo off
title Record Defective Motor Physical Experiment (10 Seconds)
color 0C
echo =====================================================================
echo    SPATIAL MULTIMODAL MOTOR RECORDER: DEFECTIVE MOTOR RUN
echo    400Hz ADXL345 Vibration + FC-03 Tachometer + 4-Mic INMP441 Audio
echo =====================================================================
echo.
echo [INSTRUCTION] Make sure your DEFECTIVE DC Motor is connected and running.
echo.

cd /d "%~dp0"
set PYTHONUNBUFFERED=1
set PYTHON_BIN=C:\Python314\python.exe
if not exist "%PYTHON_BIN%" set PYTHON_BIN=python

"%PYTHON_BIN%" tools\record_motor_experiment.py --mode defective --duration 10.0 --port COM8
if errorlevel 1 (
    echo.
    echo [ERROR] Acquisition exited with error code %ERRORLEVEL%
    pause
) else (
    echo.
    echo [SUCCESS] Defective motor run recorded to data\experiments\motor_defective_run.csv!
    echo.
    pause
)
