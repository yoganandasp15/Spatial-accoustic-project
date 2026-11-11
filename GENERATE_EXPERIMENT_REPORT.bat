@echo off
setlocal EnableExtensions
@chcp 65001 >nul
title Compare Motor Runs and Generate Minor Project Academic Report
color 0B
cls
echo ================================================================================
echo          MULTIMODAL MOTOR RUN COMPARISON AND ACADEMIC REPORT GENERATOR
echo          Rotational Orders (1X/2X) + ISO Velocity RMS + Disturbance Proof
echo ================================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0tools\compare_motor_experiments.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\tools\compare_motor_experiments.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\tools\compare_motor_experiments.py" (
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

"%PYTHON_BIN%" tools\compare_motor_experiments.py %*
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Report generation exited with error code %ERRORLEVEL%.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] Report generated: docs\evidence\PHYSICAL_EXPERIMENT_REPORT.md
    echo ================================================================================
    echo.
    pause
)
