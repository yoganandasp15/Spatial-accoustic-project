@echo off
@chcp 65001 >nul
title Validate Spatial Multimodal Motor Dataset Quality (Strict Mode)
color 0A
cls
echo ================================================================================
echo             SPATIAL MULTIMODAL MOTOR DATASET QUALITY AUDITOR
echo                 Strict Mode Enabled: Zero Tolerance for Corruption
echo ================================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0tools\validate_dataset.py" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\tools\validate_dataset.py" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\tools\validate_dataset.py" (
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
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\validate_dataset.py --strict %*
echo.
pause
