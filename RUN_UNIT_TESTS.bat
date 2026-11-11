@echo off
setlocal EnableExtensions
@chcp 65001 >nul
title Spatial Multimodal Edge-AI -- Automated Test Suite
color 0B
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL EDGE-AI -- COMPLETE AUTOMATED TEST SUITE
echo    69 Unit and Adversarial Tests across DSP, Multimodal, Firmware, and Physics
echo ================================================================================
echo.

set "PROJECT_DIR="
if exist "%~dp0tests" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\tests" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\tests" (
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

echo [*] Running full discovery across 'tests' directory...
echo.

"%PYTHON_BIN%" -m unittest discover -s tests %*

if not errorlevel 1 (
    echo.
    echo ================================================================================
    echo [PASS] All test assertions passed cleanly! Zero regressions detected.
    echo ================================================================================
) else (
    echo.
    echo ================================================================================
    echo [FAIL] Test suite encountered assertion errors. Check log output above.
    echo ================================================================================
)

echo.
pause
