@echo off
setlocal EnableExtensions
title Apply Spatial Acoustic Radar Fix
color 0A

cd /d "%~dp0"
echo ================================================================
echo   APPLY LIVE RADAR FIX + RUN ACOUSTIC REGRESSION TESTS
echo ================================================================
echo.

set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

if not exist "apply_live_radar_fix.py" (
    echo [ERROR] apply_live_radar_fix.py is missing.
    echo Copy it into the Spatial-accoustic-project repository root.
    pause
    exit /b 2
)

"%PYTHON_BIN%" "apply_live_radar_fix.py"
if errorlevel 1 (
    echo.
    echo [ERROR] Patch was not applied. Read the message above.
    pause
    exit /b 1
)

echo.
echo [INFO] Running focused acoustic regression tests...
"%PYTHON_BIN%" "tests\test_noise_gating_regression.py"
if errorlevel 1 (
    echo.
    echo [ERROR] Focused regression tests failed.
    pause
    exit /b 1
)

echo.
echo [SUCCESS] Patch applied and focused regression tests passed.
echo Now start RUN_LIVE_RADAR.bat.
pause
