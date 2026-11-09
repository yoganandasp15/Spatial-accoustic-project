@echo off
@chcp 65001 >nul
title Spatial Multimodal Edge-AI -- Automated Test Suite
color 0B
setlocal
cls
echo ================================================================================
echo          SPATIAL MULTIMODAL EDGE-AI -- COMPLETE AUTOMATED TEST SUITE
echo    55 Unit and Adversarial Tests across DSP, Multimodal, Firmware, and Physics
echo ================================================================================
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"

set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

echo [*] Running full discovery across 'tests' directory...
echo.

"%PYTHON_BIN%" -m unittest discover -s tests

if not errorlevel 1 (
    echo.
    echo ================================================================================
    echo [PASS] All 55 test assertions passed cleanly! Zero regressions detected.
    echo ================================================================================
) else (
    echo.
    echo ================================================================================
    echo [FAIL] Test suite encountered assertion errors. Check log output above.
    echo ================================================================================
)

echo.
pause