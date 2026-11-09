@echo off
@chcp 65001 >nul
title Validate Spatial Multimodal Motor Dataset Quality
color 0A
cls
echo ================================================================================
echo             SPATIAL MULTIMODAL MOTOR DATASET QUALITY AUDITOR
echo ================================================================================
echo.

cd /d "%~dp0"
set "PYTHONUNBUFFERED=1"
set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

"%PYTHON_BIN%" tools\validate_dataset.py %*
echo.
pause
