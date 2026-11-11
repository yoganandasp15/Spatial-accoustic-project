@echo off
setlocal EnableExtensions
@chcp 65001 >nul
title Flash ESP32 Spatial Acoustic Radar Firmware (Ultra-High-Speed Tachometer)
color 0B

set "PROJECT_DIR="
if exist "%~dp0stage3_esp32_firmware\ESP32_Spatial_Acoustic_Radar_Firmware\ESP32_Spatial_Acoustic_Radar_Firmware.ino" (
    set "PROJECT_DIR=%~dp0"
) else if exist "C:\Users\Yogi\Downloads\Spatial-accoustic-project\stage3_esp32_firmware\ESP32_Spatial_Acoustic_Radar_Firmware\ESP32_Spatial_Acoustic_Radar_Firmware.ino" (
    set "PROJECT_DIR=C:\Users\Yogi\Downloads\Spatial-accoustic-project"
) else if exist "%USERPROFILE%\Downloads\Spatial-accoustic-project\stage3_esp32_firmware\ESP32_Spatial_Acoustic_Radar_Firmware\ESP32_Spatial_Acoustic_Radar_Firmware.ino" (
    set "PROJECT_DIR=%USERPROFILE%\Downloads\Spatial-accoustic-project"
)

if not defined PROJECT_DIR (
    color 0C
    echo [ERROR] Could not locate Spatial-accoustic-project repository.
    echo         Expected at: C:\Users\Yogi\Downloads\Spatial-accoustic-project
    pause
    exit /b 1
)

cd /d "%PROJECT_DIR%"

set "ARDUINO_CLI=C:\Program Files\Arduino CLI\arduino-cli.exe"
if not exist "%ARDUINO_CLI%" set "ARDUINO_CLI=arduino-cli"

set "PYTHON_BIN=C:\Python314\python.exe"
if not exist "%PYTHON_BIN%" (
    for /f "delims=" %%I in ('where python 2^>nul') do (
        if not defined PYTHON_BIN set "PYTHON_BIN=%%I"
    )
)
if not exist "%PYTHON_BIN%" set "PYTHON_BIN=python"

set "SKETCH_DIR=%PROJECT_DIR%\stage3_esp32_firmware\ESP32_Spatial_Acoustic_Radar_Firmware"

:SCAN_PORT
cls
echo ================================================================================
echo       FLASH ESP32 SPATIAL ACOUSTIC RADAR FIRMWARE (PRO V6 / TYPE 0x05)
echo       Zero-Debounce Optical Tachometer (>100k RPM) + 4-Mic ULA + ADXL345
echo ================================================================================
echo.

set "PORT=NONE"
if not "%~1"=="" (
    set "PORT=%~1"
) else if exist "%PROJECT_DIR%\tools\find_port.py" (
    for /f "delims=" %%P in ('"%PYTHON_BIN%" "%PROJECT_DIR%\tools\find_port.py" --strict 2^>nul') do (
        set "PORT=%%P"
    )
)

if "%PORT%"=="NONE" (
    echo  [!] No ESP32 USB COM port detected!
    echo.
    echo  [ACTION] Please plug your ESP32 board into the USB port.
    echo           (Ensure the USB cable supports data transfer).
    echo.
    set /p RETRY="Press Enter to re-scan for ESP32 (or Q to exit): "
    if /i "%RETRY%"=="q" exit /b 1
    goto SCAN_PORT
)

echo [*] Repository Root   : %PROJECT_DIR%
echo [*] Target Serial Port : %PORT% (Auto-Detected Genuine USB Bridge)
echo [*] Tachometer Config : Zero-Debounce Direct ISR (Supports >100,000+ RPM)
echo [*] Sketch Directory   : %SKETCH_DIR%
echo.
echo [*] Compiling and uploading firmware to %PORT%...
echo.

"%ARDUINO_CLI%" compile --fqbn esp32:esp32:esp32 --upload -p %PORT% "%SKETCH_DIR%"
if errorlevel 1 (
    echo.
    echo ================================================================================
    echo [ERROR] Firmware flash failed with exit code %ERRORLEVEL%.
    echo [TIP] If connecting timed out, press and hold the 'BOOT' button on the ESP32,
    echo       then press Enter to retry.
    echo ================================================================================
    pause
) else (
    echo.
    echo ================================================================================
    echo [SUCCESS] High-Speed Tachometer Radar Firmware Flashed Successfully!
    echo           Streaming Type 0x05 binary packets on %PORT% at 921,600 baud.
    echo           Full bandwidth active: captures fast motor pulses up to >100,000 RPM.
    echo ================================================================================
    echo.
    pause
)
