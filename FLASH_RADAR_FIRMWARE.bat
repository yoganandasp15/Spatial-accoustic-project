@echo off
@chcp 65001 >nul
title Flash ESP32 Spatial Acoustic Radar Firmware (Type 0x05 DMA Streamer)
color 0B
cls
echo ================================================================================
echo       FLASH ESP32 SPATIAL ACOUSTIC RADAR FIRMWARE (PRO V6 / TYPE 0x05)
echo       Target: ESP32-WROOM-32 on COM8 (4-Mic Dual-I2S + ADXL345 + FC-03)
echo ================================================================================
echo.

set "ARDUINO_CLI=C:\Program Files\Arduino CLI\arduino-cli.exe"
if not exist "%ARDUINO_CLI%" set "ARDUINO_CLI=arduino-cli"

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

set "SKETCH_DIR=%PROJECT_DIR%\stage3_esp32_firmware\ESP32_Spatial_Acoustic_Radar_Firmware"

set "PORT=COM8"
if exist "%PROJECT_DIR%\tools\find_port.py" (
    for /f "delims=" %%P in ('python "%PROJECT_DIR%\tools\find_port.py" 2^>nul') do (
        set "PORT=%%P"
    )
)
if not "%~1"=="" set "PORT=%~1"

echo [*] Repository Root   : %PROJECT_DIR%
echo [*] Target Serial Port : %PORT% (Auto-Detected)
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
    echo [SUCCESS] Radar Firmware Flashed Successfully!
    echo           Streaming Type 0x05 binary packets on %PORT% at 921,600 baud.
    echo ================================================================================
    echo.
    pause
)
