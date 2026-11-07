@echo off
setlocal
echo =====================================================================
echo  SPATIAL MULTIMODAL EDGE-AI -- DUAL-I2S PHASE AUDIT & CALIBRATOR
echo  Measures inter-DMA clock skew between Pair A (I2S0) and Pair B (I2S1)
echo =====================================================================
echo.

cd /d "%~dp0"

python stage2_acoustic_localization\src\calibrate_i2s_phase.py

if %ERRORLEVEL% EQU 0 (
    echo.
    echo =====================================================================
    echo [SUCCESS] Dual-I2S calibration completed!
    echo [CONFIG] stage2_acoustic_localization\evidence\i2s_phase_calibration.json
    echo [PLOT]   stage2_acoustic_localization\evidence\dual_i2s_phase_skew_audit.png
    echo =====================================================================
) else (
    echo.
    echo [ERROR] Calibration execution failed with exit code %ERRORLEVEL%.
)

pause
