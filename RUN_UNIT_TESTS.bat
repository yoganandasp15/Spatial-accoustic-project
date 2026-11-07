@echo off
setlocal
echo =====================================================================
echo  SPATIAL MULTIMODAL EDGE-AI -- AUTOMATED TEST SUITE (MILESTONE 3 / R4)
echo  Validating GCC-PHAT, SRP-PHAT, Phase Compensation, Dynamic Vibration,
echo  16-bit XOR Checksum, and Rotational Order Tracking
echo =====================================================================
echo(

cd /d "%~dp0"

python -m unittest tests\test_multimodal_pipeline.py

if not errorlevel 1 (
    echo(
    echo =====================================================================
    echo [PASS] All test suite assertions verified successfully!
    echo =====================================================================
) else (
    echo(
    echo [FAIL] Unit tests encountered assertions errors.
)

pause
