@echo off
setlocal
echo =====================================================================
echo  SPATIAL MULTIMODAL EDGE-AI -- AUTOMATED TEST SUITE
echo  Validating GCC-PHAT, SRP-PHAT, ISO 10816, and Order Tracking
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
