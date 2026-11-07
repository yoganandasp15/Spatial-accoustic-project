@echo off
setlocal
echo =====================================================================
echo  SPATIAL MULTIMODAL EDGE-AI -- OFFLINE EVALUATION BENCHMARK
echo  4-Microphone Acoustic Localization + ISO 10816 Vibration Co-Fusion
echo =====================================================================
echo.

cd /d "%~dp0"

echo [*] Running full multimodal co-fusion on physical burst records...
python stage4_realtime_fusion_radar\src\offline_multimodal_evaluator.py

if %ERRORLEVEL% EQU 0 (
    echo.
    echo =====================================================================
    echo [SUCCESS] Evaluation completed!
    echo [ARTIFACT 1] stage4_realtime_fusion_radar\evidence\offline_multimodal_diagnostic_report.png
    echo [ARTIFACT 2] stage4_realtime_fusion_radar\evidence\offline_multimodal_summary.json
    echo =====================================================================
) else (
    echo.
    echo [ERROR] Benchmark execution failed with exit code %ERRORLEVEL%.
)

pause
