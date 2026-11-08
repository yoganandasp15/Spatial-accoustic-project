# Stage 4: Real-Time Multimodal Sensor Fusion Radar
**Application Release Date:** 6–7 November 2026

## Objective
Provide an interactive acoustic camera and multimodal fault diagnostic HUD running at 20 FPS on a laptop, receiving live streaming telemetry from the ESP32 over `COM8 @ 1,152,000 baud` (Type 0x05 binary protocol).

## Key Capabilities
1. **2D Acoustic Camera Thermal Heatmap:** Backprojects acoustic energy onto a $60 \times 60$ floor grid using Steered Response Power (SRP-PHAT).
2. **Directional Sound Intensity Rails:** Dynamic Left / Center / Right VU meters with square-root perceptual scaling and color alerts (Cyan $\to$ Amber $\to$ Red).
3. **Dynamic Vibration & ISO 10816 Severity:** Automated grading based on per-axis gravity-decoupled dynamic vibration and frequency-domain $\omega$-arithmetic velocity integration (mm/s).
4. **Acoustic Kurtosis ($\kappa$) & Crest Factor:** Detects impulsive commutator sparking and bearing micro-impacts ($\kappa > 4.5$).
5. **Golden Baseline Differential Engine (Key `'B'`):** Captures reference healthy motor signature and computes real-time deltas ($\Delta\text{Vib}, \Delta\text{RPM}, \Delta\text{Kurt}, \Delta\text{RMS}$) to detect microscopic differences in DC motor condition.
6. **Array Polarity Inversion (Key `'I'`):** Instantly flips channel indexing (`np.fliplr`) to match the user's breadboard orientation.

## How to Run
Double-click `RUN_LIVE_RADAR.bat` in the repository root or execute:
```powershell
python stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py
```
