# Project Milestones & Future Major-Project Roadmap

---

## 1. Minor Project Deliverables (Completed for 7 November 2026)

All planned minor-project engineering goals have been achieved, verified, and packaged:
1. **Stage 1 (Vibration Physics):** Verified rotational order signatures on CWRU and MAFAULDA datasets ($1\times$ unbalance surge, $2\times$ misalignment).
2. **Stage 2 (Acoustic Localization Simulation):**
   * Implemented Sinc-interpolated super-resolution GCC-PHAT and 2D SRP-PHAT beamforming.
   * Benchmarked across 25 off-grid test positions ($0.34^\circ$ median angular error, $100\%$ pass rate).
   * Verified noise robustness down to 0 dB SNR and simulated multipath reverberation.
   * Artifacts saved: `evidence/stage2/` and `notebooks/Stage2_Acoustic_Localization.ipynb`.
3. **Stage 3 (Physical Hardware Acquisition):**
   * Solved classic ESP32 TDM limitation via synchronous Dual-I2S DMA streaming (I2S0 + I2S1 sharing SCK on GPIO 18 and WS on GPIO 21).
   * Integrated ADXL345 on I2C and FC-03 tachometer on GPIO 4 ISR.
   * Flashed and verified live 11,025 Hz 4-channel PCM streaming on COM8 at 921,600 baud.
4. **Stage 4 (Real-Time Multimodal Sensor Fusion):**
   * Built interactive acoustic camera GUI (`src/spatial_acoustic_radar_pro.py`).
   * Implemented ISO 10816 vibration severity grading and acoustic kurtosis / crest factor diagnostics.
   * Implemented Golden Baseline differential engine (Key `'B'`) to detect microscopic differences in DC motor condition.
5. **Presentation Package:**
   * 10-slide executive PowerPoint presentation: `PRESENTATION_Spatial_Multimodal_EdgeAI.pptx`.
   * Comprehensive written report: `MINOR_PROJECT_FINAL_REPORT.md`.

---

## 2. Checklist for the User / Evaluators Today

1. **Open the PowerPoint Presentation:**
   * Launch `PRESENTATION_Spatial_Multimodal_EdgeAI.pptx` for the project review panel.
2. **Execute the Stage 2 Verification Notebook / Script (Optional Proof):**
   ```powershell
   python src/stage2_acoustic_localization.py
   ```
   * Verifies that all 25 test positions achieve $0.34^\circ$ median error and generates fresh evidence in `evidence/stage2/`.
3. **Run the Live Interactive Hardware Demonstration:**
   * Connect the ESP32 USB cable to the laptop (COM8).
   * Double-click `RUN_LIVE_RADAR.bat` (or run `python src/spatial_acoustic_radar_pro.py`).
   * **Test 1 (Acoustic Tracking):** Speak or snap fingers on the left and right sides of the breadboard; observe the DOA needle and left/right VU meters respond immediately. (Press `'I'` if you want to flip channel polarity).
   * **Test 2 (Motor Health Baseline):** Run the DC motor unloaded, press **`'B'`** to calibrate and lock the **Golden Baseline**.
   * **Test 3 (Fault Sensitivity):** Lightly apply finger friction to the motor shaft: observe speed drop ($\Delta\text{RPM} < 0$), audio RMS surge, and the Health Index drop from 95% to 68%.
   * **Test 4 (Encoder Disk):** If using a multi-slot encoder wheel, press **`'P'`** to cycle pulses per revolution.

---

## 3. Major Project Scaling Roadmap (Next Academic Year)

1. **Standalone Onboard Edge-AI Inference:**
   * Migrate from classic ESP32 to **ESP32-S3** (dual-core Xtensa LX7 with vector instructions) or an **Edge TPU / Kendryte K210**.
   * Deploy lightweight 1D-CNN or Spatio-Temporal TinyML models directly in flash memory for zero-latency offline inference without a laptop.
2. **Industrial-Grade Sensor Front-Ends:**
   * Design galvanic isolation and industrial 4–20 mA current loop / IEPE interfaces to monitor heavy-duty induction motors (3-phase 415V).
3. **Longitudinal Degradation Tracking & Predictive Maintenance:**
   * Run continuous multi-week bearing degradation experiments to capture run-to-failure vibration/acoustic trajectories.
   * Train Remaining Useful Life (RUL) prognostic models to predict hours until mechanical failure.
