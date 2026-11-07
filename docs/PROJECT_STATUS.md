# Project Status and Evidence Log

**Status snapshot:** 7 November 2026. **Minor-project evaluation & demonstration.**

This document records the verified empirical evidence, implemented stages, and hardware validation status across the entire project pipeline with strict academic distinctions between physical prototype streaming, open-source dataset benchmarks, and simulation proofs.

---

## 1. Overview of Project Completion & Evidence Tiers

| Stage | Focus Area | Evidence Tier | Status | Key Evidence / Artifact |
| :--- | :--- | :---: | :---: | :--- |
| **Stage 1A & 1B** | Rotational Order & Vibration Physics | **Open Dataset** | **VERIFIED** | `evidence/stage1b_csvs/` (CWRU & MAFAULDA matched records) |
| **Stage 2** | Acoustic Source Localization & 2D Heatmap | **Numerical Sim** | **VERIFIED** | `evidence/stage2/` (25-point benchmark, 0.34° median error) |
| **Stage 3** | ESP32 Hardware Acquisition & Dual-I2S | **Physical Hardware**| **VERIFIED** | `stage3_esp32_firmware/` (11,025 Hz 4-ch DMA on COM8 @ 921.6k baud) |
| **Stage 4** | Real-Time Multimodal Sensor Fusion HUD | **Live Demonstrator**| **VERIFIED** | `stage4_realtime_fusion_radar/` (Golden Baseline engine & GUI) |
| **Presentation** | Minor Project Review Slide Deck & Report | **Documentation** | **READY** | `PRESENTATION_Spatial_Multimodal_EdgeAI.pptx` & Report |

---

## 2. Preliminary CWRU Experiment (Stage 1A)
- Verified spectral feature extraction on healthy versus inner-race-bearing recordings (`97.mat`, `98.mat`, `105.mat`).
- Resolved RPM metadata fallback for files without explicit header fields.
- Evidence archive: `evidence/cwru_initial/cwru_initial_results.zip`.

---

## 3. MAFAULDA Matched-Group Analysis (Stage 1B)
- Analyzed 9 real MAFAULDA recordings across 3 nominal speed groups (31.7 Hz, 45.5 Hz, 53.9 Hz) comparing Normal, 10 g Imbalance, and 1.5 mm Horizontal Misalignment.
- **Tachometer Consistency:** Edge-counting vs. spectral peak frequency matched to within **0.0468%**, demonstrating reliable internal speed measurement.
- **Rotor Mass Unbalance Finding:** Radial vibration under 10 g unbalance showed a massive, consistent increase in $1\times$ shaft rotational order energy:
  * 31.7 Hz Group: 8.78% (Normal) $\to$ **40.74%** (Unbalance)
  * 45.5 Hz Group: 8.39% (Normal) $\to$ **61.39%** (Unbalance)
  * 53.9 Hz Group: 10.21% (Normal) $\to$ **66.57%** (Unbalance)
- Evidence archive: `evidence/stage1b/mafaulda_stage1b_outputs.zip`. Extracted summary CSVs: `evidence/stage1b_csvs/`.

---

## 4. Acoustic Source Localization Simulation Suite (Stage 2)
- **Array Geometry:** 4-Microphone Uniform Linear Array spaced at $d = 20.32\text{ mm}$ ($60.96\text{ mm}$ total span), matching the physical breadboard aperture.
- **Physical Observables:** For a 1D linear array, Direction of Arrival (DOA bearing angle $\theta \in [-90^\circ, +90^\circ]$) is the primary physical observable; 2D $(x,y)$ heatmaps represent radial backprojection along the steered angle.
- **Pairwise GCC-PHAT TDOA Verification (Stage 2B):**
  * Sinc-interpolated cross-correlation evaluated across all 6 microphone pairs against analytical ground truth.
  * Maximum delay error observed: **$<0.19\ \mu\text{s}$** (less than $1/50\text{th}$ of a single audio sample at 16 kHz).
  * Evidence plot: `evidence/stage2/stage2_tdoa_gcc_phat_verification.png`.
- **25-Point Off-Grid Spatial Benchmark (Stage 2C):**
  * Evaluated across 25 random off-grid source positions spanning $-65^\circ$ to $+65^\circ$ and $0.65\text{ m}$ to $2.10\text{ m}$ range.
  * **Median Angular Error:** **0.34°**
  * **Mean Angular Error:** **0.35°**
  * **90th-Percentile Error:** **0.68°**
  * **Angular Success Rate ($<5^\circ$ error):** **100.0% Pass Rate**
  * Evidence plot: `evidence/stage2/stage2_srp_heatmaps_sample.png`.
  * Summary CSV: `evidence/stage2/stage2_localization_benchmarks.csv`.
- **Noise Robustness & Multipath Sweeps (Stage 2D):**
  * **Clean ($\infty$ dB):** Median Error = 0.34°, 90th% = 0.55° (100% pass)
  * **20 dB SNR:** Median Error = 0.34°, 90th% = 0.61° (100% pass)
  * **10 dB SNR:** Median Error = 0.55°, 90th% = 1.52° (100% pass)
  * **0 dB SNR:** Median Error = 2.52°, 90th% = 5.87° (80% pass)
  * **Multipath / Reverberation (2 wall reflections, $\beta=0.35$):** Median Error = 0.40° (84% pass)
  * Evidence plot: `evidence/stage2/stage2_snr_robustness_curves.png`.
  * Summary CSV: `evidence/stage2/stage2_snr_robustness.csv`.
- Comprehensive archive: `evidence/stage2/stage2_localization_evidence.zip`.
- Interactive Notebook: `notebooks/Stage2_Acoustic_Localization.ipynb`.

---

## 5. Physical Hardware Verification (Stage 3)
- **Hardware Architecture:** Classic ESP32-WROOM-32 development board.
- **Dual-I2S Clock Synchronization:** Configured simultaneous I2S0 (Master) and I2S1 (Slave) sharing master bit clock (SCK = GPIO 18) and word select (WS = GPIO 21):
  * Pair A (Mics 1 & 2): GPIO 19 (I2S0 SD)
  * Pair B (Mics 3 & 4): GPIO 23 (I2S1 SD)
  * ADXL345: I2C on TX2 (GPIO 17, SDA) and RX2 (GPIO 16, SCL)
  * FC-03 Tachometer: Hardware ISR on D4 (GPIO 4)
- **Clock vs. DMA Queue Alignment:** Shared SCK/WS locks physical sampling clock (0 PPM drift). Sequential `i2s_read` pulls 128-sample DMA blocks. Sub-sample inter-peripheral buffer synchronization is planned for major project characterization.
- **Sample Bit Shift:** Unpacking shifts INMP441 24-bit MSB-aligned data via `>> 16` directly into signed `int16_t` with unity digital gain.
- **Live Link Performance:** Streams 11,025 Hz 4-channel 16-bit PCM + ADXL345 + Tachometer pulses over 921,600 baud serial on COM8 with 16-bit XOR checksum. Zero dropped packets observed under sustained continuous acquisition.
- Firmware source: `stage3_esp32_firmware/ESP32_Spatial_Acoustic_Radar_Firmware/ESP32_Spatial_Acoustic_Radar_Firmware.ino`.

---

## 6. Real-Time Multimodal Sensor Fusion Suite (Stage 4)
- Master application: `stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py` (Desktop launcher: `RUN_LIVE_RADAR.bat`).
- **Explicit Evidence Mode Tag:** Live HUD displays current execution mode (`LIVE 4-MIC ULA PHYSICAL HARDWARE (COM8)`, `SYNTHETIC 4-CH ARRAY SIMULATION`, or `LAPTOP INTERNAL MIC DEMO`).
- **Live 2D Thermal Heatmap:** Backprojects acoustic energy onto a spatial grid at 20 FPS with Direction of Arrival (DOA $\theta$) focus.
- **Dynamic VU Indicators:** Dedicated Left and Right flank intensity rails with square-root scaling and color alerts (Cyan $\to$ Amber $\to$ Red).
- **ISO 10816-Inspired Vibration Severity Index:** Empirical dynamic acceleration grading ($a_{\text{dyn}} = |a_{\text{mag}} - 1.0g|$): Zone A Good, Zone B Permissible, Zone C Warning, Zone D Alarm.
- **Acoustic Kurtosis ($\kappa$) & Crest Factor:** Detects impulsive commutator sparking and bearing micro-impacts ($\kappa > 4.5$).
- **Golden Baseline Differential Engine (Key `'B'`):** Captures reference healthy signature and alerts to real-time deltas ($\Delta\text{Vib}, \Delta\text{RPM}, \Delta\text{Kurt}, \Delta\text{RMS}$) with a continuous Composite Health Index bar ($0\text{--}100\%$).
