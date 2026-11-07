# Spatial Multimodal Edge-AI for Fault Detection and Localization of Rotating Machinery
## Final Comprehensive Minor Project Evaluation Report
**Academic Milestone:** ECE Minor Project Evaluation & Demonstration  
**Target Date:** 7 November 2026  
**Hardware Platform:** Classic ESP32 (WROOM-32) + 4-Mic INMP441 Linear Array + ADXL345 Accelerometer + FC-03 Optical Tachometer  

---

## 1. Executive Summary & Problem Formulation

### 1.1 Academic Project Purpose
This project addresses a fundamental limitation in classical industrial condition monitoring: **single-sensor ambiguity**. 
1. **Accelerometers (Vibration)** excel at detecting structural stresses and mechanical defects (e.g. mass unbalance, shaft misalignment), but possess **zero spatial awareness**—they cannot locate an acoustic source across a plant floor or distinguish between internal motor noise and ambient airborne interference.
2. **Microphones (Acoustic)** provide non-contact, high-frequency detection of electrical arcing, bearing impacts, and airflow anomalies, but **an acoustic hotspot is not automatically a mechanical fault**—room chatter, cooling fans, and adjacent machinery create false alarms.

### 1.2 The Core Minor Project Thesis
By simultaneously co-fusing:
* **Chassis dynamic vibration** from an ADXL345 ($a_{\text{dyn}}$),
* **Shaft rotational speed** from an FC-03 optical tachometer ($f_0 = \text{RPM} / 60$), and
* **Spatial acoustic time-difference-of-arrival (TDOA)** from a 4-microphone INMP441 uniform linear array,

the proposed system answers two essential diagnostic questions in real time:
1. **WHAT is wrong with the machine?** (e.g., Mass Unbalance, Shaft Misalignment, Bearing Impact / Brush Arcing, Commutator Drag).
2. **WHERE is the dominant acoustic emission coming from?** (Real-time 2D acoustic camera heatmap and direction-of-arrival bearing angle $\theta \in [-90^\circ, +90^\circ]$).

---

## 2. Experimental Stages & Completed Evidence

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 ECE MINOR PROJECT PIPELINE             │
                  └───────────────────────────┬────────────────────────────┘
                                              │
              ┌───────────────────────────────┼───────────────────────────────┐
              ▼                               ▼                               ▼
       [STAGE 1: VERIFIED]             [STAGE 2: VERIFIED]             [STAGES 3 & 4: VERIFIED]
       Dataset Benchmarks              Acoustic Localization           Hardware & Live Fusion
       - CWRU Bearing                  - 4-Mic ULA Simulation          - Dual-I2S DMA on ESP32
       - MAFAULDA Matched Groups       - Sinc GCC-PHAT TDOA            - 11,025 Hz 4-Ch PCM
       - 1x Unbalance Physics          - 25-Point Off-Grid Test        - Real-Time Heatmap GUI
       - 2x Misalignment Physics       - SNR & Reverb Sweeps           - Golden Baseline Engine
```

### 2.1 Stage 1: Rotational Order Physics on Benchmark Datasets
* **CWRU Benchmark (`evidence/cwru_initial/`):** Evaluated baseline nominal vs. inner-race bearing defect records, establishing spectral feature extraction.
* **MAFAULDA Matched-Speed Experiment (`evidence/stage1b_csvs/`):** Evaluated 9 records across 3 nominal speed groups (31.7 Hz, 45.5 Hz, 53.9 Hz) comparing Normal, 10 g Unbalance, and 1.5 mm Horizontal Misalignment.
* **Empirical Findings:**
  1. **Tachometer Verification:** Tachometer rising-edge counting vs. spectral peak frequency matched to within **0.0468%**, verifying sub-0.05% speed tracking accuracy.
  2. **1× Rotational Order Unbalance Signature:** Radial vibration under 10 g unbalance showed a massive, consistent increase in $1\times$ shaft rotational harmonic energy:
     * 31.7 Hz Group: 8.78% (Normal) $\to$ **40.74%** (Unbalance)
     * 45.5 Hz Group: 8.39% (Normal) $\to$ **61.39%** (Unbalance)
     * 53.9 Hz Group: 10.21% (Normal) $\to$ **66.57%** (Unbalance)
  3. **Misalignment:** Elevated $2\times$ order energy was observed, confirming multi-harmonic generation across mechanical coupling axes.

### 2.2 Stage 2: Acoustic Source Localization & Spatial Beamforming
* **Array Geometry:** 4-Microphone Uniform Linear Array with $d = 20.32\text{ mm}$ spacing ($60.96\text{ mm}$ total span), matching the physical breadboard layout.
* **Pairwise GCC-PHAT TDOA Verification (`evidence/stage2/stage2_tdoa_gcc_phat_verification.png`):**
  * Sinc-interpolated super-resolution delay estimation evaluated across all 6 pairs:
    $$\Delta\tau_{ij,\text{truth}} = \frac{\|\mathbf{x}_{\text{source}} - \mathbf{x}_i\| - \|\mathbf{x}_{\text{source}} - \mathbf{x}_j\|}{c}$$
  * Maximum delay error observed against analytical ground truth: **$<0.19\ \mu\text{s}$** (less than $1/50\text{th}$ of a single audio sample at 16 kHz).
* **25-Point Off-Grid Spatial Benchmark (`evidence/stage2/stage2_localization_benchmarks.csv`):**
  * Evaluated across 25 random off-grid positions spanning $-65^\circ$ to $+65^\circ$ at ranges from $0.65\text{ m}$ to $2.10\text{ m}$.
  * **Median Angular Error:** **0.34°**
  * **Mean Angular Error:** **0.35°**
  * **90th-Percentile Angular Error:** **0.68°**
  * **Angular Success Rate ($<5^\circ$ error):** **100.0% Pass Rate**
* **Noise Robustness & Multipath Sweeps (`evidence/stage2/stage2_snr_robustness.csv`):**
  * **Clean ($\infty$ dB):** Median Error = 0.34°, 90th% = 0.55°, Pass Rate = 100.0%
  * **20 dB SNR:** Median Error = 0.34°, 90th% = 0.61°, Pass Rate = 100.0%
  * **10 dB SNR:** Median Error = 0.55°, 90th% = 1.52°, Pass Rate = 100.0%
  * **0 dB SNR:** Median Error = 2.52°, 90th% = 5.87°, Pass Rate = 80.0%
  * **Multipath / Reverberation (2 specular wall reflections, $\beta=0.35$):** Median Error = 0.40°, Pass Rate = 84.0%

### 2.3 Stage 3: Classic ESP32 Dual-I2S Acquisition Architecture
* **Overcoming Classic ESP32 Hardware Constraints:**
  * The ESP32-WROOM-32 lacks hardware TDM (Time Division Multiplexing).
  * **Solution:** Configured simultaneous **Dual-I2S DMA engines** (I2S0 and I2S1) sharing master bit clock (SCK = GPIO 18) and word select (WS = GPIO 21).
  * **Pair A (Mics 1 & 2):** GPIO 19 (I2S0 SD)
  * **Pair B (Mics 3 & 4):** GPIO 23 (I2S1 SD)
  * **ADXL345 Accelerometer:** Hardware I2C (SDA = TX2/GPIO 17, SCL = RX2/GPIO 16)
  * **FC-03 Optical Tachometer:** Hardware GPIO interrupt on D4 (GPIO 4)
* **DMA Streaming Link:** Streams 11,025 Hz 4-channel 16-bit PCM + ADXL345 + Tachometer pulses over 921,600 baud serial with 16-bit XOR packet checksum.
* **Observed Link Performance:** Zero dropped packets; 79.4 kB/s throughput verified under continuous operation.

### 2.4 Stage 4: Real-Time Multimodal Fusion Engine (`src/spatial_acoustic_radar_pro.py`)
* **Live 2D Thermal Heatmap:** Backprojects acoustic energy onto a $60 \times 60$ floor grid at 20 FPS using bicubic interpolation.
* **Directional Intensity Rails:** Real-time Left / Center / Right VU meters with perceptual square-root scaling and color transitions (Cyan $\to$ Amber $\to$ Red).
* **ISO 10816 Standard Compliance:** Dynamic vibration severity grading ($a_{\text{dyn}} = |a_{\text{mag}} - 1.0g|$):
  * Zone A: Good / Smooth ($<0.15g$)
  * Zone B: Permissible ($0.15\text{--}0.35g$)
  * Zone C: Warning ($0.35\text{--}0.70g$)
  * Zone D: Alarm ($>0.70g$)
* **Acoustic Kurtosis ($\kappa$) & Crest Factor:** Detects non-Gaussian impulsive spiking from electrical brush arcing and bearing raceway micro-pitting.
* **Golden Baseline Differential Engine (Key `'B'`):**
  * Calibrates and locks healthy nominal reference values ($\text{RPM}_0, a_0, \kappa_0, \mathbf{S}_0$).
  * Real-time delta comparator ($\Delta\text{Vib}, \Delta\text{RPM}, \Delta\text{Kurt}, \Delta\text{RMS}$) detects microscopic degradation (e.g. slight shaft drag, loose mount screw) before human hearing can discern it.
  * Displays a live Composite Health Index bar ($0\text{--}100\%$).

---

## 3. Package Verification & File Manifest

| File / Artifact | Location | Purpose & Verification |
| :--- | :--- | :--- |
| **`stage2_acoustic_localization.py`** | `src/` | Full simulation suite for Stages 2A-2D; verified with exit code 0 |
| **`Stage2_Acoustic_Localization.ipynb`** | `notebooks/` | Guided cell-by-cell Jupyter notebook for Colab / interactive review |
| **`spatial_acoustic_radar_pro.py`** | `src/` | Master interactive GUI application with multimodal fusion HUD |
| **`RUN_LIVE_RADAR.bat`** | Project Root | One-click desktop launcher for the live interactive radar application |
| **`ESP32_Spatial_Acoustic_Radar_Firmware.ino`** | `firmware/` | Complete production firmware for ESP32 on COM8 @ 921,600 baud |
| **`stage2_localization_evidence.zip`** | `evidence/stage2/` | Comprehensive benchmark CSVs and high-res PNG diagnostic plots |
| **`PRESENTATION_Spatial_Multimodal_EdgeAI.pptx`**| Project Root | 10-slide executive PowerPoint presentation for 7 Nov review |
| **`Minor_project_topic_1.pdf`** | `references/` | Original approved minor/major project scope and architecture document |

---

## 4. Minor vs. Major Project Roadmap

### 4.1 Minor Project Scope (Achieved & Demonstrated)
* Safe physical prototype on a low-voltage DC motor.
* Synchronous dual-I2S 4-channel audio DMA acquisition demonstrated on classic ESP32.
* Laptop-assisted super-resolution signal processing (Sinc GCC-PHAT, SRP-PHAT 2D heatmaps).
* Multimodal sensor fusion establishing both **"What is wrong?"** and **"Where is the acoustic source?"**.

### 4.2 Major Project Future Scope (Next Academic Year)
* **Standalone Embedded Edge-AI:** Port feature extraction and lightweight neural inference (1D-CNN / TinyML Transformer) directly to an ESP32-S3 or Edge TPU.
* **Industrial Galvanic Isolation:** Interface with industrial-grade 4-20mA and IEPE accelerometers for high-voltage plant machinery.
* **Longitudinal Degradation Tracking:** Conduct continuous multi-day run-to-failure testing to train Remaining Useful Life (RUL) predictive maintenance models.

---

## 5. Instructions for Physical Demonstration

1. **Launch the Live Application:**
   Double-click `RUN_LIVE_RADAR.bat` in this folder (or `C:\Users\Yogi\Desktop\Launch_Spatial_Acoustic_Radar.bat`).
2. **Directional Acoustic Check:**
   Make sound on the left and right sides of the breadboard. If directions appear inverted relative to your perspective, press **`'I'`** once to flip channel polarity.
3. **DC Motor Fault Demonstration:**
   * Run the DC motor unloaded and press **`'B'`** to lock the **Golden Baseline**.
   * Lightly touch the shaft to introduce mechanical friction: observe the instantaneous drop in RPM, rise in audio RMS, and decrease in the Health Index.
   * Attach an off-axis mass to demonstrate **Mass Unbalance**: observe the $1\times$ order surge and ISO 10816 Zone C Warning.
   * Press **`'P'`** to cycle tachometer pulses per revolution if using an optical encoder wheel.
