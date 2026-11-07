# Spatial Multimodal Edge-AI for Fault Detection and Localization of Rotating Machinery

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform: ESP32 + Python](https://img.shields.io/badge/Platform-ESP32%20%7C%20Python%203.14-green.svg)](docs/HARDWARE_STATUS.md)
[![Status: Minor Evaluation Ready](https://img.shields.io/badge/Status-Minor%20Evaluation%20Ready%20(7%20Nov%202026)-success.svg)](docs/PROJECT_STATUS.md)

An end-to-end multimodal condition monitoring suite combining a **4-microphone INMP441 uniform linear array**, an **ADXL345 3-axis accelerometer**, and an **FC-03 optical tachometer** to simultaneously answer two questions for rotating machinery:
1. **WHAT is wrong with the motor?** (Mass Unbalance, Shaft Misalignment, Bearing Impact / Brush Arcing, Commutator Drag).
2. **WHERE is the dominant acoustic sound coming from?** (Real-time Direction of Arrival bearing angle $\theta \in [-90^\circ, +90^\circ]$ and 2D acoustic camera heatmap).

---

## 🏗️ System Architecture

```
                    ┌────────────────────────────────────────┐
                    │            DC MOTOR UNDER TEST         │
                    └────┬──────────────────┬──────────────┬─┘
                         │                  │              │
                   Shaft Rotation        Chassis        Acoustic
                   Interrupter Disc     Vibration       Emission
                         │                  │              │
                         ▼                  ▼              ▼
                   ┌───────────┐      ┌───────────┐  ┌───────────┐
                   │   FC-03   │      │  ADXL345  │  │  INMP441  │
                   │ Tachometer│      │   3-Axis  │  │ 4-Mic ULA │
                   │  (GPIO 4) │      │ (TX2/RX2) │  │(I2S0/I2S1)│
                   └─────┬─────┘      └─────┬─────┘  └─────┬─────┘
                         │                  │              │
                         ▼                  ▼              ▼
                    Shaft Speed       Dynamic Vib      Kurtosis &
                    (RPM, Jitter)    (ISO-Inspired)   Spatial DOA
                         │                  │              │
                         └──────────┬───────┴──────────────┘
                                    ▼
                    ┌────────────────────────────────────────┐
                    │    MULTIMODAL MOTOR FUSION ENGINE      │
                    │  - Real-time Health Index (0 - 100%)   │
                    │  - Golden Baseline Delta Comparator    │
                    │  - Harmonic Order Separation (1x, 2x)  │
                    │  - 2D Acoustic Camera Heatmap (DOA θ)  │
                    └────────────────────────────────────────┘
```

---

## 📁 Repository Organization

```
Spatial-accoustic-project/
├── GPT_REVIEW_PACK.md                           # Single-file digest tailored for ChatGPT review
├── MINOR_PROJECT_FINAL_REPORT.md                # Full academic minor project evaluation report
├── PRESENTATION_Spatial_Multimodal_EdgeAI.pptx  # 10-slide executive PowerPoint presentation
├── RUN_LIVE_RADAR.bat                           # One-click desktop application launcher
│
├── docs/                                        # Architecture & Project Documentation
│   ├── PROJECT_OVERVIEW.md                      # Problem formulation & sensor modalities
│   ├── PROJECT_STATUS.md                        # Master evidence matrix (Stages 1-4)
│   ├── HARDWARE_STATUS.md                       # Verified breadboard wiring & pinout
│   ├── NEXT_STEPS.md                            # Student checklist & Major-project roadmap
│   ├── daily_logs/                              # Date-wise development and experiment logs
│   ├── reviews/                                 # AI & Peer review transcripts and audits
│   └── references/Minor_project_topic_1.pdf     # Approved academic proposal PDF
│
├── stage1_vibration_and_tachometer/             # STAGE 1: Dataset physics & baseline findings
│   └── evidence/                                # CWRU & MAFAULDA matched-group CSVs
│
├── stage2_acoustic_localization/                # STAGE 2: Simulation & spatial beamforming
│   ├── src/stage2_acoustic_localization.py      # 25-point benchmark & SNR sweep script
│   ├── notebooks/Stage2_Acoustic_Localization.ipynb # Guided cell-by-cell Jupyter notebook
│   └── evidence/                                # Error distributions, CDFs, and heatmaps
│
├── stage3_esp32_firmware/                       # STAGE 3: Production embedded firmware
│   └── ESP32_Spatial_Acoustic_Radar_Firmware/   # 11,025 Hz 4-ch DMA streaming on COM8 (>>16)
│
├── stage4_realtime_fusion_radar/                # STAGE 4: Real-time GUI & Golden Baseline
│   └── src/spatial_acoustic_radar_pro.py        # Master interactive radar application
│
└── hardware_photos/                             # Physical hardware photos & wiring
```

---

## 📊 Summary of Completed Evidence & Scientific Tiers

To ensure complete academic integrity, project evidence is strictly separated into four empirical tiers:

| Tier | Modality / Stage | Evidence Source | Status | Key Metric / Observed Ground Truth |
| :--- | :--- | :--- | :---: | :--- |
| **Tier 1** | Machine Fault Physics (Stage 1) | **MAFAULDA & CWRU Datasets** | **VERIFIED** | 10g unbalance surges $1\times$ order energy to 66.6% (vs 8.4% Normal); tachometer matches spectrum within 0.047% |
| **Tier 2** | Acoustic Localization (Stage 2) | **Numerical Simulation Proof**| **VERIFIED** | **0.34° median error** across 25 off-grid test points; 100% pass rate (<5°); robust down to 10 dB SNR |
| **Tier 3** | Embedded Hardware (Stage 3) | **Classic ESP32 Prototype** | **VERIFIED** | Dual-I2S clock-locked 11,025 Hz 4-ch PCM (`>>16`), ADXL345 I2C ($a_z \approx -0.82g$), FC-03 tachometer on COM8 @ 921,600 baud |
| **Tier 4** | Multimodal Fusion HUD (Stage 4) | **Live Interactive GUI** | **VERIFIED** | Live 2D acoustic camera (DOA $\theta$ focus), ISO-inspired severity, Kurtosis $\kappa$, and Golden Baseline (Key `'B'`) |

---

## ⚡ Quick Start

### 1. Run the Live Interactive Application
Connect the ESP32 to USB (port `COM8`) and launch:
```cmd
RUN_LIVE_RADAR.bat
```
*(Or run `python stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py`)*

### 2. Run the Stage 2 Localization Benchmark Suite
```powershell
python stage2_acoustic_localization/src/stage2_acoustic_localization.py
```

### 3. Open the PowerPoint Presentation
Launch `PRESENTATION_Spatial_Multimodal_EdgeAI.pptx` in Microsoft PowerPoint for the project evaluation review.
