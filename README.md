# Spatial Multimodal Edge-AI for Fault Detection and Localization of Rotating Machinery

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform: ESP32 + Python](https://img.shields.io/badge/Platform-ESP32%20%7C%20Python%203.14-green.svg)](docs/HARDWARE_STATUS.md)
[![Status: Minor Complete](https://img.shields.io/badge/Status-Minor%20Complete%20(7%20Nov%202026)-success.svg)](docs/PROJECT_STATUS.md)

An end-to-end multimodal condition monitoring suite combining a **4-microphone INMP441 uniform linear array**, an **ADXL345 3-axis accelerometer**, and an **FC-03 optical tachometer** to simultaneously answer two questions for rotating machinery:
1. **WHAT is wrong with the motor?** (Mass Unbalance, Shaft Misalignment, Bearing Impact / Brush Arcing, Commutator Drag).
2. **WHERE is the dominant acoustic sound coming from?** (Real-time 2D acoustic camera heatmap and direction-of-arrival bearing angle $\theta \in [-90^\circ, +90^\circ]$).

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
                    (RPM, Jitter)    (ISO 10816)      Spatial DOA
                         │                  │              │
                         └──────────┬───────┴──────────────┘
                                    ▼
                    ┌────────────────────────────────────────┐
                    │    MULTIMODAL MOTOR FUSION ENGINE      │
                    │  - Real-time Health Index (0 - 100%)   │
                    │  - Golden Baseline Delta Comparator    │
                    │  - Harmonic Order Separation (1x, 2x)  │
                    │  - 2D Thermal Acoustic Camera Heatmap  │
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
│   └── ESP32_Spatial_Acoustic_Radar_Firmware/   # 11,025 Hz 4-ch DMA streaming on COM8
│
├── stage4_realtime_fusion_radar/                # STAGE 4: Real-time GUI & Golden Baseline
│   └── src/spatial_acoustic_radar_pro.py        # Master interactive radar application
│
└── hardware_photos/                             # Physical hardware photos & wiring
```

---

## 📊 Summary of Completed Evidence

| Stage | Focus Area | Status | Key Metric / Observed Evidence |
| :--- | :--- | :---: | :--- |
| **Stage 1** | Rotational Orders & Vibration | **VERIFIED** | 10g unbalance surges $1\times$ order energy to 66.6% in MAFAULDA |
| **Stage 2** | Acoustic Localization (25 Pts) | **VERIFIED** | **0.34° median error**; **100.0% pass rate** (<5° error threshold) |
| **Stage 2D**| Noise Robustness & Reverberation | **VERIFIED** | Sub-degree accuracy down to 10 dB SNR; 80% pass at 0 dB SNR |
| **Stage 3** | ESP32 Hardware Acquisition | **VERIFIED** | 11,025 Hz 4-ch DMA PCM @ 921,600 baud with 0 dropped packets |
| **Stage 4** | Multimodal Motor Health HUD | **VERIFIED** | ISO 10816 grading, Kurtosis $\kappa$, Golden Baseline (Key `'B'`) |

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
