# Physical Motor Experiment Report: Healthy vs. Defective Condition
**Project:** Spatial Multimodal Edge-AI for Fault Detection and Localization of Rotating Machinery  
**Author:** Yogi (ECE Minor Project)  
**Date:** 2026-11-09 11:55:34  
**Evaluation Standard:** ISO 10816-1 / Rotational Order Tracking  

---

## 1. Experimental Overview
This physical experiment validates the primary thesis of the minor project: establishing a quantitative physical baseline for rotating machinery fault detection using synchronized 3-axis chassis vibration (ADXL345 at 400 Hz), optical shaft tachometry (FC-03), and 4-channel acoustic beamforming (INMP441 array).

Two motor conditions were tested under identical mounting constraints:
1. **Healthy Motor**: Known nominal DC motor running at rated shaft speed.
2. **Defective Motor**: Test motor exhibiting mechanical eccentricity and mass unbalance.

---

## 2. Quantitative Experimental Results

| Metric | Healthy Motor | Defective Motor | Measured Delta (Δ) | Diagnostic Interpretation |
| :--- | :---: | :---: | :---: | :--- |
| **Mean Shaft Speed** | `1800.0 RPM` | `1764.9 RPM` | `-35.1 RPM` | Nominal operating speed maintained |
| **Rotational Fund. ($f_0$)** | `30.0 Hz` | `29.4 Hz` | `-0.6 Hz` | Fundamental shaft frequency |
| **Dynamic Vibration ($a_{\text{dyn}}$ RMS)** | `0.0106 g` | `0.0648 g` | `++508.4%` | **Elevated Chassis Dynamic Vibration** |
| **Peak Dynamic Acceleration** | `0.0367 g` | `0.1827 g` | `++398.3%` | Transient peak amplitude |
| **ISO 10816-1 Severity Zone** | **Zone A: Good / Smooth** | **Zone C: Warning** | Zone Shift (A → C) | Exceeds permissible vibration limits |
| **ISO Velocity RMS ($v_{\text{rms}}$)** | `0.46 mm/s` | `3.62 mm/s` | `++3.16 mm/s` | Integrated velocity in 10-200 Hz band |
| **1× Shaft Order Energy ($e_{1x}$)** | `0.1%` | `3.7%` | `++3.7%` | **Primary Mass Unbalance Signature** |
| **2× Harmonic Order Energy ($e_{2x}$)** | `0.3%` | `2.0%` | `+1.7%` | Misalignment indicator |
| **Vibration Crest Factor** | `3.45` | `2.82` | `-0.62` | Non-Gaussian impact measure |
| **Vibration Kurtosis** | `2.86` | `2.21` | `-0.65` | Peakedness / shock distribution |

---

## 3. Physical Diagnosis & Findings

1. **Mass Unbalance Confirmation**:
   The defective motor exhibits a **+3.7% elevation** in fundamental shaft rotational order energy ($1\times f_0$). Under classical vibration mechanics (ISO 10816), unbalance generates a sinusoidal dynamic force proportional to $\omega^2$, manifesting as a dominant $1\times$ peak in the order spectrum.
2. **Dynamic Severity Escalation**:
   Chassis dynamic acceleration increased by **+508.4%**, shifting the motor state from **Zone A (Good / Smooth)** to **Zone C (Warning)**.
3. **Multimodal Disturbance Rejection (PDF Page 13 Proof)**:
   In the presence of loud external acoustic noise bursts (speech / claps), the acoustic array registers elevated acoustic pressure; however, the chassis accelerometer remains stable in Zone A ($a_{\text{dyn}} < 0.15\text{g}$). The multimodal fusion engine correctly rejects false mechanical alarms, outputting:
   `[DIAGNOSED FAULT] ACOUSTIC DISTURBANCE DETECTED (Vibration Nominal -> Non-Mechanical)`

---

## 4. Evidence Artifacts
- **Vibration Comparison Figure**: `docs/evidence/physical_motor_vibration_comparison.png`
- **Disturbance Rejection Figure**: `docs/evidence/multimodal_acoustic_disturbance_rejection.png`
- **Healthy Motor Run CSV**: `data/experiments/motor_healthy_run.csv`
- **Defective Motor Run CSV**: `data/experiments/motor_defective_run.csv`
- **Summary JSON**: `data/experiments/motor_experiment_summary.json`
