# Project Review Digest for External AI Peer Review (ChatGPT Review Pack)
**Project Title:** Spatial Multimodal Edge-AI for Fault Detection and Localization of Rotating Machinery  
**Author / Engineering Team:** Yogi (yoganandasp15) & Antigravity CLI  
**Last Updated:** 7 November 2026 (Minor Project Completion Snapshot)  
**GitHub Repository:** [https://github.com/yoganandasp15/Spatial-accoustic-project](https://github.com/yoganandasp15/Spatial-accoustic-project)  
**Raw Digest URL:** `https://raw.githubusercontent.com/yoganandasp15/Spatial-accoustic-project/main/GPT_REVIEW_PACK.md`

---

## 📋 Instructions for the Reviewing AI (ChatGPT)
> **Role for ChatGPT**: You are acting as an elite adversarial peer reviewer and senior DSP / embedded Edge-AI systems architect.
> Please review the project architecture, mathematical derivations, empirical benchmark evidence, and physical hardware implementation below.
> 
> **Key Review Questions**:
> 1. Are there any hidden signal-processing traps, spatial aliasing risks, or phase-drift vulnerabilities in the 4-microphone ULA design ($d = 20.32\text{ mm}$)?
> 2. Does the multimodal fusion methodology (ISO 10816 vibration + 1x/2x rotational orders + acoustic kurtosis + Golden Baseline differential engine) soundly differentiate between mass unbalance, shaft misalignment, and electrical brush arcing?
> 3. What are the highest-value recommendations and edge cases to investigate as we transition from this minor-project prototype to a standalone embedded major-project architecture (e.g. ESP32-S3 / TinyML)?

---

## ⏱️ Project Evolution Timeline & Engineering Chronicle

### Entry 1 — 3 November 2026: Stage 1 Dataset Exploration & Feasibility Audit
* **Problem Investigated**: Can vibration harmonics ($1\times, 2\times$) reliably differentiate mechanical unbalance from shaft misalignment?
* **Evidence**:
  * Evaluated 9 matched-speed records from the MAFAULDA benchmark across 31.7 Hz, 45.5 Hz, and 53.9 Hz.
  * Verified tachometer rising-edge count vs. spectral peak to within **0.0468%** error.
  * Observed that 10 g rotor mass unbalance surges $1\times$ rotational order energy from 8.8% to **66.6%** in radial vibration.
  * Concluded that $1\times$ energy is a reliable physical indicator of mechanical unbalance, while misalignment elevates $2\times$ order energy across coupling axes.

### Entry 2 — 5 November 2026: Stage 3 ESP32 Dual-I2S Hardware Breakthrough
* **Problem Investigated**: Classic ESP32-WROOM-32 lacks hardware TDM (Time Division Multiplexing). Can it stream 4 synchronized INMP441 microphone channels?
* **Solution**:
  * Implemented simultaneous **Dual-I2S DMA capture** (I2S0 + I2S1) sharing master bit-clock (SCK = GPIO 18) and word-select (WS = GPIO 21).
  * Pair A on GPIO 19 (I2S0 SD) + Pair B on GPIO 23 (I2S1 SD).
  * Integrated ADXL345 on I2C (SDA = TX2/GPIO 17, SCL = RX2/GPIO 16) and FC-03 tachometer on D4 (GPIO 4) with hardware ISR.
  * Streamed 11,025 Hz 4-channel 16-bit PCM + ADXL345 + Tachometer pulses over 921,600 baud serial on COM8 with a 16-bit XOR checksum.
  * **Verified Result**: Zero dropped packets; 79.4 kB/s throughput verified under continuous acquisition.

### Entry 3 — 6 November 2026: Directional Delay Sign Correction & Sensor Fusion
* **Bug Investigated**: Acoustic radar was localizing sound on the opposite flank; right wing gauge stayed at 0% when sound was on the right.
* **Root Cause Proven**:
  * In the far-field planewave steering equation, a physical sound on the right ($\theta > 0$) arrives at Mic 4 ($x = +30.48\text{ mm}$) earlier than Mic 1 ($x = -30.48\text{ mm}$), producing a positive cross-correlation lag ($\tau > 0$).
  * The steering equation had an erroneous negative sign ($\tau = -(d_x/c)\sin\theta$), forcing the algorithm to sample negative lags.
  * Corrected equations to the physical positive convention ($\tau = +(d_x/c)\sin\theta$ and $\tau_{\text{grid}} = +(d_i - d_j)/c$).
* **Left-Side Responsiveness Fix**:
  * Fixed `is_active_sound` gate from hardcoded `avg_rms >= 0.025` to adaptive `avg_rms >= max(0.0020, 1.15 * noise_floor)`.
  * Expanded bandpass filter `FREQ_MIN` from 400 Hz to **80 Hz** to preserve motor rotational harmonics ($1\times, 2\times, 3\times$).

### Entry 4 — 7 November 2026: Stage 2 Acoustic Simulation & Final Minor Review
* **Engineering Completed**:
  * Built complete Stage 2 simulation suite (`stage2_acoustic_localization.py` and `Stage2_Acoustic_Localization.ipynb`).
  * Benchmarked 25 off-grid test positions: achieved **0.34° median angular error** and **100.0% pass rate** ($<5^\circ$).
  * Verified noise robustness: 0.34° median error at 20 dB SNR, 0.55° at 10 dB SNR, 0.40° in reverberant multipath.
  * Packaged 10-slide executive PowerPoint presentation (`PRESENTATION_Spatial_Multimodal_EdgeAI.pptx`) and final report (`MINOR_PROJECT_FINAL_REPORT.md`).

---

## 📐 Mathematical & Algorithmic Formulation

### 1. Super-Resolution Sinc GCC-PHAT
Cross-spectral density between microphone pair $(i, j)$:
$$G_{ij}(f) = S_i(f) \cdot S_j^*(f)$$
Normalized Phase Transform:
$$\Psi_{\text{PHAT}}(f) = \frac{G_{ij}(f)}{|G_{ij}(f)|^\gamma + \epsilon}$$
where $\gamma = 0.70$ (modified generalized PHAT) and $\epsilon = 10^{-4} \max|G_{ij}|$.  
Fine sub-sample lag resolution is achieved via frequency-domain zero-padding by factor $K = 16$:
$$R_{ij}(\tau) = \mathcal{F}^{-1}\left\{\text{pad}_{K}(\Psi_{\text{PHAT}}(f))\right\}$$
followed by parabolic interpolation around the peak.

### 2. 2D Steered Response Power (SRP-PHAT) Acoustic Camera
For any spatial candidate position $\mathbf{x} = (x, y)$ on the 2D floor grid:
$$\tau_{ij}(\mathbf{x}) = \frac{\|\mathbf{x} - \mathbf{m}_i\| - \|\mathbf{x} - \mathbf{m}_j\|}{c}$$
The backprojected spatial acoustic energy is computed by steering all $\binom{4}{2} = 6$ microphone pairs:
$$P(\mathbf{x}) = \sum_{p=1}^6 w_p \cdot \max\left(0, R_{p}\left(\tau_p(\mathbf{x})\right)\right), \quad w_p = \sqrt{\frac{d_{ij}}{d_0}} \cdot \min(\text{prom}_p, 3.0)$$

### 3. Vibration & Rotational Order Diagnostics
* **Dynamic Acceleration**: $a_{\text{dyn}} = \max\left(0, \sqrt{a_x^2 + a_y^2 + a_z^2} - 1.0g\right)$
* **ISO 10816 Severity Classification (Class 1 Machines < 15 kW)**:
  * Zone A (Good): $a_{\text{dyn}} < 0.15g$
  * Zone B (Permissible): $0.15g \le a_{\text{dyn}} < 0.35g$
  * Zone C (Warning): $0.35g \le a_{\text{dyn}} < 0.70g$
  * Zone D (Alarm): $a_{\text{dyn}} \ge 0.70g$
* **Acoustic Kurtosis ($\kappa$) & Crest Factor ($C_f$)**:
  $$\kappa = \frac{\frac{1}{N}\sum_{n=1}^N (x_n - \mu)^4}{\left(\frac{1}{N}\sum_{n=1}^N (x_n - \mu)^2\right)^2}, \quad C_f = \frac{\max|x_n|}{x_{\text{rms}}}$$
  $\kappa \approx 3.0$ indicates nominal Gaussian noise. Values $\kappa > 4.5$ indicate impulsive transients (bearing raceway micro-pitting or electrical brush arcing).

### 4. Golden Baseline Differential Comparator (Key `'B'`)
Calibrates and locks nominal reference state: $\mathbf{\Theta}_0 = \{R_0, a_0, \kappa_0, \mathbf{S}_0\}$.  
Computes live differential deviations:
$$\Delta\text{Vib} = a_{\text{dyn}} - a_0, \quad \Delta\text{RPM} = \text{RPM} - R_0, \quad \Delta\text{Kurt} = \kappa - \kappa_0, \quad \Delta\text{RMS} = \frac{x_{\text{rms}} - x_{0,\text{rms}}}{x_{0,\text{rms}}} \times 100\%$$
Composite Motor Health Index ($0\text{--}100\%$):
$$\text{Health} = \max\left(5, 100 - (\text{Penalty}_{\text{Vib}} + \text{Penalty}_{\text{Kurt}} + \text{Penalty}_{\Delta})\right)$$

---

## 📊 Empirical Benchmark Results

### 1. Stage 2B GCC-PHAT Pairwise Delay Error vs Analytical Ground Truth
| Mic Pair | Distance ($d_{ij}$) | Ground Truth Delay | Estimated Delay | Error ($\mu\text{s}$) | Error (Samples @ 16kHz) |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **M1–M2** | 20.32 mm | +31.08 µs | +31.09 µs | **+0.01 µs** | +0.0002 smp |
| **M1–M3** | 40.64 mm | +61.56 µs | +61.53 µs | **-0.03 µs** | -0.0005 smp |
| **M1–M4** | 60.96 mm | +91.42 µs | +91.57 µs | **+0.15 µs** | +0.0024 smp |
| **M2–M3** | 20.32 mm | +30.48 µs | +30.48 µs | **-0.00 µs** | -0.0000 smp |
| **M2–M4** | 40.64 mm | +60.34 µs | +60.50 µs | **+0.16 µs** | +0.0026 smp |
| **M3–M4** | 20.32 mm | +29.87 µs | +30.05 µs | **+0.19 µs** | +0.0030 smp |

### 2. Stage 2C 25-Point Off-Grid Spatial Localization Summary
* Source Angles: $-65^\circ$ to $+65^\circ$ | Source Distances: $0.65\text{ m}$ to $2.10\text{ m}$
* **Median Angular Error:** **0.34°**
* **Mean Angular Error:** **0.35°**
* **90th-Percentile Error:** **0.68°**
* **Angular Pass Rate ($<5^\circ$):** **100.0%**

### 3. Stage 2D Noise Robustness & Multipath Sweeps
| Test Condition | Median Angular Error | Mean Angular Error | 90th-Percentile Error | Success Rate ($<5^\circ$) |
| :--- | :---: | :---: | :---: | :---: |
| **Clean ($\infty$ dB)** | 0.34° | 0.34° | 0.55° | **100.0%** |
| **20 dB SNR** | 0.34° | 0.34° | 0.61° | **100.0%** |
| **10 dB SNR** | 0.55° | 0.76° | 1.52° | **100.0%** |
| **0 dB SNR** | 2.52° | 3.10° | 5.87° | **80.0%** |
| **Multipath / Reverberation** | 0.40° | 7.37° | 30.54° | **84.0%** |

---

## 🛠️ Physical Breadboard Pinout (Classic ESP32)
* **Master Bit Clock (SCK):** GPIO 18
* **Master Word Select (WS):** GPIO 21
* **Pair A Audio (M1 & M2):** GPIO 19 (I2S0 SD)
* **Pair B Audio (M3 & M4):** GPIO 23 (I2S1 SD)
* **ADXL345 Accelerometer:** SDA = TX2 (GPIO 17), SCL = RX2 (GPIO 16)
* **FC-03 Tachometer ISR:** D4 (GPIO 4)
* **Serial Telemetry:** COM8 @ 921,600 baud, 11,025 Hz 4-ch PCM (79.4 kB/s)
