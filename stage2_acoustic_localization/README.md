# Stage 2: Acoustic Source Localization & Spatial Beamforming
**Investigation & Benchmark Date:** 7 November 2026

## Objective
Implement and benchmark super-resolution Time-Difference-of-Arrival (TDOA) and 2D Steered Response Power (SRP-PHAT) beamforming algorithms on a 4-microphone Uniform Linear Array ($d = 20.32\text{ mm}$, total aperture $60.96\text{ mm}$) matching the physical breadboard.

## Contents
* **`src/stage2_acoustic_localization.py`**: Complete simulation and verification suite (Stages 2A–2D).
* **`notebooks/Stage2_Acoustic_Localization.ipynb`**: Guided cell-by-cell Jupyter notebook.
* **`evidence/`**: Benchmark CSVs, error distribution plots, and archived evidence.

## Empirical Benchmark Results
1. **Pairwise GCC-PHAT TDOA Verification (Stage 2B):**
   - Maximum delay error against analytical ground truth: **$<0.19\ \mu\text{s}$** across all 6 microphone pairs ($<1/50\text{th}$ sample at 16 kHz).
2. **25-Point Off-Grid Spatial Benchmark (Stage 2C):**
   - Evaluated across 25 random off-grid source locations spanning $-65^\circ$ to $+65^\circ$ at ranges from 0.65 m to 2.10 m.
   - **Median Angular Error:** **0.34°**
   - **Mean Angular Error:** **0.35°**
   - **90th-Percentile Error:** **0.68°**
   - **Success Rate ($<5^\circ$ threshold):** **100.0% Pass Rate**
3. **Noise Robustness & Multipath Sweeps (Stage 2D):**
   - **Clean ($\infty$ dB):** 0.34° median error (100% pass)
   - **20 dB SNR:** 0.34° median error (100% pass)
   - **10 dB SNR:** 0.55° median error (100% pass)
   - **0 dB SNR:** 2.52° median error (80% pass)
   - **Multipath (2 wall reflections, $\beta=0.35$):** 0.40° median error (84% pass)
