# Stage 1: Vibration & Rotational Order Physics
**Investigation Date:** 3 November 2026

## Objective
Verify the relationship between shaft rotational frequency ($f_0 = \text{RPM} / 60$) and mechanical vibration harmonics ($1\times, 2\times$) using standard benchmark datasets before building physical hardware.

## Evidence & Data Sources
1. **CWRU Bearing Dataset (`evidence/cwru_initial/`):**
   - Verified FFT and statistical feature extraction (RMS, peak, crest factor, kurtosis) on inner-race defects vs. healthy baselines.
2. **MAFAULDA Matched-Speed Dataset (`evidence/stage1b_csvs/`):**
   - 9 real machine recordings across 3 nominal speed groups (31.7 Hz, 45.5 Hz, 53.9 Hz) comparing Normal, 10 g Unbalance, and 1.5 mm Horizontal Misalignment.
   - **Tachometer Verification:** Agreement between optical pulse edge-counting and spectral peak frequency matched to within **0.0468%**, validating internal speed estimation.
   - **Rotor Mass Unbalance:** Proved that 10 g unbalance causes a dramatic surge in $1\times$ rotational order energy (from 8.8% to 66.6% in UB-radial vibration).
   - **Shaft Misalignment:** Showed elevated $2\times$ harmonic components across coupling axes.
