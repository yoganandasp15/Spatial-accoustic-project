# Project Status and Evidence Log

**Status snapshot:** 8 November 2026. **Post-Review Remediation & Multi-Agent Milestone 1–4 Verification.**

This document records the verified empirical evidence, implemented stages, architectural remediations, and hardware validation status across the entire Spatial Acoustic Radar pipeline. It maintains strict academic and engineering distinctions between **verified algorithmic/synthetic functionality** and **pending physical testbench experiments**.

---

## 1. Overview of Project Completion & Evidence Tiers

| Subsystem / Focus Area | Evidence Tier | Status | Key Evidence / Artifact |
| :--- | :---: | :---: | :--- |
| **Stage 1A: CWRU Dataset Benchmark** | Open Dataset | **VERIFIED** | `evidence/cwru_initial/cwru_initial_results.zip` (Inner-race bearing faults) |
| **Stage 1B: MAFAULDA Matched Groups** | Open Dataset | **VERIFIED** | `evidence/stage1b_csvs/` (Normal, 10g Imbalance, 1.5mm Misalignment; $1\times$ unbalance surge 40.7%–66.6%) |
| **Stage 2: Acoustic Source Localization** | Numerical Sim | **VERIFIED** | `evidence/stage2/` (25-point benchmark: 0.34° median error; 100% angular pass rate down to 10 dB SNR) |
| **Stage 3: ESP32 Hardware Acquisition** | Physical Bringup | **VERIFIED** | Dual-I2S DMA (I2S0+I2S1 shared SCK/WS clocks), ADXL345 I2C (DEVID 0xE5), FC-03 tachometer ISR (COM8 @ 1.152M baud) |
| **Milestone 1: DSP Pipeline Corrections** | Synthetic & Unit Tests | **VERIFIED** | `calibrate_i2s_phase.py`, `order_tracking.py`, `offline_multimodal_evaluator.py`, `spatial_acoustic_radar_pro.py` |
| **Milestone 2: 400 Hz FIFO & Type 0x05** | Firmware & Receiver Tests | **VERIFIED** | `ESP32_Spatial_Acoustic_Radar_Firmware.ino` (FIFO_CTL 0x80), Type 0x05 framing (16-bit XOR checksum, 1.152M baud) |
| **Milestone 3: Automated Unit Test Suite** | Anti-Tautology Harness | **VERIFIED** | 44 tests passing repository-wide (100% mutation kill rate on 6 anti-tautology mutants, 0 failures, 0 errors) |
| **Milestone 4: Log Reconciliation & Docs** | Repository Governance | **VERIFIED** | Reconciled `AGENT_SYNCHRONIZATION_LOG.md`, `PROJECT_STATUS.md`, and `NEXT_STEPS.md` |
| **Physical Motor Unbalance & Acoustic Rig**| Physical Testbench | **PENDING** | Requires continuous 400 Hz motor run-up, tachometer jitter pulse trains, and calibrated room acoustic sweeps |

---

## 2. Verified Algorithmic & Synthetic Functionality

The following software, firmware, and signal processing components have been fully remediated, analytically proven, and verified through automated unit tests, adversarial stress tests, and mutation suites:

### 2.1 Dual-I2S Phase Calibration & Sub-Sample Fractional Delay
- **Cross-Correlation Convention:** Standardized cross-spectral density calculation to $X_1(f) X_2^*(f)$ in `stage2_acoustic_localization/src/calibrate_i2s_phase.py`, ensuring consistent phase sign interpretation across all acoustic algorithms.
- **Angle-Dependent Skew Harmonization:** Corrected the geometric incident angle subtraction so that physical inter-pair DMA clock skew is accurately isolated independently of incident sound arrival angle ($\theta \in [-60^\circ, +60^\circ]$ verified with $<0.0011\ \mu\text{s}$ variance).
- **Physical Compensation Delay:** Stored physical delay compensation in seconds ($+24.69\ \mu\text{s}$ at $44.1\text{ kHz}$) within `i2s_phase_calibration.json`, eliminating integer-sample truncation.
- **Frequency-Domain Fractional Delay Compensation:** Implemented phase compensation filter $H(f) = \exp(-j 2\pi f \tau_{\text{comp}})$ in `compute_srp_phat_grid()` and downstream consumers. Verified across sample rates (11.025 kHz, 16 kHz, 44.1 kHz, 48 kHz, up to 192 kHz) with $<0.1\ \mu\text{s}$ residual skew and strictly unitary gain ($1.0000 \pm 0.0001$).

### 2.2 Dynamic Signed Vibration & Per-Axis Gravity Decoupling
- **Signed Component Preservation:** Eliminated full-wave Euclidean magnitude rectification ($\sqrt{x^2 + y^2 + z^2}$) from vibration processing pipelines. Full-wave rectification was empirically proven to crush dynamic vibration amplitudes by $80\times$ and double the perceived fundamental vibration frequency ($25\text{ Hz} \to 50\text{ Hz}$).
- **Per-Axis Gravity DC Decoupling:** Implemented per-axis running DC gravity estimation ($a_{i,\text{dyn}} = a_i - \bar{g}_i$) with exponential filtering ($\alpha = 0.05$). Preserves signed lateral and vertical oscillations ($a_x, a_y, a_z$), maintaining true frequency harmonics and peak-to-peak excursions.
- **Decoupled GUI Ring Buffer:** Integrated high-speed background thread-safe ring buffer in `stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py`, completely decoupling serial accelerometer sample ingestion from the 20 FPS GUI rendering timer.

### 2.3 Tachometer Revolution Tracking & Absent-Pulse Handling
- **Exact Revolution Interval Counting:** Fixed revolution counting in `stage1_vibration_and_tachometer/src/order_tracking.py` to count exactly $N-1$ intervals across $N$ marker timestamps, spanning $(N-1) \times 2\pi$ total radians. This eliminated a $20\%$ systematic order scaling distortion present in $N$-interval implementations.
- **Elimination of Synthetic RPM Fallback:** Completely eliminated the legacy silent substitution of 1500 RPM / 25 Hz when tachometer pulses are missing. When pulses are absent ($<2$ pulse edges or analog dynamic range $<0.5\text{ V}$), the engine reports `status: "UNAVAILABLE"` with zeroed orders and explicit warning diagnostics.

### 2.4 Continuous 400 Hz ADXL345 Stream FIFO Firmware & Protocol
- **ADXL345 Hardware Configuration:** Configured ADXL345 in 32-sample Stream FIFO mode (`FIFO_CTL = 0x80`) at 400 Hz Output Data Rate (ODR, `BW_RATE = 0x0C`) and $\pm 2g$ full resolution (`DATA_FORMAT = 0x08`) in `stage3_esp32_firmware/ESP32_Spatial_Acoustic_Radar_Firmware/ESP32_Spatial_Acoustic_Radar_Firmware.ino`.
- **Burst FIFO Ingestion:** Firmware reads available FIFO depth per audio block (typically 4–5 samples, up to 8 samples) and bundles them into the binary serial packet with single-sample fallback if FIFO is empty.
- **Type 0x05 Binary Packet Protocol:**
  * 16-byte header: `0xAA 0x55` sync, packet type `0x05`, channels `4`, 16-bit sequence counter, sample rate `11025 Hz`, block frames `128`, `num_vib` (1–32), tachometer pulse counter.
  * Accelerometer payload: `num_vib * 6` bytes (signed 16-bit $X, Y, Z$ milli-$g$).
  * Audio payload: $128 \times 4 \times 2 = 1024$ bytes (4-channel signed 16-bit PCM).
  * 16-bit XOR Checksum: Initialized to `0x55AA` calculated across all preceding header, vibration, and audio bytes.
- **Serial Baud Rate Upgrade:** Upgraded primary UART communication from 921,600 baud to 1,152,000 baud. Guarantees $24.7\%$ bandwidth headroom ($92.3\text{ kB/s}$ required throughput vs $115.2\text{ kB/s}$ channel capacity), completely preventing UART buffer overflow under continuous 400 Hz multi-sample streaming.
- **Receiver Robustness & Timestamp Monotonicity:** Serial receiver validates header contract bounds (`channels == 4`, `block_frames == 128`, `num_vib <= 32`), handles sequence roll-over ($65535 \to 0$), validates defensive buffer shapes, and enforces strictly monotonic vibration timestamps ($t_{\text{start}} = \max(t_{\text{start}}, t_{\text{last}} + \Delta t)$) across batched packet arrivals.

### 2.5 Offline Evaluator & Spatial Display Nomenclature
- **Dynamic Severity Index Rebranding:** In `stage4_realtime_fusion_radar/src/offline_multimodal_evaluator.py`, removed all invalid claims of ISO 10816-3 compliance. Rebranded metric to **Dynamic Vibration Severity Index** ($a_{\text{dyn}}$ in $g$ pk-pk / RMS), clarifying that ISO 10816-3 requires calibrated velocity RMS (mm/s) over 10–1000 Hz with machine support classification.
- **1D ULA Spatial Ray Backprojection:** Explicitly documented that for a 1D Uniform Linear Array, Direction of Arrival (DOA bearing $\theta \in [-90^\circ, +90^\circ]$) is the true physical observable. 2D grid energy peaks along the steered angle represent radial backprojection rays rather than measured physical distance.

---

## 3. Automated Test Suite & Anti-Tautology Verification

The repository contains an expanded, robust test harness validating algorithmic correctness, protocol security, and edge-case resilience:

| Test Module | Test Focus | Tests | Status |
| :--- | :--- | :---: | :---: |
| `tests/test_multimodal_pipeline.py` | Signed vibration preservation, XOR checksum bit-flip rejection, fractional delay phase compensation across sample rates, $N-1$ tachometer intervals, absent-pulse UNAVAILABLE reporting, SRP-PHAT steering. | 15 (1 skipped) | **PASS** |
| `tests/test_m1_adversarial_stress.py` | Incident angle invariance ($-60^\circ$ to $+60^\circ$), fractional delay all-pass amplitude, dynamic vibration vs rectification comparison, tachometer interval counting ($N=2..50$). | 4 | **PASS** |
| `tests/test_m2_firmware_protocol.py` | UART bandwidth headroom calculation, single-bit flip rejection, default baudrate 1.152M, legacy Type 0x04 backward compatibility, sequence counter tracking, stream garbage resynchronization, Type 0x05 packet unpacking. | 7 | **PASS** |
| `tests/test_m2_stress_adversarial.py` | Bit-flip fuzzing across all byte positions, concurrent read/write stress, fragmented stream reconstruction (1B–1024B), inter-packet garbage & split sync, sequence roll-over ($65535 \to 0$), payload boundaries (num_vib 0, 8, 32), and batched timestamp monotonicity. | 9 | **PASS** |
| `tests/test_m3_mutation_adversarial.py` | Anti-tautology mutation tests injecting deliberate bugs: Euclidean magnitude rectification, zero checksum seed, parser bypass, phase compensation sign inversion, $N$ vs $N-1$ intervals, synthetic tachometer fallback; plus extreme sample rate boundaries (8k–192k Hz) and pulse boundaries. | 9 | **PASS** |
| **Total Test Coverage** | **Repository-wide test execution via `RUN_UNIT_TESTS.bat`** | **44 (1 skipped)** | **100% PASS** |

### Anti-Tautology Mutation Results
Every test in the test suite was verified to be strictly non-tautological by injecting 6 deliberate algorithmic mutations:
1. *Euclidean Rectification Mutation:* Correctly caught by 4 physical assertions (doubled frequency, 80x crushed amplitude, lack of second harmonic, asymmetric excursion).
2. *Checksum Zero Seed Mutation:* Correctly caught by seed validation assertion.
3. *Corrupted Packet Bypass Mutation:* Correctly caught by bit-flip corruption rejection assertions across 11 byte positions.
4. *Phase Compensation Sign Inversion Mutation:* Correctly caught by sub-sample delay residual error assertion ($49.38\ \mu\text{s}$ error vs $<0.1\ \mu\text{s}$ threshold).
5. *$N$ Intervals Mutation:* Correctly caught by $1\times$ order peak shift ($0.80$ vs $1.000$) and total revolutions assertions.
6. *Synthetic Tachometer Fallback Mutation:* Correctly caught by all 6 absence assertions.
**Mutation Kill Rate:** 6/6 mutants killed (100%).

### Firmware Compilation Status
- Integrated firmware `stage3_esp32_firmware/ESP32_Spatial_Acoustic_Radar_Firmware/ESP32_Spatial_Acoustic_Radar_Firmware.ino` compiles cleanly with **0 errors and 0 warnings** under `arduino-cli compile --fqbn esp32:esp32:esp32`.
- Program storage: 317,732 bytes (24% of 1.31 MB max).
- Dynamic memory: 27,316 bytes (8% of 327 kB max).

---

## 4. Prior Open Dataset & Simulation Benchmarks

### 4.1 CWRU Bearing Dataset (Stage 1A)
- Verified spectral feature extraction on healthy versus inner-race-bearing recordings (`97.mat`, `98.mat`, `105.mat`).
- Resolved RPM metadata fallback for files without explicit header fields.
- Evidence archive: `evidence/cwru_initial/cwru_initial_results.zip`.

### 4.2 MAFAULDA Matched-Group Analysis (Stage 1B)
- Analyzed 9 real MAFAULDA recordings across 3 nominal speed groups (31.7 Hz, 45.5 Hz, 53.9 Hz) comparing Normal, 10 g Imbalance, and 1.5 mm Horizontal Misalignment.
- **Tachometer Consistency:** Edge-counting vs. spectral peak frequency matched to within **0.0468%**, demonstrating reliable internal speed measurement.
- **Rotor Mass Unbalance Finding:** Radial vibration under 10 g unbalance showed a massive, consistent increase in $1\times$ shaft rotational order energy:
  * 31.7 Hz Group: 8.78% (Normal) $\to$ **40.74%** (Unbalance)
  * 45.5 Hz Group: 8.39% (Normal) $\to$ **61.39%** (Unbalance)
  * 53.9 Hz Group: 10.21% (Normal) $\to$ **66.57%** (Unbalance)
- Evidence archive: `evidence/stage1b/mafaulda_stage1b_outputs.zip`. Extracted summary CSVs: `evidence/stage1b_csvs/`.

### 4.3 Acoustic Source Localization Simulation Suite (Stage 2)
- **Array Geometry:** 4-Microphone Uniform Linear Array spaced at $d = 20.32\text{ mm}$ ($60.96\text{ mm}$ total span), matching the physical breadboard aperture.
- **25-Point Off-Grid Spatial Benchmark (Stage 2C):**
  * Evaluated across 25 random off-grid source positions spanning $-65^\circ$ to $+65^\circ$ and $0.65\text{ m}$ to $2.10\text{ m}$ range.
  * **Median Angular Error:** **0.34°**
  * **90th-Percentile Error:** **0.68°**
  * **Angular Success Rate ($<5^\circ$ error):** **100.0% Pass Rate**
- **Noise Robustness & Multipath Sweeps (Stage 2D):**
  * **Clean ($\infty$ dB):** Median Error = 0.34° (100% pass)
  * **20 dB SNR:** Median Error = 0.34° (100% pass)
  * **10 dB SNR:** Median Error = 0.55° (100% pass)
  * **0 dB SNR:** Median Error = 2.52° (80% pass)
  * **Multipath / Reverberation (2 wall reflections, $\beta=0.35$):** Median Error = 0.40° (84% pass)
- Evidence plot: `evidence/stage2/stage2_snr_robustness_curves.png`. Comprehensive archive: `evidence/stage2/stage2_localization_evidence.zip`.

---

## 5. Delineation: Verified Status vs. Pending Physical Experiments

To maintain strict academic integrity and avoid conflating simulation/bringup proofs with completed mechanical experiments, the table below explicitly delineates current verified status from pending physical testbench validation:

| Subsystem / Capability | Verified Algorithmic & Synthetic State | Physical Hardware Bringup State | Pending Physical Testbench Validation |
| :--- | :--- | :--- | :--- |
| **ADXL345 Accelerometer** | Verified signed $a_x, a_y, a_z$ preservation, per-axis DC gravity removal ($25\text{ Hz}$ sine test), 400 Hz ring buffer ingestion, Type 0x05 parsing. | I2C communication verified on COM8, registers initialized, DEVID `0xE5` confirmed, static gravity $Z \approx -0.82g$ captured. | **Pending:** Continuous 400 Hz stream recording under actual motor unbalance mass load to capture physical harmonic distribution. |
| **FC-03 Optical Tachometer** | Verified $N-1$ revolution interval counting across $N=2..100$ pulses (1.000 order lock), absent-pulse UNAVAILABLE reporting without 1500 RPM fallback. | Hardware ISR trigger verified on GPIO 4, 32-bit pulse counter increments in packet header. | **Pending:** Synchronous pulse train capture under physical motor rotation with dynamic speed jitter to validate angular resampling on non-stationary hardware. |
| **4-Microphone ULA Array** | Verified sub-sample fractional delay phase compensation ($<0.1\ \mu\text{s}$ residual skew), SRP-PHAT steering to $0^\circ$ and $+30^\circ$, 25-point benchmark (0.34° error). | Synchronous Dual-I2S DMA clock lock (I2S0+I2S1 shared SCK/WS), 11,025 Hz 4-ch 16-bit PCM streaming on COM8 at 1.152M baud. | **Pending:** Calibrated acoustic chamber recordings using physical loudspeakers at known azimuths ($-45^\circ, 0^\circ, +45^\circ$) to quantify physical room reverberation and mic phase mismatch. |
| **Multimodal Real-Time HUD** | Decoupled background thread-safe ring buffer, Type 0x05 framing, sequence tracking, dynamic severity index, baseline differential engine. | GUI connects to COM8, streams live 4-ch audio and accelerometer packets without buffer overflows at 1.152M baud. | **Pending:** Live end-to-end motor fault demonstration on physical test rig with unbalance and misalignment loads. |
