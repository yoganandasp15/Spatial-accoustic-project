# Project Milestones, Verification Guide & Major-Project Roadmap

---

## 1. Project Milestones & Remediation Status (Completed November 2026)

All planned core engineering milestones, architectural remediations, and automated verification suites have been completed and verified across the repository:

- **Milestone 1 (DSP Pipeline Corrections):**
  * Harmonized cross-correlation convention to $X_1(f) X_2^*(f)$ in `calibrate_i2s_phase.py`.
  * Corrected incident angle geometry and saved physical compensation delay in seconds ($+24.69\ \mu\text{s}$).
  * Implemented frequency-domain fractional delay phase compensation ($H(f) = e^{-j 2\pi f \tau_{\text{comp}}}$) across multiple sample rates.
  * Corrected tachometer revolution counting in `order_tracking.py` to count exactly $N-1$ intervals across $N$ marker timestamps.
  * Eliminated silent 1500 RPM fallback when tachometer pulses are absent; reports explicit `UNAVAILABLE` status.
  * Preserved signed per-axis dynamic vibration ($a_x, a_y, a_z$) with DC gravity decoupling, eliminating full-wave Euclidean magnitude rectification.
  * Decoupled serial vibration ingestion via background ring buffer in `spatial_acoustic_radar_pro.py`.
  * Rebranded empirical Dynamic Vibration Severity Index in `offline_multimodal_evaluator.py`, removing uncertified ISO 10816-3 claims.
  * Labeled 1D ULA spatial output as radial backprojection grid peak ray (DOA $\theta$ focus).

- **Milestone 2 (ESP32 Firmware Continuous 400 Hz Streaming & Receiver Upgrade):**
  * Configured ADXL345 in 32-sample Stream FIFO mode (`FIFO_CTL = 0x80`) at 400 Hz ODR in `ESP32_Spatial_Acoustic_Radar_Firmware.ino`.
  * Upgraded binary packet protocol to Type 0x05 (16B header, $N$ accelerometer records, 1024B audio, 16-bit XOR checksum with `0x55AA` seed).
  * Configured primary UART communication to 921,600 baud (zero framing errors on CP2102; receiver also supports 1,152,000 baud).
  * Enhanced serial receiver in `spatial_acoustic_radar_pro.py` with defensive boundary checks, buffer shape validation, sequence roll-over handling, and batched timestamp monotonicity enforcement.

- **Milestone 3 (Automated Test Suite Expansion & Anti-Tautology Verification):**
  * Expanded `tests/test_multimodal_pipeline.py` with comprehensive unit tests for signed vibration, 16-bit XOR checksum corruption rejection, fractional delay phase compensation across sample rates, and $N-1$ tachometer interval counting.
  * Segregated physical hardware ping tests behind `@unittest.skipUnless(ENABLE_HARDWARE_TESTS=1)`.
  * Implemented adversarial stress suites (`test_m1_adversarial_stress.py`, `test_m2_firmware_protocol.py`, `test_m2_stress_adversarial.py`).
  * Created anti-tautology mutation suite (`test_m3_mutation_adversarial.py`) achieving 100% kill rate across 6 deliberate algorithmic mutants.
  * Repository-wide test pass rate: **55/55 tests passing (1 segregated hardware test skipped)**.

- **Milestone 4 (Repository Documentation & Log Reconciliation):**
  * Reconciled all historical commit SHAs in `docs/AGENT_SYNCHRONIZATION_LOG.md` against git history.
  * Updated `docs/PROJECT_STATUS.md` to strictly delineate verified algorithmic/synthetic functionality from pending physical experiments.
  * Updated `docs/NEXT_STEPS.md` with complete test instructions and technical specifications for missing physical datasets.

---

## 2. Missing Physical Hardware Datasets Required for External Feedback

While the entire algorithmic pipeline, firmware binary framing, serial receiver, and test harnesses are fully verified in simulation, synthetic streaming, and unit test environments, the following **three physical testbench datasets** are required to conclude end-to-end mechanical and acoustic experimental validation:

### Dataset A: Continuous 400 Hz 3-Axis Accelerometer Stream under Physical Motor Rotation
- **Objective:** Empirically validate the $1\times$ shaft unbalance order surge on physical hardware and confirm gravity vector stability under lateral dynamic oscillation without clipping or sensor resonance.
- **Hardware Setup:**
  * Sensor: ADXL345 accelerometer mounted rigidly to the bearing housing of a small DC or AC induction motor using cyanoacrylate adhesive or a direct screw mount (avoiding compliant breadboard tape).
  * Accelerometer Configuration: $\pm 2g$ or $\pm 4g$ full resolution, 400 Hz ODR, Stream FIFO mode via I2C to ESP32 on GPIO 17 (SDA) and GPIO 16 (SCL).
- **Test Conditions:**
  1. *Baseline Healthy Run:* Smooth rotor at steady-state operating speeds (1200, 1800, and 2400 RPM).
  2. *Unbalance Fault Run:* Known eccentric unbalance mass (e.g., 5g or 10g set-screw / washer attached to motor pulley or balance disc) at the same rotational speeds.
  3. *Radial Misalignment Run (optional):* Motor shaft coupled with $1.0\text{ mm}$ parallel or angular misalignment.
- **Required Data Format:**
  * Minimum recording duration: $>10\text{ seconds}$ per condition (>4,000 samples @ 400 Hz).
  * Storage format: Continuous Type 0x05 binary serial capture or parsed CSV containing columns `Timestamp_us`, `Accel_X_mg`, `Accel_Y_mg`, `Accel_Z_mg`.
  * Validation Target: Confirmation that $1\times$ order harmonic surges significantly ($>30\%$ total spectral energy) under unbalance, with signed dynamic components ($a_{\text{dyn}}$) maintaining symmetric zero-mean excursions.

### Dataset B: Simultaneous Optical Tachometer Pulse Trains with Dynamic Speed Variation
- **Objective:** Validate angular domain resampling, fractional interpolation, and order tracking under non-stationary rotational speed, ramp-up/coast-down transients, and motor speed jitter.
- **Hardware Setup:**
  * Sensor: FC-03 infrared optical slot sensor connected to ESP32 GPIO 4 with hardware interrupt (FALLING edge matching active-low blade entry, with INPUT_PULLUP and 200 µs debounce).
  * Target: Single-slot or multi-slot optical encoder wheel mounted on the rotating motor shaft.
- **Test Conditions:**
  1. *Steady-State Operation with Natural Jitter:* Motor running at nominal 1500 RPM with natural load fluctuations ($1\text{--}5\%$ RPM jitter).
  2. *Speed Ramp-Up Transient:* Accelerating from 0 to 2400 RPM over 5 seconds.
  3. *Coast-Down Transient:* Power cut from 2400 RPM coasting to a stop under bearing friction.
- **Required Data Format:**
  * Microsecond-precision edge timestamp log `pulse_timestamps_us` recorded synchronously with the accelerometer stream.
  * Validation Target: Verifying that $N-1$ revolution interval interpolation maintains a sharp $1.000$ order lock without spectral blurring or leakage across speed variations, and verifying clean graceful degradation (`status: "UNAVAILABLE"`) during zero-speed stops.

### Dataset C: Calibrated Acoustic Multi-Microphone Chamber Recordings at Known Ground-Truth Angles
- **Objective:** Characterize physical inter-channel phase variance, validate the $+24.69\ \mu\text{s}$ inter-pair DMA delay compensation in open air, and quantify DOA angular resolution under real-world multipath and room reverberation.
- **Hardware Setup:**
  * Array: 4-microphone Uniform Linear Array (INMP441 MEMS microphones, inter-element spacing $d = 20.32\text{ mm}$, total aperture $60.96\text{ mm}$) streaming 4 channels @ 10,000 Hz via Dual-I2S DMA (I2S0 + I2S1).
  * Sound Source: Calibrated studio loudspeaker or reference acoustic transducer driven by an audio interface.
- **Test Conditions:**
  * Source Test Signals: Bandpass white noise (1 kHz – 4 kHz), linear swept-sine chirps (500 Hz – 5 kHz), and recorded electric motor acoustic signatures.
  * Known Angular Bearings: Fixed radial standoff $r = 1.0\text{ m}$ placed at calibrated azimuth angles:
    * Broadside: $0^\circ$
    * Off-Axis Left: $-30^\circ$, $-45^\circ$, $-60^\circ$
    * Off-Axis Right: $+30^\circ$, $+45^\circ$, $+60^\circ$
  * Acoustic Environments: (a) Anechoic / quiet acoustic space (free-field baseline), (b) Typical laboratory / office room with reflective walls (multipath reverberation).
- **Required Data Format:**
  * 4-channel 16-bit PCM WAV recording (`multimodal_4mic_audio.wav`) alongside calibrated geometric ground-truth coordinates $(x_s, y_s, z_s)$.
  * Validation Target: Demonstrating median DOA bearing error $<2.0^\circ$ across all angles, verifying that frequency-domain fractional delay phase compensation eliminates DMA skew in physical space, and measuring the array directivity pattern.

---

## 3. Checklist for Users and Evaluators

### 3.1 Execute the Automated Unit & Adversarial Test Suite
To verify all 44 unit, protocol, adversarial stress, and anti-tautology tests in 1 second:
```powershell
# From the repository root:
python -m unittest discover -s tests -p "test_*.py" -v
# Or run the batch launcher:
.\RUN_UNIT_TESTS.bat
```
Expected output: `Ran 44 tests in ~1.05s. OK (skipped=1)`. (The single skipped test is the segregated physical hardware ping test, which requires connected hardware).

### 3.2 Verify ESP32 Firmware Compilation
To verify that the continuous 400 Hz Stream FIFO firmware compiles cleanly:
```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 "stage3_esp32_firmware\ESP32_Spatial_Acoustic_Radar_Firmware\ESP32_Spatial_Acoustic_Radar_Firmware.ino"
```
Expected output: 0 errors, 0 warnings. Program flash usage ~24%, dynamic memory ~8%.

### 3.3 Run the Offline Multimodal Benchmark
To run the end-to-end evaluation benchmark on stored multi-modal recordings:
```powershell
python stage4_realtime_fusion_radar/src/offline_multimodal_evaluator.py
# Or run the batch launcher:
.\RUN_OFFLINE_BENCHMARK.bat
```
Generates offline diagnostic plots and summaries in `evidence/offline_multimodal_summary.json`.

### 3.4 Run the Interactive Live Radar HUD
Connect the ESP32 board to the USB port (COM8) and launch the live HUD:
```powershell
python stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py
# Or double-click:
.\RUN_LIVE_RADAR.bat
```
* **Acoustic Tracking:** Speak or clap near the array; observe the DOA needle and left/right flank VU meters track sound direction.
* **Golden Baseline (Key `'B'`):** Calibrate and lock healthy reference baseline.
* **Evidence Mode Tags:** Status bar displays whether running in live hardware mode, synthetic simulation, or laptop audio demo.

---

## 4. Major Project Scaling Roadmap (Future Academic Year)

1. **Autonomous On-Chip Edge-AI Inference:**
   * Migrate hardware from classic ESP32 to **ESP32-S3** (dual-core Xtensa LX7 with 512-bit SIMD vector instructions) or an **Edge TPU / Kendryte K210**.
   * Deploy lightweight 1D-CNN or Spatio-Temporal Graph Neural Networks (ST-GNN) quantized to INT8 directly in onboard flash for zero-latency localized inference without requiring a laptop host.
2. **Industrial-Grade Sensor Front-Ends & Packaging:**
   * Design custom PCB with galvanic isolation and industrial IEPE / 4–20 mA current loop interfaces to monitor heavy-duty induction motors (3-phase 415V).
   * Integrate hermetically sealed industrial piezoelectric accelerometers for high-temperature and harsh-vibration environments.
3. **Longitudinal Degradation Tracking & Prognostics:**
   * Conduct continuous multi-week run-to-failure bearing wear experiments on a dedicated mechanical test rig.
   * Train Remaining Useful Life (RUL) prognostic models (e.g., Temporal Convolutional Networks / Transformers) to predict hours until mechanical failure.
