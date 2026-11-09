# Spatial Multimodal Edge-AI Motor Dataset Recording Protocol

**Document Version**: 2.0 (Post-Audit Reconciled)  
**Date**: November 9, 2026  
**Authors**: Yogi (*Hardware Lead*), Antigravity Autonomous Agent (*Edge-AI Systems Architect*)  
**Status**: APPROVED FOR PHYSICAL DATASET ACQUISITION (Phase 1)  
**Target Repository**: `https://github.com/yoganandasp15/Spatial-accoustic-project`  

---

## 1. Executive Summary & Experimental Objective

This document establishes the **Standard Operating Procedure (SOP)** for physical multimodal dataset recording in support of the Minor Project:
> **"Spatial Multimodal Edge-AI: Answering both WHAT is wrong? and WHERE is the sound coming from? while proving Acoustic Disturbance $\neq$ Mechanical Fault"**

The objective of **Phase 1** is to acquire a verified, leak-free, empirically grounded physical dataset from the ESP32 multimodal sensor rig. This dataset will train and benchmark our pure-C TinyML classifier (`emlearn` Decision Tree / Random Forest) on the ESP32 in Phase 2.

### Core Scientific Premise (Project Blueprint & PDF Page 13)
$$\text{Acoustic Disturbance} \neq \text{Mechanical Fault}$$

A machine is **not mechanically broken** simply because someone is speaking, clapping, or operating a noisy power tool nearby. Conversely, a machine **is defective** if mechanical degradation produces abnormal vibration and rotational harmonics, regardless of whether ambient room noise is quiet or loud.

---

## 2. Sensor Hardware Configuration & Pre-Flight Checklist

Before launching any recording run, verify the physical hardware state:

```
                  +----------------------------------------------+
                  |         ESP32 Master Acquisition Rig         |
                  |          Dual-I2S + I2C + GPIO 4 ISR         |
                  +----------------------------------------------+
                               /            |             \
                              /             |              \
           [4-Mic INMP441 ULA]      [ADXL345 Accelerometer] [FC-03 Tachometer]
           I2S0 (MIC1, MIC2)        400 Hz Stream FIFO      GPIO 4 (D4) Interrupt
           I2S1 (MIC3, MIC4)        +/-2g Full Resolution   FALLING Edge (INPUT_PULLUP)
           10,000 Hz 16-bit PCM     Rigid Casing Mount      Optical Shaft Pulses (RPM)
```

### Pre-Flight Hardware Checklist:
- [ ] **ESP32 COM Port**: Plugged into host USB port (`COM8` at 921,600 baud).
- [ ] **Port Exclusivity**: Close any running instance of `spatial_acoustic_radar_pro.py`, Serial Monitor, or PuTTY. COM8 must be exclusively acquired by the recorder.
- [ ] **ADXL345 Accelerometer**: Firmly glued, bolted, or taped with industrial high-density foam tape directly to the motor stator / bearing block. Range is configured in firmware to **$\pm 2\text{ g}$ Full Resolution** (`DATA_FORMAT = 0x08`, 3.9 mg/LSB). Readings must stay within $\pm 2.0\text{ g}$ dynamic range; values approaching $1.95\text{ g}$ will trigger saturation warnings.
- [ ] **FC-03 Optical Tachometer**: Sensor slot aligned with the slotted encoder disc or reflective shaft marker. Connected to **GPIO 4** (`FC03_DO_PIN = 4` with `INPUT_PULLUP` on `FALLING` edge). Verify the module's onboard LED toggles when the motor shaft rotates.
- [ ] **4-Microphone ULA**: Breadboard linear array placed broadside to the motor ($d = 20.32\text{ mm}$ spacing, $60.96\text{ mm}$ total aperture), oriented so MIC1 is left and MIC4 is right.
- [ ] **DC Power Supply**: Independent bench power supply or battery powering the motor (5.0V – 12.0V). *Never power DC motors directly from the ESP32 3.3V/5V rails.*

### ⚠️ Critical Mechanical Safety Rule:
> **DO NOT attach improvised masses, bolts, washers, or adhesive weights to spinning motor shafts.**  
> DC motor shafts can reach rotational speeds in excess of 7,000 RPM ($\sim 116\text{ Hz}$). At these velocities, improvised detached masses become dangerous high-speed projectiles.  
> **Use the existing defective motor (`motor_B`) in its current physical state with `--fault-type unknown`.**

---

## 3. The 2×2 Factorial Experimental Matrix

To statistically prove disturbance rejection and train an unbiased classifier, physical recordings must cover the complete $2 \times 2$ factorial space:

| Cell ID | Motor Mechanical State | Acoustic Environment | Description & Fault Induction | Batch Launcher | Target Runs | Total Duration |
|:---:|:---:|:---:|:---|:---|:---:|:---:|
| **H-Q** | **HEALTHY** (`motor_A`) | **QUIET** | Motor in good condition, smooth rotation, ambient room noise ($<45\text{ dBA}$). | `RECORD_HEALTHY_MOTOR.bat` | 5 runs $\times$ 30s | 150 s (2.5 min) |
| **H-D** | **HEALTHY** (`motor_A`) | **DISTURBANCE** | Healthy motor running while external acoustic noise (speech, phone audio) is injected. | `RECORD_ACOUSTIC_DISTURBANCE.bat` | 5 runs $\times$ 30s | 150 s (2.5 min) |
| **D-Q** | **DEFECTIVE** (`motor_B`) | **QUIET** | Defective motor in existing condition (fault type `unknown`) in a quiet room. | `RECORD_DEFECTIVE_MOTOR.bat` | 5 runs $\times$ 30s | 150 s (2.5 min) |
| **D-D** | **DEFECTIVE** (`motor_B`) | **DISTURBANCE** | Defective motor running while external acoustic noise is simultaneously injected. | `RECORD_DEFECTIVE_DISTURBANCE.bat` | 5 runs $\times$ 30s | 150 s (2.5 min) |

### Aggregate Dataset Yield (20 Total Runs):
- **Total Physical Recording Time**: 600 seconds (10 minutes of active motor run).
- **Vibration Records**: $600\text{ s} \times 400\text{ Hz} = 240,000$ synchronized 3-axis samples.
- **Audio Records**: $600\text{ s} \times 10,000\text{ Hz} \times 4\text{ channels} = 24,000,000$ 16-bit PCM audio samples.
- **Tachometer Pulses**: Continuous optical shaft edge counting ($1\times, 2\times, 3\times$ order tracking).

---

## 4. One-Click Batch Launchers & CLI Execution

### Option A: One-Click Desktop Batch Launchers (Recommended)
From the repository root or Windows Explorer, double-click the corresponding `.bat` file:

1. **Record Healthy Motor (Quiet)**:
   ```cmd
   RECORD_HEALTHY_MOTOR.bat
   ```
2. **Record Healthy Motor + Acoustic Disturbance**:
   ```cmd
   RECORD_ACOUSTIC_DISTURBANCE.bat
   ```
3. **Record Defective Motor (Quiet)**:
   ```cmd
   RECORD_DEFECTIVE_MOTOR.bat
   ```
4. **Record Defective Motor + Acoustic Disturbance**:
   ```cmd
   RECORD_DEFECTIVE_DISTURBANCE.bat
   ```
5. **Validate Dataset Quality (Strict Mode)**:
   ```cmd
   VALIDATE_DATASET.bat
   ```

### Option B: Advanced Command-Line Interface (CLI)
You can customize parameters directly from PowerShell / CMD:
```powershell
python tools/record_motor_experiment.py `
    --motor-id motor_A `
    --health-label healthy `
    --environment quiet `
    --fault-type none `
    --session-id session_01_pilot `
    --duration 30.0 `
    --port COM8 `
    --supply-voltage 9.0 `
    --notes "Baseline healthy run with 9V supply"
```

For defective runs (using verified `unknown` fault label):
```powershell
python tools/record_motor_experiment.py `
    --motor-id motor_B `
    --health-label defective `
    --environment quiet `
    --fault-type unknown `
    --session-id session_01_pilot `
    --duration 30.0 `
    --port COM8 `
    --supply-voltage 9.0 `
    --notes "Defective motor B baseline run"
```

For acoustic disturbance injection:
```powershell
python tools/record_motor_experiment.py `
    --motor-id motor_A `
    --health-label healthy `
    --environment disturbance `
    --fault-type none `
    --session-id session_01_pilot `
    --disturbance-desc "Phone playing YouTube speech 40cm from array" `
    --duration 30.0 `
    --port COM8
```

---

## 5. Controlled Acoustic Disturbance Generation Procedures

When recording disturbance runs (**H-D** and **D-D**), use the following standardized disturbance sources:

1. **Continuous Speech / Podcast**: Place a smartphone or speaker 30–50 cm to the side of the 4-microphone array ($\theta \approx 30^\circ - 45^\circ$) playing speech at conversational volume ($65-75\text{ dBA}$).
2. **Periodic Hand Claps**: Clap your hands 3–5 times during the 30-second recording window at 0.5–1.0 meter distance.
3. **External Cooling Fan**: Run a small desk fan nearby to introduce turbulent broadband noise.

*Observation Goal*: The 4-channel microphone array and SRP-PHAT radar will detect and localize the incoming acoustic disturbance angle, while the accelerometer dynamic vibration ($a_{\text{dyn}}$) on the healthy motor remains low ($<0.08\text{ g}$), proving the motor itself is healthy.

---

## 6. Dataset Directory & Artifact Architecture

Each execution creates a unique, non-overwriting timestamped directory under `data/dataset/`:

```
data/dataset/
├── manifest.json                                       <-- Global registry of all recorded runs
├── validation_report.json                              <-- Output of validate_dataset.py
└── run_20261109_221530_123456_a1b2c3_motor_A_healthy_quiet/
    ├── metadata.json                                   <-- Provenance, status, RPM, metrics
    ├── vibration.csv                                   <-- 400 Hz Signed X, Y, Z + Dynamic (g)
    ├── audio_4ch.wav                                   <-- 10 kHz 4-Channel 16-bit PCM Audio
    └── packet_log.csv                                  <-- Monotonic arrival times, checksums, drops
```

### Artifact Description:
- **`metadata.json`**: Records run metadata, date/time, exact motor ID, condition labels, measured mean RPM, tachometer pulse count, dynamic vibration RMS, recording status (`COMPLETE`, `INTERRUPTED`, `INCOMPLETE`, `FAILED`), and data provenance (`PHYSICAL_HARDWARE` vs `SYNTHETIC_TESTBENCH`).
- **`vibration.csv`**: Contains per-sample signed acceleration ($a_x, a_y, a_z$), dynamic acceleration without gravity ($d_x, d_y, d_z, |d|$), tachometer pulse count, and instantaneous RPM.
- **`audio_4ch.wav`**: Standard RIFF WAVE file with 4 discrete channels (MIC1 through MIC4) sampled at 10,000 Hz in 16-bit signed PCM.
- **`packet_log.csv`**: Monotonic host arrival times (`time.monotonic()`), sample sequence numbers, CRC/checksum validity, and sequence gaps for network/bus health verification.
- **`manifest.json`**: Dataset-wide registry enabling automated loading, indexing, and stratified train/test splitting without directory scanning.

---

## 7. Quality Assurance & Validation Gate

After completing recording runs, execute the strict dataset validator:
```cmd
VALIDATE_DATASET.bat
```
*(Or in terminal: `python tools/validate_dataset.py --strict`)*

### Verification Pass Criteria:
1. **Recording Status**: Must be strictly `COMPLETE` (rejects `INTERRUPTED`, `INCOMPLETE`, or `FAILED` runs).
2. **Manifest Integrity**: Every directory on disk is registered in `manifest.json`.
3. **Audio Validity**: 4 channels, exactly 10,000 Hz framerate, 16-bit PCM, duration within $\pm 10\%$ of target, no NaNs/Infs, non-silent audio ($RMS > 0.0001$).
4. **Vibration Validity**: Sample count within $\pm 10\%$ of $400\text{ Hz} \times \text{duration}$, signed axes within configured $\pm 2\text{ g}$ dynamic range with zero sensor saturation ($|a| < 1.95\text{ g}$).
5. **Packet Transport**: Zero checksum errors (`checksum_errors == 0`), sequence gap packet drop rate $< 1.0\%$.
6. **No Synthetic Contamination**: Physical training datasets must have **zero synthetic testbench runs**.

---

## 8. Anti-Leakage Machine Learning Contract & Pilot Limitations

### GroupKFold by Session / Run ID (Phase 2):
1. Feature extraction windows (e.g. 1.0-second sliding windows with 50% overlap) from the *same* recording run must **NEVER** appear in both training and test sets.
2. Group cross-validation will strictly partition by `session_id` and `run_id`.

### Scientific Limitation of the Two-Motor Pilot:
In this initial 20-run pilot experiment:
- Motor A is always healthy.
- Motor B is always defective.

*Caution*: A machine learning model evaluated solely across these two specific physical motors could potentially learn specimen-specific acoustic or vibration resonances rather than universal mechanical fault mechanics. This 20-run pilot establishes that our sensor signals contain discriminatory features and proves acoustic disturbance rejection. Future multi-specimen testing with healthy and defective states on identical motor models will be required for universal generalization claims.
