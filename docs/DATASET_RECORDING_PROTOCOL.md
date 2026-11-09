# Spatial Multimodal Edge-AI Motor Dataset Recording Protocol

**Document Version**: 1.0  
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

A machine is **not mechanically broken** simply because someone is speaking, clapping, or operating a noisy power tool nearby. Conversely, a machine **is defective** if bearing cage failure or rotor unbalance produces abnormal vibration and rotational harmonics, regardless of whether ambient room noise is quiet or loud.

---

## 2. Sensor Hardware Configuration & Pre-Flight Checklist

Before launching any recording run, verify the physical hardware state:

```
                  +----------------------------------------------+
                  |         ESP32 Master Acquisition Rig         |
                  |          Dual-I2S + SPI/I2C + GPIO          |
                  +----------------------------------------------+
                               /            |             \
                              /             |              \
           [4-Mic INMP441 ULA]      [ADXL345 Accelerometer] [FC-03 Tachometer]
           I2S0 (MIC1, MIC2)        400 Hz FIFO ODR         GPIO 34 Interrupt
           I2S1 (MIC3, MIC4)        Signed X, Y, Z (g)      Optical Pulse Edge
           10,000 Hz 16-bit PCM     Rigid Casing Mount      Shaft Speed (RPM)
```

### Pre-Flight Hardware Checklist:
- [ ] **ESP32 COM Port**: Plugged into host USB port (`COM8` at 921,600 baud).
- [ ] **Port Exclusivity**: Close any running instance of `spatial_acoustic_radar_pro.py`, Serial Monitor, or PuTTY. COM8 must be exclusively acquired by the recorder.
- [ ] **ADXL345 Accelerometer**: Firmly glued, bolted, or taped with industrial double-sided foam tape directly to the motor stator / bearing block. *Loose mounting corrupts high-frequency vibration signals.*
- [ ] **FC-03 Optical Tachometer**: Sensor slot aligned with the slotted encoder disc or reflective shaft marker. Verify the module's onboard LED toggles when the motor shaft rotates.
- [ ] **4-Microphone ULA**: Breadboard linear array placed broadside to the motor ($d = 20.32\text{ mm}$ spacing, $60.96\text{ mm}$ total aperture), oriented so MIC1 is left and MIC4 is right.
- [ ] **DC Power Supply**: Independent DC bench supply or battery powering the motor (5.0V – 12.0V). *Never power high-current DC motors directly from the ESP32 3.3V/5V rails.*

---

## 3. The 2×2 Factorial Experimental Matrix

To statistically prove disturbance rejection and train an unbiased classifier, physical recordings must cover the complete $2 \times 2$ factorial space:

| Cell ID | Motor Mechanical State | Acoustic Environment | Description & Fault Induction | Target Runs | Total Duration |
|:---:|:---:|:---:|:---|:---:|:---:|
| **H-Q** | **HEALTHY** (`motor_A`) | **QUIET** | Motor in good condition, smooth rotation, ambient room noise ($<45\text{ dBA}$). | 5 runs $\times$ 30s | 150 s (2.5 min) |
| **H-D** | **HEALTHY** (`motor_A`) | **DISTURBANCE** | Healthy motor running while external acoustic noise (speech, clapping, phone music) is played nearby. | 5 runs $\times$ 30s | 150 s (2.5 min) |
| **D-Q** | **DEFECTIVE** (`motor_B`) | **QUIET** | Motor with mass unbalance (eccentric mass on rotor) or shaft misalignment in a quiet room. | 5 runs $\times$ 30s | 150 s (2.5 min) |
| **D-D** | **DEFECTIVE** (`motor_B`) | **DISTURBANCE** | Defective motor running while external acoustic noise is simultaneously injected. | 5 runs $\times$ 30s | 150 s (2.5 min) |

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
2. **Record Defective Motor (Quiet)**:
   ```cmd
   RECORD_DEFECTIVE_MOTOR.bat
   ```
3. **Record Acoustic Disturbance Test**:
   ```cmd
   RECORD_ACOUSTIC_DISTURBANCE.bat
   ```
4. **Validate Dataset Quality**:
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
    --duration 30.0 `
    --port COM8 `
    --supply-voltage 9.0 `
    --notes "Baseline healthy run with 9V supply"
```

For defective runs with mass unbalance:
```powershell
python tools/record_motor_experiment.py `
    --motor-id motor_B `
    --health-label defective `
    --environment quiet `
    --fault-type unbalance `
    --duration 30.0 `
    --port COM8 `
    --supply-voltage 9.0 `
    --notes "Attached 1.5g eccentric adhesive mass to rotor blade"
```

For acoustic disturbance injection:
```powershell
python tools/record_motor_experiment.py `
    --motor-id motor_A `
    --health-label healthy `
    --environment disturbance `
    --fault-type none `
    --disturbance-desc "Loudspeaker playing YouTube speech 40cm from array" `
    --duration 30.0 `
    --port COM8
```

---

## 5. Controlled Acoustic Disturbance Generation Procedures

When recording disturbance runs (**H-D** and **D-D**), use the following standardized disturbance sources:

1. **Continuous Speech / Podcast**: Place a smartphone or speaker 30–50 cm to the side of the 4-microphone array ($\theta \approx 30^\circ - 45^\circ$) playing speech at conversational volume ($65-75\text{ dBA}$).
2. **Periodic Hand Claps**: Clap your hands 3–5 times during the 30-second recording window at 0.5–1.0 meter distance.
3. **Broadband Motor/Fan Interference**: Run a small hair dryer or external cooling fan in the background.

*Observation Goal*: The 4-channel microphone array and SRP-PHAT radar will detect and localize the incoming acoustic disturbance angle, while the accelerometer dynamic vibration ($a_{\text{dyn}}$) remains low ($<0.08\text{ g}$), proving the motor itself is healthy.

---

## 6. Dataset Directory & Artifact Architecture

Each execution creates a unique, non-overwriting timestamped directory under `data/dataset/`:

```
data/dataset/
├── manifest.json                                  <-- Global registry of all recorded runs
├── validation_report.json                         <-- Output of validate_dataset.py
└── run_20261109_221530_motor_A_healthy_quiet/     <-- Timestamped Run Folder
    ├── metadata.json                              <-- Provenance, parameters, RPM, metrics
    ├── vibration.csv                              <-- 400 Hz Signed X, Y, Z + Dynamic (g)
    ├── audio_4ch.wav                              <-- 10 kHz 4-Channel 16-bit PCM Audio
    └── packet_log.csv                             <-- Monotonic timestamps, checksums, drops
```

### Artifact Description:
- **`metadata.json`**: Records run metadata, date/time, exact motor ID, condition labels, measured mean RPM, tachometer pulse count, dynamic vibration RMS, and data provenance (`PHYSICAL_HARDWARE` vs `SYNTHETIC_TESTBENCH`).
- **`vibration.csv`**: Contains per-sample signed acceleration ($a_x, a_y, a_z$), dynamic acceleration without gravity ($d_x, d_y, d_z, |d|$), tachometer pulse count, and instantaneous RPM.
- **`audio_4ch.wav`**: Standard RIFF WAVE file with 4 discrete channels (MIC1 through MIC4) sampled at 10,000 Hz in 16-bit signed PCM.
- **`packet_log.csv`**: Host receive arrival times, sample sequence numbers, CRC/checksum validity, and sequence gaps for network/bus health verification.
- **`manifest.json`**: Dataset-wide registry enabling automated loading, indexing, and stratified train/test splitting without directory scanning.

---

## 7. Quality Assurance & Validation Gate

After completing a batch of runs, execute the dataset validator:
```cmd
VALIDATE_DATASET.bat
```
*(Or in terminal: `python tools/validate_dataset.py`)*

### Verification Pass Criteria:
1. **Manifest Integrity**: Every directory on disk is registered in `manifest.json`.
2. **Audio Validity**: 4 channels, exactly 10,000 Hz framerate, 16-bit PCM, no NaNs/Infs, non-silent audio ($RMS > 0.0005$).
3. **Vibration Validity**: Sample count within $\pm 10\%$ of $400\text{ Hz} \times \text{duration}$, signed axes within ADXL345 $\pm 16\text{ g}$ dynamic range.
4. **Packet Transport**: 0 corrupted packets (checksum valid), packet drop rate $< 0.5\%$.
5. **No Synthetic Contamination**: Physical training datasets must have **0 synthetic testbench runs**.

---

## 8. Anti-Leakage Machine Learning Contract (Phase 2 Preview)

When training models in Phase 2, the following rules are strictly enforced:
1. **GroupKFold by Session / Run ID**: Feature extraction windows (e.g. 1.0-second sliding windows with 50% overlap) from the *same* recording run must **NEVER** appear in both training and test sets.
2. **Motor Stratification**: If multiple physical motors are available, evaluate test generalization on an unseen motor ID.
3. **Strict Feature Set**: Input features to the TinyML classifier will consist of vibration RMS, crest factor, kurtosis, $1\times/2\times$ rotational order energy, and acoustic spectral energy. High-frequency bearing frequencies above the 200 Hz Nyquist floor of ADXL345 will not be claimed.
