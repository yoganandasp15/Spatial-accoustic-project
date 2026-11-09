# Physical Motor Experiment Records

This directory is designated exclusively for **genuine physical recordings** captured from the rotating machinery test rig via the automated experiment suite:

- `RECORD_HEALTHY_MOTOR.bat`: Records baseline baseline telemetry from the healthy rotating motor.
- `RECORD_DEFECTIVE_MOTOR.bat`: Records fault telemetry from a motor with mass unbalance or shaft misalignment.
- `RECORD_ACOUSTIC_DISTURBANCE.bat`: Records acoustic disturbance rejection (external noise bursts while motor remains nominal).
- `GENERATE_EXPERIMENT_REPORT.bat`: Performs multi-axis spectral order analysis, Parseval ISO velocity RMS integration, and compiles `docs/evidence/PHYSICAL_EXPERIMENT_REPORT.md`.

## Epistemic Integrity Notice
Per the project's zero-fabrication contract:
- Automatic synthetic fallback has been permanently removed from all data acquisition tools (`record_motor_experiment.py` and `calibrate_physical_array.py`).
- Hardware recording tools require a live ESP32 on COM8; if hardware is disconnected or unpowered, the scripts cleanly abort without generating synthetic data or masquerading simulations as physical experiments.
- Synthetic simulations may only be generated if explicitly passed `--synthetic`, and are tagged with metadata provenance `SYNTHETIC_SIMULATION (EXPLICIT USER FLAG)`.
