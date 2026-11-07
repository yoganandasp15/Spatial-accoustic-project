# Project Overview

## Approved project title and purpose

**Spatial Multimodal Edge-AI for Fault Detection and Localization of Rotating Machinery**

The proposal's core question for the minor is: **Can we detect a controlled mechanical fault and spatially localize the dominant acoustic source?** The minor uses a small, safe DC motor. The broader major-project direction can scale to larger/industrial machinery using properly rated, isolated sensing interfaces.

## Intended sensing modalities

1. **Vibration:** accelerometer attached to the machine. Features in the proposal include RMS, peak, kurtosis, spectral energy, 1×/2× amplitudes and harmonics.
2. **Acoustic array:** multiple microphones at known coordinates. Sound arrival-time differences are used for TDOA, GCC-PHAT and SRP-PHAT/beamforming, producing a spatial heatmap.
3. **RPM:** measures actual rotational speed. Shaft frequency is `f_r = RPM / 60`; vibration peaks can then be expressed as rotational orders.

The modalities describe the same physical event. The intended system combines vibration features, acoustic source position/energy/features and RPM/order features, then estimates fault, location and potentially severity/confidence.

## Fault conditions named in the proposal

- Normal
- Imbalance
- Misalignment
- Looseness

The proposal also discusses bearing-related events as an example, but the controlled minor-project demonstration is framed around the four conditions above.

## Intended signal-processing order

1. Motor → raw vibration → FFT → RPM relationship/order analysis.
2. Microphones → TDOA → beamforming/SRP-PHAT → acoustic heatmap.
3. Vibration + acoustic + RPM → feature/sensor fusion.
4. AI classification/inference only after the physical/DSP pipeline is understood.
5. Edge deployment after laptop-based validation.

## Minor versus major

### Minor
- Safe small DC motor.
- Sensor acquisition and processing can be laptop-assisted.
- Demonstrate controlled fault signatures and spatial localization of dominant acoustic source.
- Outputs should support “what is wrong?” and “where is the dominant sound source?” without overclaiming internal fault-part localization.

### Major
- Custom acquisition hardware.
- MCU/edge processor and real-time DSP.
- Edge-AI and standalone diagnostic instrument.
- Longer-term condition history, degradation trends and early-warning/predictive maintenance only after collecting longitudinal degradation data.

The proposal explicitly says not to start by training an AI model. Begin with physics, raw signals, FFT and RPM; then microphones/TDOA/beamforming/heatmap; then feature fusion; then AI; then edge deployment.

## Main risks highlighted in the proposal

- Acoustic reflections and background noise.
- Microphone geometry and array calibration.
- Channel synchronization and shared clocks.
- Array aperture versus spatial resolution/frequency.
- Limited real fault data.
- Acoustic anomalies are not automatically mechanical faults.
- AI overfitting across motor, RPM, load, position and environment.
- Embedded RAM, compute and real-time deadlines.

## Original proposal

See `references/Minor_project_topic_1.pdf`. Treat it as the primary source for the intended scope and terminology. Do not silently expand or change its scope.
