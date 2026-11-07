import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    os.path.join(BASE_DIR, "src"),
    os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "src"),
    os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import numpy as np

# Construct test signal: 1g static gravity on Z, 0.05g lateral oscillation on X at 25 Hz
fs = 400.0  # ADXL345 ODR = 400 Hz
duration = 4.0  # 4 seconds = 100 revolutions at 25 Hz
t = np.arange(0, duration, 1.0 / fs)
f_rot = 25.0  # 25 Hz shaft rotation (1500 RPM)
amp_rot = 0.05  # 0.05 g amplitude

ax = amp_rot * np.sin(2.0 * np.pi * f_rot * t)
ay = np.zeros_like(t)
az = -1.0 * np.ones_like(t)  # -1.0 g static gravity

print("=" * 90)
print("DYNAMIC VIBRATION VS RECTIFIED EUCLIDEAN NORM STRESS TEST")
print("=" * 90)
print(f"Input Signal: Static Gravity Az = -1.0g | Lateral Oscillation Ax = {amp_rot}g @ {f_rot} Hz")
print("-" * 90)

# -------------------------------------------------------------------------
# Method 1: Per-Axis DC Removal (M1 Implementation in evaluator & radar_pro)
# -------------------------------------------------------------------------
ax_dyn = ax - np.mean(ax)
ay_dyn = ay - np.mean(ay)
az_dyn = az - np.mean(az)

# Compute FFT on dynamic lateral component ax_dyn
N = len(t)
freqs = np.fft.rfftfreq(N, d=1.0/fs)
fft_ax = np.abs(np.fft.rfft(ax_dyn)) * (2.0 / N)

peak_idx_m1 = np.argmax(fft_ax[1:]) + 1
peak_freq_m1 = freqs[peak_idx_m1]
peak_amp_m1 = fft_ax[peak_idx_m1]
rms_m1 = np.sqrt(np.mean(ax_dyn**2))
expected_rms = amp_rot / np.sqrt(2.0)

# Check energy at 25 Hz (1X) vs 50 Hz (2X)
idx_25hz = np.argmin(np.abs(freqs - 25.0))
idx_50hz = np.argmin(np.abs(freqs - 50.0))
e_25hz_m1 = fft_ax[idx_25hz]
e_50hz_m1 = fft_ax[idx_50hz]

print("[Method 1: Per-Axis DC Decoupling]")
print(f"  Detected Peak Frequency : {peak_freq_m1:.2f} Hz (Expected: {f_rot:.2f} Hz)")
print(f"  Peak Amplitude          : {peak_amp_m1:.5f} g (Expected: {amp_rot:.5f} g)")
print(f"  Measured RMS            : {rms_m1:.5f} g (Expected: {expected_rms:.5f} g)")
print(f"  25 Hz (1X) Amplitude    : {e_25hz_m1:.5f} g")
print(f"  50 Hz (2X) Amplitude    : {e_50hz_m1:.5f} g")
print(f"  Frequency Doubling Ratio: {e_50hz_m1 / max(1e-12, e_25hz_m1):.6f} (Must be near 0.0)")

# -------------------------------------------------------------------------
# Method 2: Rectified Euclidean Magnitude Norm (Flawed Pre-M1 Implementation)
# -------------------------------------------------------------------------
a_mag = np.sqrt(ax**2 + ay**2 + az**2)
a_mag_dyn = a_mag - np.mean(a_mag)
fft_mag = np.abs(np.fft.rfft(a_mag_dyn)) * (2.0 / N)

peak_idx_m2 = np.argmax(fft_mag[1:]) + 1
peak_freq_m2 = freqs[peak_idx_m2]
peak_amp_m2 = fft_mag[peak_idx_m2]
rms_m2 = np.sqrt(np.mean(a_mag_dyn**2))

e_25hz_m2 = fft_mag[idx_25hz]
e_50hz_m2 = fft_mag[idx_50hz]

print("\n[Method 2: Flawed Rectified Euclidean Magnitude Norm]")
print(f"  Detected Peak Frequency : {peak_freq_m2:.2f} Hz (DOUBLED HARMONIC!)")
print(f"  Peak Amplitude          : {peak_amp_m2:.7f} g (Crushed from 0.05g by factor of {amp_rot/peak_amp_m2:.1f}x!)")
print(f"  Measured RMS            : {rms_m2:.7f} g")
print(f"  25 Hz (1X) Amplitude    : {e_25hz_m2:.7f} g (Fundamental completely suppressed!)")
print(f"  50 Hz (2X) Amplitude    : {e_50hz_m2:.7f} g (False misalignment artifact)")
print(f"  Frequency Doubling Ratio: {e_50hz_m2 / max(1e-12, e_25hz_m2):.2f}")

print("\n" + "=" * 90)
assert abs(peak_freq_m1 - 25.0) < 0.1, f"Method 1 peak frequency {peak_freq_m1} != 25.0 Hz"
assert abs(peak_amp_m1 - 0.05) < 0.001, f"Method 1 peak amplitude {peak_amp_m1} != 0.05g"
assert e_50hz_m1 < 1e-4, f"Method 1 has unexpected 2X harmonic: {e_50hz_m1}"
assert abs(peak_freq_m2 - 50.0) < 0.1, f"Method 2 peak frequency {peak_freq_m2} != 50.0 Hz"
print("[PASS] Method 1 preserves 25 Hz fundamental at 0.05g without frequency doubling.")
print("[PASS] Method 2 empirically confirmed to crush amplitude by 80x and double frequency to 50 Hz.")
print("=" * 90)
