import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    os.path.join(BASE_DIR, "src"),
    os.path.join(BASE_DIR, "stage2_acoustic_localization", "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import scipy.signal as signal
from calibrate_i2s_phase import gcc_phat, DEFAULT_ARRAY_GEOMETRY, SPEED_OF_SOUND

fs = 44100.0
duration = 0.5
n_samples = int(fs * duration)
t = np.arange(n_samples) / fs

# Create wideband bandpass signal (500 Hz to 4000 Hz)
np.random.seed(42)
white = np.random.randn(n_samples)
b, a = signal.butter(4, [500.0 / (fs/2), 4000.0 / (fs/2)], btype='bandpass')
source_sig = signal.filtfilt(b, a, white)

mics = DEFAULT_ARRAY_GEOMETRY['mic_positions']
angles = [-60.0, -30.0, 0.0, 30.0, 60.0]
injected_hw_skew_us = 24.69
injected_hw_skew_sec = injected_hw_skew_us * 1e-6

freqs = np.fft.rfftfreq(n_samples, d=1.0/fs)
source_fft = np.fft.rfft(source_sig)

pairs = [
    (0, 1, 'Intra-Pair A (Mic1-Mic2, I2S0)'),
    (2, 3, 'Intra-Pair B (Mic3-Mic4, I2S1)'),
    (1, 2, 'Inter-Pair Center (Mic2-Mic3, I2S0 vs I2S1)'),
    (0, 3, 'Inter-Pair Outer (Mic1-Mic4, I2S0 vs I2S1)'),
    (0, 2, 'Cross-Pair 1-3 (I2S0 vs I2S1)'),
    (1, 3, 'Cross-Pair 2-4 (I2S0 vs I2S1)')
]

print("=" * 95)
print(f"I2S PHASE CALIBRATION ANGLE-INVARIANCE STRESS TEST (Injected Skew: {injected_hw_skew_us:.2f} μs)")
print("=" * 95)
header = "{:<8} | {:<22} | {:<20} | {:<18} | {:<18}".format(
    "Angle", "Inter-Pair Skew (μs)", "Residual Error (μs)", "Intra-Pair A (μs)", "Intra-Pair B (μs)"
)
print(header)
print("-" * 95)

max_residual_err = 0.0

for angle in angles:
    theta_rad = np.radians(angle)
    audio = np.zeros((n_samples, 4), dtype=np.float32)
    for m in range(4):
        tau_acoustic = -mics[m]['x'] * np.sin(theta_rad) / SPEED_OF_SOUND
        total_delay = tau_acoustic
        if m in [2, 3]:  # Pair B has hardware skew
            total_delay += -injected_hw_skew_sec
        ch_fft = source_fft * np.exp(-2j * np.pi * freqs * total_delay)
        audio[:, m] = np.fft.irfft(ch_fft, n=n_samples)
        
    skews_inter = []
    intra_a = 0.0
    intra_b = 0.0
    for m1, m2, label in pairs:
        theo_tau = (mics[m2]['x'] - mics[m1]['x']) * np.sin(theta_rad) / SPEED_OF_SOUND
        meas_tau, _, _ = gcc_phat(audio[:, m1], audio[:, m2], fs, max_tau=0.005, interp=32)
        skew_sec = meas_tau - theo_tau
        skew_us = skew_sec * 1e6
        if 'Inter-Pair' in label or 'Cross-Pair' in label:
            skews_inter.append(skew_us)
        elif 'Intra-Pair A' in label:
            intra_a = skew_us
        elif 'Intra-Pair B' in label:
            intra_b = skew_us
            
    mean_skew = float(np.mean(skews_inter))
    res_err = abs(mean_skew - injected_hw_skew_us)
    max_residual_err = max(max_residual_err, res_err)
    
    row = "{:<8.1f} | {:<22.4f} | {:<20.4f} | {:<18.4f} | {:<18.4f}".format(
        angle, mean_skew, res_err, intra_a, intra_b
    )
    print(row)
    
    # Assert residual error is < 0.1 μs per requirement
    assert res_err < 0.1, f"Angle {angle}° failed with residual error {res_err:.4f} μs >= 0.1 μs"

print("-" * 95)
print(f"[PASS] All extreme angles tested: Maximum Residual Error is {max_residual_err:.4f} μs (< 0.1 μs requirement).")
print("=" * 95)
