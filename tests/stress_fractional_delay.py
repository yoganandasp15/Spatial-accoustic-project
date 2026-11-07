import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    os.path.join(BASE_DIR, "src"),
    os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "src"),
    os.path.join(BASE_DIR, "stage2_acoustic_localization", "src"),
    os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import scipy.signal as signal
from offline_multimodal_evaluator import apply_pair_phase_compensation
from calibrate_i2s_phase import gcc_phat

sample_rates = [44100.0, 16000.0, 11025.0]
test_skews_us = [24.69, -15.35, 45.12, 5.50]

print("=" * 105)
print("FRACTIONAL DELAY PHASE COMPENSATION STRESS TEST")
print("=" * 105)
header = "{:<10} | {:<15} | {:<13} | {:<20} | {:<12} | {:<15}".format(
    "Fs (Hz)", "Inj Skew (us)", "Inj Samples", "Residual Skew (us)", "Amp Ratio", "Interior NMSE"
)
print(header)
print("-" * 105)

for fs in sample_rates:
    dt = 1.0 / fs
    duration = 0.2
    n_samples = int(fs * duration)
    t = np.arange(n_samples) * dt
    
    f_high = min(4000.0, fs * 0.4)
    b, a = signal.butter(4, [300.0 / (fs/2), f_high / (fs/2)], btype='bandpass')
    np.random.seed(123)
    base_sig = signal.filtfilt(b, a, np.random.randn(n_samples))
    
    audio_orig = np.zeros((n_samples, 4), dtype=np.float64)
    for c in range(4):
        audio_orig[:, c] = base_sig
        
    freqs = np.fft.rfftfreq(n_samples, d=dt)
    base_fft = np.fft.rfft(base_sig)
    
    for skew_us in test_skews_us:
        skew_sec = skew_us * 1e-6
        skew_samples = skew_sec * fs
        
        audio_skewed = audio_orig.copy()
        skew_shift = np.exp(-2j * np.pi * freqs * skew_sec)
        ch_skewed = np.fft.irfft(base_fft * skew_shift, n=n_samples)
        audio_skewed[:, 2] = ch_skewed
        audio_skewed[:, 3] = ch_skewed
        
        comp_delay_sec = -skew_sec
        audio_comp = apply_pair_phase_compensation(audio_skewed, fs, comp_delay_sec)
        
        est_tau, _, _ = gcc_phat(audio_orig[:, 0], audio_comp[:, 2], fs, max_tau=0.005, interp=32)
        res_skew_us = est_tau * 1e6
        
        interior = slice(128, n_samples - 128)
        orig_interior = audio_orig[interior, 2]
        comp_interior = audio_comp[interior, 2]
        
        amp_ratio = np.std(comp_interior) / np.std(orig_interior)
        nmse = np.mean((comp_interior - orig_interior)**2) / np.var(orig_interior)
        
        row = "{:<10.0f} | {:<15.2f} | {:<13.3f} | {:<20.4f} | {:<12.5f} | {:<15.2e}".format(
            fs, skew_us, skew_samples, res_skew_us, amp_ratio, nmse
        )
        print(row)
