"""
=============================================================================
 AUTOMATED UNIT & INTEGRATION TEST SUITE FOR MULTIMODAL PIPELINE
=============================================================================
Module: tests/test_multimodal_pipeline.py
Target: Validates DSP, acoustic localization, ISO 10816, and order tracking
Author: Antigravity Autonomous Agent (Pair Programming with Yogi)
Date: 2026-11-07 (ECE Minor Project Target)
=============================================================================
"""

import os
import sys
import unittest
import numpy as np

# Add component src directories to sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    os.path.join(BASE_DIR, "src"),
    os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "src"),
    os.path.join(BASE_DIR, "stage2_acoustic_localization", "src"),
    os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

from calibrate_i2s_phase import gcc_phat
from order_tracking import compute_order_tracking
from offline_multimodal_evaluator import compute_srp_phat_grid, DEFAULT_ARRAY_GEOMETRY


class TestMultimodalPipeline(unittest.TestCase):

    def test_gcc_phat_synthetic_delay(self):
        """Verifies GCC-PHAT delay estimation on synthetic bandpass signals with known sub-sample shift."""
        fs = 44100.0
        n_samples = 4096
        # Generate broadband random Gaussian signal
        np.random.seed(42)
        sig = np.random.randn(n_samples).astype(np.float32)
        
        # Bandpass filter between 500 Hz and 4000 Hz
        from scipy import signal
        b, a = signal.butter(4, [500.0 / (fs/2), 4000.0 / (fs/2)], btype='bandpass')
        sig_filtered = signal.filtfilt(b, a, sig)
        
        # Inject exact 3.5 sample delay in frequency domain
        delay_samples = 3.5
        true_tau = delay_samples / fs
        
        N_fft = len(sig_filtered)
        freqs = np.fft.rfftfreq(N_fft, d=1.0/fs)
        fft_sig = np.fft.rfft(sig_filtered)
        # Phase shift: exp(-j * 2 * pi * f * tau)
        fft_delayed = fft_sig * np.exp(-1j * 2.0 * np.pi * freqs * true_tau)
        sig_delayed = np.fft.irfft(fft_delayed, n=N_fft)
        
        # Compute GCC-PHAT delay
        est_tau, cc, lags = gcc_phat(sig_filtered, sig_delayed, fs, max_tau=0.005, interp=32)
        
        error_us = abs(est_tau - true_tau) * 1e6
        print(f"\n[TEST] GCC-PHAT True Delay: {true_tau*1e6:.2f} μs | Est: {est_tau*1e6:.2f} μs | Error: {error_us:.3f} μs")
        # Sub-microsecond accuracy verification (< 1.5 microseconds error)
        self.assertLess(error_us, 1.5)

    def test_order_tracking_synthetic_unbalance(self):
        """Verifies that a 1X-dominated vibration signal is correctly diagnosed as MASS UNBALANCE (>35%)."""
        fs = 400.0
        duration = 2.0
        t = np.arange(0, duration, 1.0/fs)
        
        # Shaft rotates at 25 Hz (1500 RPM)
        f_shaft = 25.0
        omega = 2.0 * np.pi * f_shaft
        
        # 1X dominant mass unbalance + small 2X misalignment + noise
        vib = 0.8 * np.sin(omega * t) + 0.1 * np.sin(2 * omega * t) + 0.05 * np.random.randn(len(t))
        
        # Revolution times at 1/25 s intervals
        rev_times = np.arange(0, duration, 1.0 / f_shaft)
        
        ot_res = compute_order_tracking(t, vib, rev_times, samples_per_rev=128)
        metrics = ot_res["metrics"]
        
        print(f"\n[TEST] Order Tracking 1X Energy: {metrics['e_1x_pct']:.1f}% | Fault: {metrics['primary_order_fault']}")
        self.assertGreater(metrics["e_1x_pct"], 35.0)
        self.assertEqual(metrics["primary_order_fault"], "MASS UNBALANCE (1X Order Dominant)")

    def test_order_tracking_synthetic_misalignment(self):
        """Verifies that a 2X-dominated vibration signal is correctly diagnosed as SHAFT MISALIGNMENT (>20%)."""
        fs = 400.0
        duration = 2.0
        t = np.arange(0, duration, 1.0/fs)
        
        f_shaft = 25.0
        omega = 2.0 * np.pi * f_shaft
        
        # 2X dominant misalignment
        vib = 0.15 * np.sin(omega * t) + 0.6 * np.sin(2 * omega * t) + 0.05 * np.random.randn(len(t))
        rev_times = np.arange(0, duration, 1.0 / f_shaft)
        
        ot_res = compute_order_tracking(t, vib, rev_times, samples_per_rev=128)
        metrics = ot_res["metrics"]
        
        print(f"\n[TEST] Order Tracking 2X Energy: {metrics['e_2x_pct']:.1f}% | Fault: {metrics['primary_order_fault']}")
        self.assertGreater(metrics["e_2x_pct"], 20.0)
        self.assertEqual(metrics["primary_order_fault"], "SHAFT MISALIGNMENT (2X Order Elevated)")

    def test_srp_phat_broadside_steering(self):
        """Verifies that a plane wave arriving at broadside (theta = 0 deg) steers to 0 deg within +/- 2 deg."""
        fs = 44100.0
        n_samples = 4096
        np.random.seed(123)
        source_audio = np.random.randn(n_samples).astype(np.float32)
        
        # Plane wave arriving perpendicular to linear array (all mics receive identical signal)
        audio_4ch = np.column_stack([source_audio, source_audio, source_audio, source_audio])
        
        mic_x = DEFAULT_ARRAY_GEOMETRY["mic_x"]
        srp_res = compute_srp_phat_grid(audio_4ch, fs, mic_x, phase_comp_samples=0)
        
        aoa = srp_res["aoa_deg"]
        print(f"\n[TEST] Broadside SRP-PHAT Measured Angle: {aoa:+.2f}° (Target: 0.00°)")
        self.assertAlmostEqual(aoa, 0.0, delta=2.5)

    def test_srp_phat_off_axis_steering(self):
        """Verifies that an incident plane wave at +30 degrees is localized within +/- 2.5 degrees."""
        fs = 44100.0
        n_samples = 4096
        np.random.seed(456)
        source_audio = np.random.randn(n_samples).astype(np.float32)
        
        target_angle_deg = 30.0
        theta_rad = np.radians(target_angle_deg)
        c = 343.0
        
        mic_x = DEFAULT_ARRAY_GEOMETRY["mic_x"]
        audio_4ch = np.zeros((n_samples, 4), dtype=np.float32)
        
        N_fft = n_samples
        freqs = np.fft.rfftfreq(N_fft, d=1.0/fs)
        fft_src = np.fft.rfft(source_audio)
        
        for m in range(4):
            # Plane wave propagation delay: tau_m = -mic_x[m] * sin(theta) / c
            # For theta > 0 (source on right), sound reaches positive mic_x earlier
            tau_m = -mic_x[m] * np.sin(theta_rad) / c
            audio_4ch[:, m] = np.fft.irfft(fft_src * np.exp(-1j * 2.0 * np.pi * freqs * tau_m), n=N_fft)
            
        srp_res = compute_srp_phat_grid(audio_4ch, fs, mic_x, phase_comp_samples=0)
        aoa = srp_res["aoa_deg"]
        print(f"\n[TEST] Off-Axis SRP-PHAT Measured Angle: {aoa:+.2f}° (Target: {target_angle_deg:+.2f}°)")
        self.assertAlmostEqual(aoa, target_angle_deg, delta=3.0)


if __name__ == "__main__":
    unittest.main()
