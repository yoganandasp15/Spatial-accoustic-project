"""
=============================================================================
 ADVERSARIAL STRESS TEST SUITE -- MILESTONE 1 VERIFICATION
=============================================================================
Module: tests/test_m1_adversarial_stress.py
Author: teamwork_challenger_m1_1 (Empirical Challenger)
Date: 2026-11-07
Scope: Adversarially stress-tests Milestone 1 DSP, Phase Calibration,
       Fractional Delay Compensation, Vibration Decoupling, and Order Tracking.
=============================================================================
"""

import os
import sys
import unittest
import tempfile
import numpy as np
import scipy.signal as signal

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    os.path.join(BASE_DIR, "src"),
    os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "src"),
    os.path.join(BASE_DIR, "stage2_acoustic_localization", "src"),
    os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

from calibrate_i2s_phase import gcc_phat, DEFAULT_ARRAY_GEOMETRY, SPEED_OF_SOUND
from offline_multimodal_evaluator import apply_pair_phase_compensation
from order_tracking import compute_order_tracking, run_order_tracking_on_csv


class TestM1AdversarialStress(unittest.TestCase):
    """Rigorous empirical stress tests challenging Milestone 1 implementation."""

    def test_calibrate_i2s_phase_angle_invariance(self):
        """
        Adversarial Stress Test: Verify that recovered I2S inter-pair DMA skew
        is strictly angle-invariant across extreme incident angles (-60°, -30°, 0°, +30°, +60°).
        Residual error must be < 0.1 μs.
        """
        fs = 44100.0
        n_samples = int(fs * 0.4)
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
            (0, 1, 'Intra-Pair A'),
            (2, 3, 'Intra-Pair B'),
            (1, 2, 'Inter-Pair Center'),
            (0, 3, 'Inter-Pair Outer'),
            (0, 2, 'Cross-Pair 1-3'),
            (1, 3, 'Cross-Pair 2-4')
        ]

        for angle in angles:
            theta_rad = np.radians(angle)
            audio = np.zeros((n_samples, 4), dtype=np.float32)
            for m in range(4):
                tau_acoustic = -mics[m]['x'] * np.sin(theta_rad) / SPEED_OF_SOUND
                total_delay = tau_acoustic
                if m in [2, 3]:
                    total_delay += -injected_hw_skew_sec
                ch_fft = source_fft * np.exp(-2j * np.pi * freqs * total_delay)
                audio[:, m] = np.fft.irfft(ch_fft, n=n_samples)

            skews_inter = []
            for m1, m2, label in pairs:
                theo_tau = (mics[m2]['x'] - mics[m1]['x']) * np.sin(theta_rad) / SPEED_OF_SOUND
                meas_tau, _, _ = gcc_phat(audio[:, m1], audio[:, m2], fs, max_tau=0.005, interp=32)
                skew_sec = meas_tau - theo_tau
                skew_us = skew_sec * 1e6
                if 'Inter-Pair' in label or 'Cross-Pair' in label:
                    skews_inter.append(skew_us)

            mean_skew = float(np.mean(skews_inter))
            residual_err_us = abs(mean_skew - injected_hw_skew_us)
            
            # Critical constraint: residual error < 0.1 μs
            self.assertLess(
                residual_err_us, 0.1,
                f"Angle {angle}° failed: Recovered {mean_skew:.4f} μs vs Injected {injected_hw_skew_us:.4f} μs (err: {residual_err_us:.4f} μs)"
            )

    def test_fractional_delay_across_sample_rates(self):
        """
        Adversarial Stress Test: Verify fractional delay phase compensation
        H(f) = exp(-j * 2 * pi * f * tau_comp) across 44.1 kHz, 16 kHz, and 11.025 kHz.
        Must eliminate sub-sample timing errors without amplitude scaling.
        """
        sample_rates = [44100.0, 16000.0, 11025.0]
        test_skews_us = [24.69, -15.35, 45.12, 5.50]

        for fs in sample_rates:
            dt = 1.0 / fs
            n_samples = int(fs * 0.2)
            f_high = min(4000.0, fs * 0.4)
            b, a = signal.butter(4, [300.0 / (fs/2), f_high / (fs/2)], btype='bandpass')
            np.random.seed(123)
            base_sig = signal.filtfilt(b, a, np.random.randn(n_samples))

            audio_orig = np.tile(base_sig[:, None], (1, 4))
            freqs = np.fft.rfftfreq(n_samples, d=dt)
            base_fft = np.fft.rfft(base_sig)

            for skew_us in test_skews_us:
                skew_sec = skew_us * 1e-6
                audio_skewed = audio_orig.copy()
                skew_shift = np.exp(-2j * np.pi * freqs * skew_sec)
                ch_skewed = np.fft.irfft(base_fft * skew_shift, n=n_samples)
                audio_skewed[:, 2] = ch_skewed
                audio_skewed[:, 3] = ch_skewed

                # Compensate with exact inverse delay
                audio_comp = apply_pair_phase_compensation(audio_skewed, fs, -skew_sec)

                # Timing recovery check via GCC-PHAT
                est_tau, _, _ = gcc_phat(audio_orig[:, 0], audio_comp[:, 2], fs, max_tau=0.005, interp=32)
                self.assertLess(abs(est_tau * 1e6), 0.05, f"Residual timing error too high at fs={fs}: {est_tau*1e6:.4f} μs")

                # Amplitude preservation check (interior)
                interior = slice(128, n_samples - 128)
                amp_ratio = np.std(audio_comp[interior, 2]) / np.std(audio_orig[interior, 2])
                self.assertAlmostEqual(amp_ratio, 1.0, places=3, msg=f"Amplitude scaled at fs={fs}: ratio={amp_ratio:.5f}")

    def test_dynamic_vibration_dc_removal_vs_rectification(self):
        """
        Adversarial Stress Test: Compare per-axis DC removal against full-wave Euclidean rectification.
        Input: 1g static gravity on Z, 0.05g lateral oscillation on X at 25 Hz.
        Confirms Method 1 recovers 25 Hz at 0.05g without doubling, while Method 2 doubles to 50 Hz.
        """
        fs = 400.0
        duration = 4.0
        t = np.arange(0, duration, 1.0 / fs)
        f_rot = 25.0
        amp_rot = 0.05

        ax = amp_rot * np.sin(2.0 * np.pi * f_rot * t)
        ay = np.zeros_like(t)
        az = -1.0 * np.ones_like(t)

        # Method 1: Per-axis DC removal
        ax_dyn = ax - np.mean(ax)
        N = len(t)
        freqs = np.fft.rfftfreq(N, d=1.0/fs)
        fft_ax = np.abs(np.fft.rfft(ax_dyn)) * (2.0 / N)

        peak_idx_m1 = np.argmax(fft_ax[1:]) + 1
        peak_freq_m1 = freqs[peak_idx_m1]
        peak_amp_m1 = fft_ax[peak_idx_m1]

        idx_25hz = np.argmin(np.abs(freqs - 25.0))
        idx_50hz = np.argmin(np.abs(freqs - 50.0))
        e_25hz_m1 = fft_ax[idx_25hz]
        e_50hz_m1 = fft_ax[idx_50hz]

        self.assertAlmostEqual(peak_freq_m1, 25.0, delta=0.05, msg="Method 1 failed to detect 25 Hz peak")
        self.assertAlmostEqual(peak_amp_m1, 0.05, delta=0.001, msg="Method 1 failed to recover 0.05g amplitude")
        self.assertLess(e_50hz_m1, 1e-4, "Method 1 contains unexpected 50 Hz harmonic")

        # Method 2: Rectified Euclidean norm
        a_mag = np.sqrt(ax**2 + ay**2 + az**2)
        a_mag_dyn = a_mag - np.mean(a_mag)
        fft_mag = np.abs(np.fft.rfft(a_mag_dyn)) * (2.0 / N)
        peak_idx_m2 = np.argmax(fft_mag[1:]) + 1
        peak_freq_m2 = freqs[peak_idx_m2]
        peak_amp_m2 = fft_mag[peak_idx_m2]

        self.assertAlmostEqual(peak_freq_m2, 50.0, delta=0.05, msg="Method 2 did not demonstrate frequency doubling")
        self.assertLess(peak_amp_m2, 0.001, "Method 2 amplitude not crushed")

    def test_order_tracking_interval_counting_and_absent_tach(self):
        """
        Adversarial Stress Test: Verify revolution interval counting across N=2, 3, 5, 10, 50.
        Reported revolutions must always be N - 1, and 1X peak locked at 1.000.
        Verify that absent/insufficient tachometer pulses report UNAVAILABLE.
        """
        fs = 4000.0
        f_shaft = 25.0
        t_rev = 1.0 / f_shaft

        for N in [2, 3, 5, 10, 50]:
            rev_times = np.arange(N) * t_rev
            t_vib = np.arange(0, rev_times[-1] + 0.01, 1.0 / fs)
            vib_sig = 0.5 * np.sin(2.0 * np.pi * f_shaft * t_vib)

            res = compute_order_tracking(t_vib, vib_sig, rev_times=rev_times, samples_per_rev=128)
            self.assertEqual(res["total_revolutions"], N - 1, f"Failed for N={N}")
            
            peak_order = res["orders"][np.argmax(res["order_mag"])]
            self.assertAlmostEqual(peak_order, 1.0, delta=0.01, msg=f"Order lock failed for N={N}: {peak_order}")

        # Boundary checks for N=0 and N=1
        with self.assertRaises(ValueError):
            compute_order_tracking(np.arange(100)/100.0, np.zeros(100), rev_times=np.array([]))

        with self.assertRaises(ValueError):
            compute_order_tracking(np.arange(100)/100.0, np.zeros(100), rev_times=np.array([0.05]))

        # Check absent tachometer handling on CSV
        repo_csv = os.path.join(BASE_DIR, "stage1_vibration_and_tachometer", "evidence", "multimodal_4mic_vibration.csv")
        if os.path.exists(repo_csv):
            with tempfile.TemporaryDirectory() as tmpdir:
                metrics = run_order_tracking_on_csv(repo_csv, tmpdir)
                self.assertEqual(metrics.get("status"), "UNAVAILABLE")
                self.assertEqual(metrics.get("mean_rpm"), 0.0)
                self.assertIn("UNAVAILABLE", metrics.get("primary_order_fault"))


if __name__ == "__main__":
    unittest.main()
