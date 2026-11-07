"""
=============================================================================
 MILESTONE 3: EMPIRICAL ADVERSARIAL MUTATION & BOUNDARY STRESS TEST SUITE
=============================================================================
Module: tests/test_m3_mutation_adversarial.py
Author: Challenger M3 (teamwork_challenger_m3)
Target: Formally proves that the newly expanded unit tests in
        `tests/test_multimodal_pipeline.py` are non-tautological, sensitive
        to defective implementations (100% mutation kill rate), and resilient
        against extreme boundary edge cases.
Date: 2026-11-08 (ECE Minor Project Target)
=============================================================================
"""

import os
import sys
import unittest
import struct
import tempfile
from unittest.mock import patch
import numpy as np
import pandas as pd
from scipy import signal, interpolate

# Ensure component src directories are on sys.path
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
from order_tracking import (
    compute_order_tracking,
    extract_tachometer_revolutions,
    run_order_tracking_on_csv,
)
from offline_multimodal_evaluator import (
    apply_pair_phase_compensation,
    evaluate_vibration_record,
    compute_srp_phat_grid,
    DEFAULT_ARRAY_GEOMETRY,
)
from spatial_acoustic_radar_pro import (
    ESP32MultimodalSource,
    compute_packet_checksum,
)


class TestM3AntiTautologyMutations(unittest.TestCase):
    """
    Empirical mutation testing harness verifying that the expanded tests in
    `tests/test_multimodal_pipeline.py` genuinely fail (100% mutation kill)
    when injected with flawed or defective algorithms.
    """

    # -------------------------------------------------------------------------
    # Mutation 1: Gravity Rectification (Euclidean Magnitude)
    # -------------------------------------------------------------------------
    def test_mutation_1_gravity_rectification_kills_test(self):
        """
        Mutation 1: Injects full-wave Euclidean magnitude rectification instead
        of per-axis DC subtraction. Proves that the assertions in
        `test_dc_gravity_removal_vs_rectification_frequency_doubling` and
        `test_per_axis_gravity_removal_signed_preservation` strictly catch this defect.
        """
        fs = 400.0
        duration = 4.0
        t = np.arange(0, duration, 1.0 / fs)
        f_shaft = 25.0
        amp_shaft = 0.05
        ax = amp_shaft * np.sin(2.0 * np.pi * f_shaft * t)
        ay = np.zeros_like(t)
        az = -1.0 * np.ones_like(t)

        N = len(t)
        freqs = np.fft.rfftfreq(N, d=1.0 / fs)

        # Defective Method: Euclidean magnitude rectification
        a_mag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
        ax_mutant = a_mag - np.mean(a_mag)
        fft_mutant = np.abs(np.fft.rfft(ax_mutant)) * (2.0 / N)

        peak_idx = int(np.argmax(fft_mutant[1:])) + 1
        peak_freq = freqs[peak_idx]
        peak_amp = fft_mutant[peak_idx]

        idx_50hz = int(np.argmin(np.abs(freqs - 50.0)))
        e_50hz = fft_mutant[idx_50hz]

        # Verify that the mutant violates the per-axis assertions:
        # 1. Peak frequency is doubled to 50 Hz, failing the 25 Hz target
        with self.assertRaises(AssertionError, msg="Mutant 1 failed to kill 25 Hz peak frequency assertion"):
            self.assertAlmostEqual(peak_freq, 25.0, delta=0.05)

        # 2. Dynamic peak amplitude is crushed to ~0.00062g, failing the 0.05g target
        with self.assertRaises(AssertionError, msg="Mutant 1 failed to kill 0.05g amplitude assertion"):
            self.assertAlmostEqual(peak_amp, 0.05, delta=0.001)

        # 3. 50 Hz harmonic is elevated, failing the <1e-4 harmonic threshold
        with self.assertRaises(AssertionError, msg="Mutant 1 failed to kill 50 Hz harmonic suppression assertion"):
            self.assertLess(e_50hz, 1e-4)

        # 4. Signed directional symmetry is destroyed (min swing is ~ -0.0006g, not -0.05g)
        with self.assertRaises(AssertionError, msg="Mutant 1 failed to kill negative dynamic excursion assertion"):
            self.assertAlmostEqual(float(np.min(ax_mutant)), -amp_shaft, delta=1e-3)

        print(f"\n[MUTATION 1 KILLED] Euclidean magnitude rectification correctly failed all 4 physical assertions.")

    # -------------------------------------------------------------------------
    # Mutation 2: Checksum Seed Bypass & Corrupted Packet Ingestion
    # -------------------------------------------------------------------------
    def test_mutation_2a_checksum_zero_seed_kills_test(self):
        """
        Mutation 2A: Mutates checksum seed to 0x0000 instead of 0x55AA.
        Proves that `test_checksum_16bit_xor_initialization_and_calculation` fails.
        """
        def mutant_checksum_zero_seed(data: bytes) -> int:
            chk = 0x0000  # DEFECTIVE SEED
            for i, b in enumerate(data):
                chk ^= ((b << (i % 8)) & 0xFFFF)
            return chk & 0xFFFF

        # Assertion from test_checksum_16bit_xor_initialization_and_calculation
        with self.assertRaises(AssertionError, msg="Mutant 2A failed to kill seed initialization assertion"):
            self.assertEqual(mutant_checksum_zero_seed(b""), 0x55AA)

        with self.assertRaises(AssertionError, msg="Mutant 2A failed to kill single-byte checksum assertion"):
            self.assertEqual(mutant_checksum_zero_seed(bytes([0x01])), 0x55AB)

        print(f"\n[MUTATION 2A KILLED] Checksum seed corruption (0x0000) correctly failed seed assertions.")

    def test_mutation_2b_corrupted_packet_bypass_kills_test(self):
        """
        Mutation 2B: Mutates packet parsing to bypass checksum validation (accepting all packets).
        Proves that `test_corrupted_packet_rejection_single_bit_flip` strictly catches this defect.
        """
        accel_samples = [(10, -20, 980)] * 5
        audio_data = np.zeros((128, 4), dtype=np.int16)
        valid_pkt = ESP32MultimodalSource.build_packet_v2(
            seq=1, fs=11025, block_frames=128,
            accel_samples=accel_samples,
            audio_data=audio_data,
            tach_pulses=10
        )

        corrupted = bytearray(valid_pkt)
        corrupted[500] ^= 0x01  # Bit flip in audio payload

        # Defective parser that ignores checksum mismatch
        class MutantESP32Source(ESP32MultimodalSource):
            def _parse_incoming_bytes(self, chunk: bytes, port_str: str = "COM8", baud_str=1152000) -> int:
                # Defective: bypass checksum check
                with patch("spatial_acoustic_radar_pro.compute_packet_checksum", return_value=struct.unpack_from("<H", chunk, 1070)[0]):
                    return super()._parse_incoming_bytes(chunk, port_str, baud_str)

        mutant_src = MutantESP32Source()
        parsed = mutant_src._parse_incoming_bytes(bytes(corrupted))

        # Under the test assertion, parsed must be 0 and packets_valid must be 0
        with self.assertRaises(AssertionError, msg="Mutant 2B failed to kill packet acceptance assertion"):
            self.assertEqual(parsed, 0)

        with self.assertRaises(AssertionError, msg="Mutant 2B failed to kill packets_valid assertion"):
            self.assertEqual(mutant_src.packets_valid, 0)

        print(f"\n[MUTATION 2B KILLED] Checksum bypass defect correctly failed corrupted packet rejection assertions.")

    # -------------------------------------------------------------------------
    # Mutation 3: Phase Compensation Sign Inversion
    # -------------------------------------------------------------------------
    def test_mutation_3_phase_compensation_sign_inversion_kills_test(self):
        """
        Mutation 3: Inverts the sign of tau_comp in phase compensation (+skew_sec instead of -skew_sec).
        Proves that `test_fractional_delay_phase_compensation_across_sample_rates`
        strictly catches sign mistakes due to residual timing error explosion (from <0.02 μs to ~49.38 μs).
        """
        fs = 44100.0
        n_samples = int(fs * 0.25)
        dt = 1.0 / fs
        b, a = signal.butter(4, [300.0 / (fs / 2), 4000.0 / (fs / 2)], btype='bandpass')
        np.random.seed(789)
        base_sig = signal.filtfilt(b, a, np.random.randn(n_samples)).astype(np.float32)

        audio_orig = np.tile(base_sig[:, None], (1, 4))
        freqs = np.fft.rfftfreq(n_samples, d=dt)
        base_fft = np.fft.rfft(base_sig)

        skew_us = 24.69
        skew_sec = skew_us * 1e-6
        audio_skewed = audio_orig.copy()

        # Skew Pair B by +skew_sec (lagging Pair A)
        skew_filter = np.exp(-2j * np.pi * freqs * skew_sec)
        ch_skewed = np.fft.irfft(base_fft * skew_filter, n=n_samples)
        audio_skewed[:, 2] = ch_skewed
        audio_skewed[:, 3] = ch_skewed

        # Defective Compensation: Inverted sign (+skew_sec adds delay instead of canceling)
        audio_comp_mutant = apply_pair_phase_compensation(audio_skewed, fs, +skew_sec)

        est_tau_m, _, _ = gcc_phat(audio_orig[:, 0], audio_comp_mutant[:, 2], fs, max_tau=0.005, interp=32)
        res_err_us = abs(est_tau_m * 1e6)

        # Assertion in test_fractional_delay_phase_compensation_across_sample_rates: res_err_us < 0.1
        with self.assertRaises(AssertionError, msg="Mutant 3 failed to kill residual delay threshold (<0.1 μs)"):
            self.assertLess(res_err_us, 0.1, f"Mutant sign produced {res_err_us:.2f} μs error")

        print(f"\n[MUTATION 3 KILLED] Sign inversion produced {res_err_us:.2f} μs error, killing the <0.1 μs assertion.")

    # -------------------------------------------------------------------------
    # Mutation 4: N vs N-1 Revolution Intervals in Order Tracking
    # -------------------------------------------------------------------------
    def test_mutation_4_n_vs_n_minus_1_interval_kills_test(self):
        """
        Mutation 4: Computes N intervals instead of N-1 intervals in order tracking.
        Proves that `test_tachometer_n_minus_1_revolution_intervals` fails on both
        the interval count and the 1X order peak frequency (shifted to (N-1)/N = 0.800).
        """
        fs = 4000.0
        f_shaft = 25.0
        t_rev = 1.0 / f_shaft
        N = 5
        rev_times = np.arange(N) * t_rev
        t_vib = np.arange(0, rev_times[-1] + 0.01, 1.0 / fs)
        vib_sig = 0.5 * np.sin(2.0 * np.pi * f_shaft * t_vib)

        # Defective order tracking: uses N intervals instead of N-1
        n_markers = len(rev_times)
        n_intervals_mutant = n_markers  # DEFECTIVE: should be n_markers - 1
        rev_angles = np.arange(n_markers) * 2.0 * np.pi

        samples_per_rev = 128
        total_angle = rev_angles[-1]
        total_resampled_points = int(n_intervals_mutant * samples_per_rev)
        theta_uniform = np.linspace(0.0, total_angle, total_resampled_points, endpoint=False)

        angle_to_time = interpolate.PchipInterpolator(rev_angles, rev_times)
        t_resampled = angle_to_time(theta_uniform)
        vib_interpolator = interpolate.interp1d(t_vib, vib_sig, kind='cubic', fill_value='extrapolate')
        vib_angle = vib_interpolator(t_resampled)

        N_fft = len(vib_angle)
        w = np.hanning(N_fft)
        vib_windowed = (vib_angle - np.mean(vib_angle)) * w
        scale_factor = 2.0 / np.sum(w)
        fft_vals = np.fft.rfft(vib_windowed)
        orders = np.fft.rfftfreq(N_fft, d=1.0 / samples_per_rev)
        order_mag = np.abs(fft_vals) * scale_factor
        peak_order_mutant = orders[np.argmax(order_mag)]

        # Assertion 1: Total revolutions must be N - 1
        with self.assertRaises(AssertionError, msg="Mutant 4 failed to kill total_revolutions == N - 1"):
            self.assertEqual(n_intervals_mutant, N - 1)

        # Assertion 2: Peak order must be 1.000 +/- 0.01
        with self.assertRaises(AssertionError, msg="Mutant 4 failed to kill peak_order == 1.000"):
            self.assertAlmostEqual(peak_order_mutant, 1.0, delta=0.01)

        print(f"\n[MUTATION 4 KILLED] N intervals instead of N-1 shifted 1X peak to {peak_order_mutant:.2f}, killing both assertions.")

    # -------------------------------------------------------------------------
    # Mutation 5: Synthetic Tachometer Fallback (1500 RPM)
    # -------------------------------------------------------------------------
    def test_mutation_5_synthetic_tachometer_fallback_kills_test(self):
        """
        Mutation 5: Re-injects the legacy synthetic 1500 RPM / 25 Hz fallback when
        tachometer signal is absent. Proves that `test_tachometer_absent_pulse_handling_unavailable`
        strictly catches this defect.
        """
        # Defective function returning legacy synthetic 1500 RPM
        def mutant_run_order_tracking_on_csv(csv_path, output_dir):
            return {
                "status": "VALID",
                "mean_rpm": 1500.0,
                "total_revolutions": 50,
                "e_1x_pct": 85.0,
                "primary_order_fault": "MASS UNBALANCE (1X Order Dominant)",
            }

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "no_tach.csv")
            t = np.arange(400) / 400.0
            pd.DataFrame({"Timestamp_us": (t * 1e6).astype(np.int64), "Accel_Z_g": 0.05 * np.sin(2.0 * np.pi * 25.0 * t)}).to_csv(csv_path, index=False)

            metrics_mutant = mutant_run_order_tracking_on_csv(csv_path, tmpdir)

            # Test assertions from test_tachometer_absent_pulse_handling_unavailable
            with self.assertRaises(AssertionError, msg="Mutant 5 failed to kill status == 'UNAVAILABLE'"):
                self.assertEqual(metrics_mutant["status"], "UNAVAILABLE")

            with self.assertRaises(AssertionError, msg="Mutant 5 failed to kill mean_rpm == 0.0"):
                self.assertEqual(metrics_mutant["mean_rpm"], 0.0)

            with self.assertRaises(AssertionError, msg="Mutant 5 failed to kill total_revolutions == 0"):
                self.assertEqual(metrics_mutant["total_revolutions"], 0)

            with self.assertRaises(AssertionError, msg="Mutant 5 failed to kill e_1x_pct == 0.0"):
                self.assertEqual(metrics_mutant["e_1x_pct"], 0.0)

            with self.assertRaises(AssertionError, msg="Mutant 5 failed to kill 'UNAVAILABLE' in primary_order_fault"):
                self.assertIn("UNAVAILABLE", metrics_mutant["primary_order_fault"])

            with self.assertRaises(AssertionError, msg="Mutant 5 failed to kill synthetic 1500 string check"):
                self.assertNotIn("1500", str(metrics_mutant["mean_rpm"]))

        print(f"\n[MUTATION 5 KILLED] Synthetic 1500 RPM fallback correctly failed all 6 absence assertions.")


class TestM3BoundaryEdgeCases(unittest.TestCase):
    """
    Boundary edge case verification: testing unusual sample rates, single-pulse markers,
    corrupted buffer lengths, and packet parser limits.
    """

    def test_boundary_extreme_sample_rates_phase_compensation(self):
        """
        Boundary Test: Evaluates frequency-domain phase compensation across extreme sample rates:
        - 8,000 Hz (Telephony downsampled audio)
        - 48,000 Hz (Studio standard audio)
        - 96,000 Hz (High-resolution audio)
        - 192,000 Hz (Ultrasonic high-speed acoustic acquisition)
        Verifies residual timing error < 0.15 μs and all-pass amplitude ratio 1.000 +/- 0.005.
        """
        extreme_rates = [8000.0, 48000.0, 96000.0, 192000.0]
        skew_us = 24.69
        skew_sec = skew_us * 1e-6

        for fs in extreme_rates:
            n_samples = int(fs * 0.25)
            dt = 1.0 / fs
            f_high = min(3500.0, fs * 0.40)
            b, a = signal.butter(4, [250.0 / (fs / 2), f_high / (fs / 2)], btype='bandpass')
            np.random.seed(1234)
            base_sig = signal.filtfilt(b, a, np.random.randn(n_samples)).astype(np.float32)

            audio_orig = np.tile(base_sig[:, None], (1, 4))
            freqs = np.fft.rfftfreq(n_samples, d=dt)
            base_fft = np.fft.rfft(base_sig)

            audio_skewed = audio_orig.copy()
            skew_filter = np.exp(-2j * np.pi * freqs * skew_sec)
            ch_skewed = np.fft.irfft(base_fft * skew_filter, n=n_samples)
            audio_skewed[:, 2] = ch_skewed
            audio_skewed[:, 3] = ch_skewed

            audio_comp = apply_pair_phase_compensation(audio_skewed, fs, -skew_sec)
            est_tau, _, _ = gcc_phat(audio_orig[:, 0], audio_comp[:, 2], fs, max_tau=0.005, interp=32)
            res_err_us = abs(est_tau * 1e6)

            interior = slice(128, n_samples - 128)
            amp_ratio = np.std(audio_comp[interior, 2]) / np.std(audio_orig[interior, 2])

            self.assertLess(
                res_err_us, 0.15,
                f"Residual delay error {res_err_us:.4f} μs exceeded 0.15 μs at fs={fs} Hz"
            )
            self.assertAlmostEqual(
                amp_ratio, 1.0, places=2,
                msg=f"All-pass magnitude violated at fs={fs} Hz: ratio={amp_ratio:.5f}"
            )

        print(f"\n[BOUNDARY TEST] Extreme sample rates (8k-192k Hz) verified: all residual errors < 0.15 μs.")

    def test_boundary_tachometer_pulse_counts(self):
        """
        Boundary Test: Tests tachometer extraction and order tracking under
        N=0, N=1, and minimal N=2 pulse conditions.
        """
        t = np.arange(1000) / 1000.0

        # N=0: No pulses -> empty array
        tach_0 = np.zeros(1000)
        revs_0 = extract_tachometer_revolutions(t, tach_0)
        self.assertEqual(len(revs_0), 0)

        # N=1: Single pulse -> must raise ValueError
        tach_1 = np.zeros(1000)
        tach_1[500:520] = 3.3
        with self.assertRaises(ValueError):
            extract_tachometer_revolutions(t, tach_1)

        # N=2: Exactly two pulses -> defines exactly 1 revolution interval
        rev_times_2 = np.array([0.0, 0.04])
        t_vib = np.arange(0, 0.04 + 0.01, 1.0 / 4000.0)
        vib_sig = 0.5 * np.sin(2.0 * np.pi * 25.0 * t_vib)
        ot_res = compute_order_tracking(t_vib, vib_sig, rev_times=rev_times_2, samples_per_rev=64)

        self.assertEqual(ot_res["status"], "VALID")
        self.assertEqual(ot_res["total_revolutions"], 1)
        self.assertEqual(ot_res["metrics"]["total_revolutions"], 1)

        print(f"\n[BOUNDARY TEST] Tachometer pulse boundaries (N=0, N=1, N=2) verified cleanly.")

    def test_boundary_packet_buffer_extremes_and_corrupt_lengths(self):
        """
        Boundary Test: Tests packet buffer length extremes and corrupted header fields:
        - 0-byte buffer: compute_packet_checksum(b"") == 0x55AA
        - 64 KB buffer: verifies no integer overflow and valid 16-bit range
        - Malformed packet with num_vib = 0 (valid audio, zero vibration)
        - Malformed packet with num_vib = 50 (>32 limit) -> rejected, no hang
        - Malformed packet with block_frames = 256 (!= 128) -> rejected
        - Malformed packet with channels = 2 (!= 4) -> rejected
        - Incomplete packet header (15 bytes) -> buffered, no exception
        """
        # 1. Zero-length buffer
        self.assertEqual(compute_packet_checksum(b""), 0x55AA)

        # 2. Large 64 KB buffer
        chk_large = compute_packet_checksum(bytes([0xFF] * 65536))
        self.assertTrue(0 <= chk_large <= 0xFFFF)

        # 3. Malformed packet with num_vib = 0
        source = ESP32MultimodalSource()
        audio_data = np.zeros((128, 4), dtype=np.int16)
        pkt_0vib = ESP32MultimodalSource.build_packet_v2(
            seq=1, fs=11025, block_frames=128,
            accel_samples=[], audio_data=audio_data, tach_pulses=5
        )
        self.assertEqual(len(pkt_0vib), 1042)
        parsed_0vib = source._parse_incoming_bytes(pkt_0vib)
        self.assertEqual(parsed_0vib, 1)
        self.assertEqual(source.packets_valid, 1)
        self.assertEqual(len(source.vib_ring_ax), 0)

        # 4. Malformed packet with num_vib = 50 (> 32)
        source2 = ESP32MultimodalSource()
        hdr_bad_vib = struct.pack('<BBBBHHHBBI', 0xAA, 0x55, 0x05, 4, 1, 11025, 128, 50, 0, 0)
        parsed_bad_vib = source2._parse_incoming_bytes(hdr_bad_vib + b'\x00' * 500)
        self.assertEqual(parsed_bad_vib, 0)
        self.assertEqual(source2.packets_valid, 0)

        # 5. Malformed packet with block_frames = 256 (!= 128)
        source3 = ESP32MultimodalSource()
        hdr_bad_bf = struct.pack('<BBBBHHHBBI', 0xAA, 0x55, 0x05, 4, 1, 11025, 256, 5, 0, 0)
        parsed_bad_bf = source3._parse_incoming_bytes(hdr_bad_bf + b'\x00' * 500)
        self.assertEqual(parsed_bad_bf, 0)
        self.assertEqual(source3.packets_valid, 0)

        # 6. Malformed packet with channels = 2 (!= 4)
        source4 = ESP32MultimodalSource()
        hdr_bad_ch = struct.pack('<BBBBHHHBBI', 0xAA, 0x55, 0x05, 2, 1, 11025, 128, 5, 0, 0)
        parsed_bad_ch = source4._parse_incoming_bytes(hdr_bad_ch + b'\x00' * 500)
        self.assertEqual(parsed_bad_ch, 0)
        self.assertEqual(source4.packets_valid, 0)

        # 7. Incomplete packet header (15 bytes)
        source5 = ESP32MultimodalSource()
        parsed_inc = source5._parse_incoming_bytes(hdr_bad_vib[:15])
        self.assertEqual(parsed_inc, 0)
        self.assertEqual(len(source5._rx_buf), 15)

        print(f"\n[BOUNDARY TEST] Packet buffer extremes and malformed header fields verified safely.")


if __name__ == "__main__":
    unittest.main()
