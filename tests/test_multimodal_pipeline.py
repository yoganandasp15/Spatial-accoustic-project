"""
=============================================================================
 AUTOMATED UNIT & INTEGRATION TEST SUITE FOR MULTIMODAL PIPELINE
=============================================================================
Module: tests/test_multimodal_pipeline.py
Target: Validates DSP, acoustic localization, phase compensation across
        sample rates, per-axis gravity decoupling, Type 0x05 checksum
        integrity, and rotational order tracking (Milestone 3 / Requirement R4).
Author: Antigravity Autonomous Agent (Pair Programming with Yogi)
Date: 2026-11-08 (ECE Minor Project Target)
=============================================================================
"""

import os
import sys
import unittest
import tempfile
import struct
import numpy as np
import pandas as pd
from scipy import signal

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
from order_tracking import (
    compute_order_tracking,
    extract_tachometer_revolutions,
    run_order_tracking_on_csv,
)
from offline_multimodal_evaluator import (
    compute_srp_phat_grid,
    apply_pair_phase_compensation,
    evaluate_vibration_record,
    DEFAULT_ARRAY_GEOMETRY,
    SPEED_OF_SOUND,
)
from spatial_acoustic_radar_pro import (
    ESP32MultimodalSource,
    compute_packet_checksum,
)


class TestMultimodalPipeline(unittest.TestCase):
    """
    Comprehensive automated unit test suite for the multimodal acoustic-vibration pipeline.
    All tests in this class are 100% synthetic and run offline without physical hardware.
    """

    # =========================================================================
    # 1. ACOUSTIC LOCALIZATION & GCC-PHAT TESTS (Existing Baseline)
    # =========================================================================

    def test_gcc_phat_synthetic_delay(self):
        """Verifies GCC-PHAT delay estimation on synthetic bandpass signals with known sub-sample shift."""
        fs = 44100.0
        n_samples = 4096
        np.random.seed(42)
        sig = np.random.randn(n_samples).astype(np.float32)

        # Bandpass filter between 500 Hz and 4000 Hz
        b, a = signal.butter(4, [500.0 / (fs/2), 4000.0 / (fs/2)], btype='bandpass')
        sig_filtered = signal.filtfilt(b, a, sig)

        # Inject exact 3.5 sample delay in frequency domain
        delay_samples = 3.5
        true_tau = delay_samples / fs

        N_fft = len(sig_filtered)
        freqs = np.fft.rfftfreq(N_fft, d=1.0/fs)
        fft_sig = np.fft.rfft(sig_filtered)
        fft_delayed = fft_sig * np.exp(-1j * 2.0 * np.pi * freqs * true_tau)
        sig_delayed = np.fft.irfft(fft_delayed, n=N_fft)

        # Compute GCC-PHAT delay
        est_tau, cc, lags = gcc_phat(sig_delayed, sig_filtered, fs, max_tau=0.005, interp=32)

        error_us = abs(est_tau - true_tau) * 1e6
        print(f"\n[TEST] GCC-PHAT True Delay: {true_tau*1e6:.2f} μs | Est: {est_tau*1e6:.2f} μs | Error: {error_us:.3f} μs")
        # Sub-microsecond accuracy verification (< 1.5 microseconds error)
        self.assertLess(error_us, 1.5)

    def test_srp_phat_broadside_steering(self):
        """Verifies that a plane wave arriving at broadside (theta = 0 deg) steers to 0 deg within +/- 2.5 deg."""
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
        """Verifies that an incident plane wave at +30 degrees is localized within +/- 3.0 degrees."""
        fs = 44100.0
        n_samples = 4096
        np.random.seed(456)
        source_audio = np.random.randn(n_samples).astype(np.float32)

        target_angle_deg = 30.0
        theta_rad = np.radians(target_angle_deg)
        c = SPEED_OF_SOUND

        mic_x = DEFAULT_ARRAY_GEOMETRY["mic_x"]
        audio_4ch = np.zeros((n_samples, 4), dtype=np.float32)

        N_fft = n_samples
        freqs = np.fft.rfftfreq(N_fft, d=1.0/fs)
        fft_src = np.fft.rfft(source_audio)

        for m in range(4):
            # Plane wave propagation delay: tau_m = -mic_x[m] * sin(theta) / c
            tau_m = -mic_x[m] * np.sin(theta_rad) / c
            audio_4ch[:, m] = np.fft.irfft(fft_src * np.exp(-1j * 2.0 * np.pi * freqs * tau_m), n=N_fft)

        srp_res = compute_srp_phat_grid(audio_4ch, fs, mic_x, phase_comp_samples=0)
        aoa = srp_res["aoa_deg"]
        print(f"\n[TEST] Off-Axis SRP-PHAT Measured Angle: {aoa:+.2f}° (Target: {target_angle_deg:+.2f}°)")
        self.assertAlmostEqual(aoa, target_angle_deg, delta=3.0)

    # =========================================================================
    # 2. PER-AXIS GRAVITY REMOVAL & DYNAMIC COMPONENT ISOLATION (R4.a)
    # =========================================================================

    def test_per_axis_gravity_removal_signed_preservation(self):
        """
        R4.a: Verifies that per-axis DC gravity removal (a_k - mean(a_k)) preserves
        signed lateral vibration signals without full-wave magnitude rectification.
        Negative dynamic swings must be preserved symmetrically.
        """
        fs = 400.0  # 400 Hz ADXL345 sampling rate
        duration = 2.0
        t = np.arange(0, duration, 1.0 / fs)
        f_vib = 25.0
        amp_vib = 0.05  # 0.05g dynamic oscillation

        # Lateral oscillation on X with zero mean, Y quiet, static gravity on Z (-1.0g)
        ax = amp_vib * np.sin(2.0 * np.pi * f_vib * t)
        ay = np.zeros_like(t)
        az = -1.0 + 0.02 * np.cos(2.0 * np.pi * f_vib * t)  # 1g static bias + small AC

        # Per-axis DC removal
        ax_dyn = ax - np.mean(ax)
        ay_dyn = ay - np.mean(ay)
        az_dyn = az - np.mean(az)

        # 1. Preserves signed directional excursion (both positive and negative)
        self.assertAlmostEqual(float(np.min(ax_dyn)), -amp_vib, delta=1e-3)
        self.assertAlmostEqual(float(np.max(ax_dyn)), +amp_vib, delta=1e-3)
        self.assertGreater(int(np.sum(ax_dyn < 0)), int(0.45 * len(t)))
        self.assertGreater(int(np.sum(ax_dyn > 0)), int(0.45 * len(t)))

        # 2. Dynamic RMS matches theoretical expectation: A / sqrt(2)
        rms_expected = amp_vib / np.sqrt(2.0)
        rms_observed = float(np.sqrt(np.mean(ax_dyn ** 2)))
        self.assertAlmostEqual(rms_observed, rms_expected, delta=1e-3)

        # 3. Static gravity decoupled: az_dyn centered at 0.0
        self.assertAlmostEqual(float(np.mean(az_dyn)), 0.0, delta=1e-6)
        print(f"\n[TEST] Signed Dynamic Vibration: Peak={np.max(ax_dyn):+.4f}g, Min={np.min(ax_dyn):+.4f}g, RMS={rms_observed:.4f}g")

    def test_dc_gravity_removal_vs_rectification_frequency_doubling(self):
        """
        R4.a: Verifies that per-axis DC gravity removal recovers true fundamental frequency (25 Hz)
        at 0.05g without harmonic distortion, whereas Euclidean magnitude rectification
        (sqrt(ax^2 + ay^2 + az^2)) doubles the frequency to 50 Hz and crushes amplitude by ~80x.
        """
        fs = 400.0
        duration = 4.0
        t = np.arange(0, duration, 1.0 / fs)
        f_shaft = 25.0
        amp_shaft = 0.05  # 0.05g amplitude

        # Dynamic lateral unbalance on X, static gravity (-1g) on Z
        ax = amp_shaft * np.sin(2.0 * np.pi * f_shaft * t)
        ay = np.zeros_like(t)
        az = -1.0 * np.ones_like(t)

        N = len(t)
        freqs = np.fft.rfftfreq(N, d=1.0 / fs)

        # Method 1: Proper per-axis DC gravity removal
        ax_dyn = ax - np.mean(ax)
        fft_ax = np.abs(np.fft.rfft(ax_dyn)) * (2.0 / N)

        peak_idx_m1 = int(np.argmax(fft_ax[1:])) + 1
        peak_freq_m1 = freqs[peak_idx_m1]
        peak_amp_m1 = fft_ax[peak_idx_m1]

        idx_50hz = int(np.argmin(np.abs(freqs - 50.0)))
        e_50hz_m1 = fft_ax[idx_50hz]

        # Method 1 must detect 25.0 Hz fundamental at 0.05g with NO 50 Hz harmonic
        self.assertAlmostEqual(peak_freq_m1, 25.0, delta=0.05)
        self.assertAlmostEqual(peak_amp_m1, 0.05, delta=0.001)
        self.assertLess(e_50hz_m1, 1e-4, "Per-axis isolation produced unexpected 50 Hz harmonic")

        # Method 2: Defective Euclidean magnitude rectification
        # sqrt((-1)^2 + ax^2) ~= 1 + 0.5 * ax^2 = 1 + (A^2/4) * (1 - cos(2*omega*t))
        a_mag = np.sqrt(ax ** 2 + ay ** 2 + az ** 2)
        a_mag_dyn = a_mag - np.mean(a_mag)
        fft_mag = np.abs(np.fft.rfft(a_mag_dyn)) * (2.0 / N)

        peak_idx_m2 = int(np.argmax(fft_mag[1:])) + 1
        peak_freq_m2 = freqs[peak_idx_m2]
        peak_amp_m2 = fft_mag[peak_idx_m2]

        # Method 2 demonstrates severe frequency doubling to 50 Hz and crushed amplitude
        self.assertAlmostEqual(peak_freq_m2, 50.0, delta=0.05, msg="Rectification failed to exhibit 50 Hz doubling")
        self.assertLess(peak_amp_m2, 0.001, "Rectification dynamic amplitude was not crushed")
        print(f"\n[TEST] Method 1 (Per-Axis): Peak={peak_freq_m1:.1f}Hz @ {peak_amp_m1:.4f}g | Method 2 (Rectified): Peak={peak_freq_m2:.1f}Hz @ {peak_amp_m2:.5f}g")

    def test_evaluate_vibration_record_dynamic_severity_index(self):
        """
        R4.a: Verifies that evaluate_vibration_record correctly processes a CSV dataset,
        computes empirical dynamic severity, preserves signed dynamic signals,
        and brands output as 'Project Dynamic Vibration Severity Index' without ISO-10816-3 claims.
        """
        fs = 400.0
        n_samples = 800
        t = np.arange(n_samples) / fs
        ax = 0.08 * np.sin(2.0 * np.pi * 25.0 * t)
        ay = 0.04 * np.cos(2.0 * np.pi * 25.0 * t)
        az = -0.98 + 0.02 * np.sin(2.0 * np.pi * 50.0 * t)

        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "test_vibration.csv")
            df = pd.DataFrame({
                "Timestamp_us": (t * 1e6).astype(np.int64),
                "Raw_X": np.zeros(n_samples),
                "Raw_Y": np.zeros(n_samples),
                "Raw_Z": np.zeros(n_samples),
                "Accel_X_g": ax,
                "Accel_Y_g": ay,
                "Accel_Z_g": az,
            })
            df.to_csv(csv_path, index=False)

            res = evaluate_vibration_record(csv_path)

            # Check that dynamic components are signed and preserved
            self.assertIn("ax_dyn", res)
            self.assertIn("ay_dyn", res)
            self.assertIn("az_dyn", res)
            self.assertAlmostEqual(float(np.mean(res["ax_dyn"])), 0.0, delta=1e-3)
            self.assertAlmostEqual(float(np.mean(res["az_dyn"])), 0.0, delta=1e-3)

            # Check DC gravity extraction
            self.assertAlmostEqual(res["dc_gravity_g"], 0.98, delta=0.02)

            # Check severity zone branding (Zone A: Dynamic Accel Index < 0.15g)
            self.assertIn("Zone A", res["severity_zone"])
            self.assertIn("Dynamic Accel Index", res["severity_zone"])
            self.assertNotIn("ISO-10816-3 Compliant Standard Velocity", res["severity_zone"])
            print(f"\n[TEST] Vibration Evaluation: RMS={res['rms_dyn_g']:.4f}g | DC Gravity={res['dc_gravity_g']:.3f}g | Zone={res['severity_zone']}")

    # =========================================================================
    # 3. CHECKSUM CALCULATION & CORRUPTED PACKET REJECTION (R4.b)
    # =========================================================================

    def test_checksum_16bit_xor_initialization_and_calculation(self):
        """
        R4.b: Verifies 16-bit XOR checksum calculation initialized to 0x55AA
        with rotating bit-shifts across contiguous byte buffers matching ESP32 firmware.
        """
        # Test 1: Empty buffer returns initial seed 0x55AA
        self.assertEqual(compute_packet_checksum(b""), 0x55AA)

        # Test 2: Single byte 0x00 returns 0x55AA
        self.assertEqual(compute_packet_checksum(bytes([0x00])), 0x55AA)

        # Test 3: Single byte 0x01 at index 0: 0x55AA ^ (0x01 << 0) = 0x55AB
        self.assertEqual(compute_packet_checksum(bytes([0x01])), 0x55AB)

        # Test 4: Single byte 0x01 at index 1: 0x55AA ^ (0x01 << 1) = 0x55AA ^ 0x0002 = 0x55A8
        self.assertEqual(compute_packet_checksum(bytes([0x00, 0x01])), 0x55A8)

        # Test 5: Verify bitwise determinism across a 16-byte block
        data = bytes(range(16))
        chk_manual = 0x55AA
        for i, b in enumerate(data):
            chk_manual ^= ((b << (i % 8)) & 0xFFFF)
        self.assertEqual(compute_packet_checksum(data), chk_manual & 0xFFFF)
        print(f"\n[TEST] 16-bit XOR Checksum: Seed=0x{compute_packet_checksum(b''):04X} | Sample=0x{compute_packet_checksum(data):04X}")

    def test_type_0x05_packet_checksum_validation(self):
        """
        R4.b: Verifies that a valid Type 0x05 packet with 5 accelerometer samples
        and 128 frames of 4-channel audio builds with exact length and valid checksum,
        and is accepted by ESP32MultimodalSource with zero checksum errors.
        """
        accel_samples = [
            (50, -100, 980),
            (55, -95, 982),
            (60, -90, 984),
            (65, -85, 986),
            (70, -80, 988),
        ]
        audio_data = np.zeros((128, 4), dtype=np.int16)
        audio_data[:, 0] = 300
        audio_data[:, 1] = -300
        audio_data[:, 2] = 600
        audio_data[:, 3] = -600

        pkt = ESP32MultimodalSource.build_packet_v2(
            seq=100,
            fs=11025,
            block_frames=128,
            accel_samples=accel_samples,
            audio_data=audio_data,
            tach_pulses=42
        )

        # Expected size: 16 (hdr) + 5*6 (vib) + 128*4*2 (audio) + 2 (chk) = 1072 bytes
        self.assertEqual(len(pkt), 1072)

        # Verify internal checksum matches computation over preceding 1070 bytes
        expected_chk = struct.unpack("<H", pkt[-2:])[0]
        calculated_chk = compute_packet_checksum(pkt[:-2])
        self.assertEqual(expected_chk, calculated_chk)

        # Parse into receiver and verify acceptance
        source = ESP32MultimodalSource(port="COM8")
        parsed = source._parse_incoming_bytes(pkt)
        self.assertEqual(parsed, 1)
        self.assertEqual(source.packets_valid, 1)
        self.assertEqual(source.checksum_errors, 0)
        self.assertEqual(source.last_seq, 100)
        print(f"\n[TEST] Type 0x05 Packet Verified: Len={len(pkt)}B | Checksum=0x{expected_chk:04X} | Valid Pkts={source.packets_valid}")

    def test_corrupted_packet_rejection_single_bit_flip(self):
        """
        R4.b: Verifies 100% rejection of corrupted packets across all packet segments:
        single-bit flips in the header, vibration payload, audio PCM payload, or checksum
        must trigger checksum failure and packet discard without polluting ring buffers.
        """
        accel_samples = [(10, -20, 980)] * 5
        audio_data = np.zeros((128, 4), dtype=np.int16)

        valid_pkt = ESP32MultimodalSource.build_packet_v2(
            seq=1, fs=11025, block_frames=128,
            accel_samples=accel_samples,
            audio_data=audio_data,
            tach_pulses=10
        )
        self.assertEqual(len(valid_pkt), 1072)

        # Test bit flips across critical positions:
        # Header: byte 0 (sync), byte 2 (type), byte 4 (seq), byte 10 (num_vib)
        # Vibration payload: byte 16, byte 30
        # Audio payload: byte 50, byte 500, byte 1000
        # Checksum: byte 1070, byte 1071
        corruption_indices = [0, 2, 4, 10, 16, 30, 50, 500, 1000, 1070, 1071]

        for byte_idx in corruption_indices:
            source = ESP32MultimodalSource(port="COM8")
            corrupted = bytearray(valid_pkt)
            corrupted[byte_idx] ^= 0x01  # Flip least significant bit

            parsed = source._parse_incoming_bytes(corrupted)

            # Corrupted packet must NOT be accepted as valid
            self.assertEqual(parsed, 0, f"Corrupted packet at byte {byte_idx} was erroneously parsed!")
            self.assertEqual(source.packets_valid, 0, f"Corrupted packet at byte {byte_idx} incremented packets_valid!")

            # If sync bytes (0, 1) intact, checksum failure is logged
            if byte_idx >= 2:
                self.assertGreaterEqual(
                    source.checksum_errors, 1,
                    f"Corrupted packet at byte {byte_idx} did not increment checksum_errors!"
                )

        print(f"\n[TEST] Corrupted Packet Rejection: Verified 100% rejection across {len(corruption_indices)} bit-flip locations.")

    # =========================================================================
    # 4. FRACTIONAL-DELAY SIMULATION & PHASE COMPENSATION (R4.c)
    # =========================================================================

    def test_fractional_delay_phase_compensation_across_sample_rates(self):
        """
        R4.c: Verifies frequency-domain phase compensation H(f) = exp(-j * 2 * pi * f * tau_comp)
        across both 44.1 kHz and 11.025 kHz (and 16 kHz) sample rates.
        Verifies positive tau_comp compensates inter-pair DMA delay with sub-sample precision
        (< 0.1 μs residual error) and strict all-pass amplitude preservation.
        """
        sample_rates = [44100.0, 16000.0, 11025.0]
        # Injected DMA startup latency (24.69 μs hardware offset) and various fractional delays
        test_skews_us = [24.69, -15.20, 33.10]

        for fs in sample_rates:
            n_samples = int(fs * 0.25)
            dt = 1.0 / fs
            f_high = min(4000.0, fs * 0.40)
            b, a = signal.butter(4, [300.0 / (fs/2), f_high / (fs/2)], btype='bandpass')
            np.random.seed(789)
            base_sig = signal.filtfilt(b, a, np.random.randn(n_samples)).astype(np.float32)

            audio_orig = np.tile(base_sig[:, None], (1, 4))
            freqs = np.fft.rfftfreq(n_samples, d=dt)
            base_fft = np.fft.rfft(base_sig)

            for skew_us in test_skews_us:
                skew_sec = skew_us * 1e-6
                audio_skewed = audio_orig.copy()

                # Skew Pair B (channels 2 & 3) by +skew_sec (lagging Pair A)
                skew_filter = np.exp(-2j * np.pi * freqs * skew_sec)
                ch_skewed = np.fft.irfft(base_fft * skew_filter, n=n_samples)
                audio_skewed[:, 2] = ch_skewed
                audio_skewed[:, 3] = ch_skewed

                # Apply phase compensation: H(f) = exp(-2j * pi * f * (-skew_sec))
                # Advances Pair B by -(-skew_sec) = +skew_sec, canceling the lag
                audio_comp = apply_pair_phase_compensation(audio_skewed, fs, -skew_sec)

                # GCC-PHAT timing recovery check between Mic 0 (Pair A) and Mic 2 (Compensated Pair B)
                est_tau, _, _ = gcc_phat(audio_orig[:, 0], audio_comp[:, 2], fs, max_tau=0.005, interp=32)
                res_err_us = abs(est_tau * 1e6)

                # Must achieve sub-sample accuracy: residual error < 0.1 μs
                self.assertLess(
                    res_err_us, 0.1,
                    f"Phase compensation at fs={fs}Hz with skew={skew_us}μs left {res_err_us:.4f}μs residual error!"
                )

                # Must preserve amplitude: all-pass property |H(f)| = 1
                interior = slice(128, n_samples - 128)
                amp_ratio = np.std(audio_comp[interior, 2]) / np.std(audio_orig[interior, 2])
                self.assertAlmostEqual(
                    amp_ratio, 1.0, places=2,
                    msg=f"All-pass magnitude violation at fs={fs}Hz: ratio={amp_ratio:.5f}"
                )

        print(f"\n[TEST] Fractional-Delay Phase Compensation: Verified across {len(sample_rates)} sample rates (res error < 0.1 μs).")

    # =========================================================================
    # 5. TACHOMETER INTERVAL COUNTING & ABSENT-PULSE HANDLING (R4.d)
    # =========================================================================

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

    def test_tachometer_n_minus_1_revolution_intervals(self):
        """
        R4.d: Verifies that N pulse timestamps yield exactly N-1 revolution intervals
        spanning (N-1)*2*pi radians in order_tracking.py.
        Eliminates the 20% order frequency scaling bias, locking 1X order at exactly 1.000.
        Boundary check: N < 2 raises ValueError.
        """
        fs = 4000.0
        f_shaft = 25.0  # 1500 RPM
        t_rev = 1.0 / f_shaft

        # Test across various revolution counts N
        for N in [2, 3, 5, 10, 50]:
            rev_times = np.arange(N) * t_rev
            t_vib = np.arange(0, rev_times[-1] + 0.01, 1.0 / fs)
            # Pure 1X unbalance signal
            vib_sig = 0.5 * np.sin(2.0 * np.pi * f_shaft * t_vib)

            ot_res = compute_order_tracking(t_vib, vib_sig, rev_times=rev_times, samples_per_rev=128)

            # Assert exactly N - 1 revolution intervals
            self.assertEqual(ot_res["total_revolutions"], N - 1, f"Failed N-1 interval count for N={N}")
            self.assertEqual(ot_res["metrics"]["total_revolutions"], N - 1)

            # Assert 1X order peak is locked at exactly 1.000
            peak_order = ot_res["orders"][np.argmax(ot_res["order_mag"])]
            self.assertAlmostEqual(peak_order, 1.0, delta=0.01, msg=f"1X order lock failed for N={N}: {peak_order}")

        # Boundary checks: N=0 and N=1 must raise ValueError
        with self.assertRaises(ValueError):
            compute_order_tracking(np.arange(100)/100.0, np.zeros(100), rev_times=np.array([]))

        with self.assertRaises(ValueError):
            compute_order_tracking(np.arange(100)/100.0, np.zeros(100), rev_times=np.array([0.04]))

        print(f"\n[TEST] Tachometer Interval Counting: Verified N-1 intervals across N in [2, 3, 5, 10, 50].")

    def test_tachometer_absent_pulse_handling_unavailable(self):
        """
        R4.d: Verifies that absent or insufficient tachometer pulses (< 2 edges)
        report status == 'UNAVAILABLE', mean_rpm == 0.0, and do NOT fabricate
        a synthetic 1500 RPM / 25 Hz timeline.
        """
        # Test 1: extract_tachometer_revolutions on noise floor (< 0.5V dynamic range)
        t_noise = np.arange(1000) / 1000.0
        tach_noise = np.random.uniform(0.0, 0.35, len(t_noise))  # Noise floor < 0.5V dynamic range
        rev_noise = extract_tachometer_revolutions(t_noise, tach_noise)
        self.assertEqual(len(rev_noise), 0, "Noise floor was erroneously detected as tachometer edges")

        # Test 2: extract_tachometer_revolutions with single pulse (< 2 edges) raises ValueError
        tach_single = np.zeros(1000)
        tach_single[500:510] = 3.3
        with self.assertRaises(ValueError):
            extract_tachometer_revolutions(t_noise, tach_single)

        # Test 3: run_order_tracking_on_csv with absent tachometer
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "no_tach.csv")
            t = np.arange(400) / 400.0
            df = pd.DataFrame({
                "Timestamp_us": (t * 1e6).astype(np.int64),
                "Accel_Z_g": 0.05 * np.sin(2.0 * np.pi * 25.0 * t),
                # No tachometer column present
            })
            df.to_csv(csv_path, index=False)

            metrics = run_order_tracking_on_csv(csv_path, tmpdir)

            self.assertEqual(metrics["status"], "UNAVAILABLE")
            self.assertEqual(metrics["mean_rpm"], 0.0)
            self.assertEqual(metrics["total_revolutions"], 0)
            self.assertEqual(metrics["e_1x_pct"], 0.0)
            self.assertIn("UNAVAILABLE", metrics["primary_order_fault"])
            self.assertNotIn("1500", str(metrics["mean_rpm"]))

        print(f"\n[TEST] Absent Tachometer Handling: Verified UNAVAILABLE status without synthetic RPM fallback.")


# =============================================================================
# 6. PHYSICAL HARDWARE INTEGRATION TESTS (SEGREGATED) (R4.e)
# =============================================================================

class TestHardwareIntegration(unittest.TestCase):
    """
    R4.e: Physical hardware integration tests.
    Segregated from synthetic unit tests so the primary test suite runs 100% offline
    in CI environments without physical COM ports or serial hardware.
    To run this class, set environment variable ENABLE_HARDWARE_TESTS=1.
    """

    @unittest.skipUnless(
        os.environ.get("ENABLE_HARDWARE_TESTS") == "1",
        "Physical hardware test segregated from synthetic offline suite. Set ENABLE_HARDWARE_TESTS=1 to execute."
    )
    def test_live_esp32_serial_port_detection(self):
        """Probes COM8 or available serial ports when physical hardware is connected."""
        import serial
        import serial.tools.list_ports

        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        print(f"\n[HARDWARE TEST] Available Serial Ports: {available_ports}")

        target_port = "COM8"
        if target_port in available_ports:
            ser = serial.Serial(target_port, 1152000, timeout=1.0)
            self.assertTrue(ser.is_open)
            ser.close()
            print(f"[HARDWARE TEST] Successfully probed {target_port} at 1,152,000 baud.")
        else:
            self.skipTest(f"{target_port} not currently connected to host.")


if __name__ == "__main__":
    unittest.main()
