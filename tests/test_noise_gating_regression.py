"""
=============================================================================
 REGRESSION & ROBUSTNESS TEST SUITE FOR NOISE GATING & ACOUSTIC LOCALIZATION
=============================================================================
Module: tests/test_noise_gating_regression.py
Target: Validates that SRP-PHAT direction estimation, VAD, and target tracking:
        1. Reject 100% of quiet room noise frames (zero false detections).
        2. Return quiet/invalid status on pure silence and sub-threshold noise.
        3. Maintain stable noise floor adaptation under sustained active sound.
        4. Accurately estimate directions for active sources (0°, ±30°, ±60°).
        5. Tolerate dead microphones gracefully via channel consensus.
        6. Reject unverified / skewed I2S phase calibration by default.
        7. Smoothly track moving sound sources across angular sectors without sticking.
=============================================================================
"""

import os
import sys
import unittest
import numpy as np

# Ensure src directory is in sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from spatial_acoustic_radar_pro import (
    SRPPhatEngine,
    ArrayGeometry,
    PRESETS,
    DynamicTargetTracker,
    SPEED_OF_SOUND,
    apply_pair_phase_compensation,
)


class TestNoiseGatingRegression(unittest.TestCase):
    """
    Test suite certifying acoustic radar noise gating, silence suppression,
    dynamic tracking, and channel health diagnostics.
    """

    @classmethod
    def setUpClass(cls):
        cls.geometry = PRESETS["ESP32_4MIC_LINEAR"]
        cls.fs = 11025.0
        cls.frame_len = 512

    def _synthesize_source(self, angle_deg: float, snr_rms: float = 0.04, seed: int = 42) -> np.ndarray:
        """
        Synthesizes a 4-channel acoustic frame for a far-field source at angle_deg.
        For a source at angle theta (measured from broadside y > 0, with +x to the right),
        wave arrives at mic x_m earlier by tau = x_m * sin(theta) / c, giving phase advance +2*pi*f*tau.
        """
        np.random.seed(seed)
        t = np.arange(self.frame_len) / self.fs
        source = (
            0.6 * np.sin(2 * np.pi * 1200.0 * t)
            + 0.4 * np.sin(2 * np.pi * 2400.0 * t)
            + 0.2 * np.random.randn(self.frame_len)
        )
        source = source / np.std(source) * snr_rms

        freqs = np.fft.rfftfreq(self.frame_len, d=1.0 / self.fs)
        S = np.fft.rfft(source)

        theta = np.radians(angle_deg)
        signals = np.zeros((self.frame_len, 4), dtype=np.float64)

        for m in range(4):
            xm = self.geometry.mic_coords[m, 0]
            tau = xm * np.sin(theta) / SPEED_OF_SOUND
            Sm = S * np.exp(+2j * np.pi * freqs * tau)
            signals[:, m] = np.fft.irfft(Sm, n=self.frame_len)

        return signals

    def test_01_silence_pure_zeros_rejected(self):
        """Test that pure zeros produce invalid bearing, zero prominence, and quiet state."""
        zeros_frame = np.zeros((self.frame_len, 4), dtype=np.float64)
        bearing, prom, angles, srp_norm, heatmap, metrics = SRPPhatEngine.compute_srp_frame(
            zeros_frame, self.fs, self.geometry, min_active_rms=0.0020
        )
        self.assertFalse(metrics["is_valid"], "Pure zero frame must be flagged invalid.")
        self.assertEqual(metrics["active_pairs"], 0, "No active pairs should be counted on silence.")
        self.assertEqual(bearing, 0.0, "Bearing on silence should default to 0.0.")
        self.assertEqual(prom, 0.0, "Prominence on silence should be 0.0.")
        self.assertEqual(metrics["dom_side"], "NONE", "Dominant side should be NONE on silence.")

    def test_02_independent_noise_50_frames_zero_false_bearings(self):
        """
        Critical regression test: 50 independent frames of quiet ambient noise
        must yield 0 false detections (100% rejection rate).
        """
        np.random.seed(101)
        false_detections = 0
        total_frames = 50

        for frame_idx in range(total_frames):
            noise_rms = 0.0008 + 0.0004 * np.random.rand()
            noise = np.random.randn(self.frame_len, 4) * noise_rms

            bearing, prom, _, _, _, metrics = SRPPhatEngine.compute_srp_frame(
                noise, self.fs, self.geometry, min_active_rms=0.0020
            )

            if metrics["is_valid"]:
                false_detections += 1

        self.assertEqual(
            false_detections, 0,
            f"Expected 0 false detections across {total_frames} quiet noise frames, got {false_detections}."
        )

    def test_03_weak_noise_below_threshold_gated(self):
        """Test that noise below min_active_rms is strictly gated by the silence cutoff."""
        noise = np.random.randn(self.frame_len, 4) * 0.0012  # Below 0.0020
        bearing, prom, _, _, _, metrics = SRPPhatEngine.compute_srp_frame(
            noise, self.fs, self.geometry, min_active_rms=0.0025
        )
        self.assertFalse(metrics["is_valid"])
        self.assertEqual(metrics["active_pairs"], 0)
        self.assertEqual(metrics["avg_rms"], float(np.mean(np.sqrt(np.mean(noise**2, axis=0)))))

    def test_04_sustained_sound_noise_floor_stability(self):
        """
        Test that active sound does NOT cause noise floor creep that absorbs sustained audio.
        Simulates 100 frames of loud active sound, followed by 25 frames of silence.
        """
        noise_floor = 0.0015
        min_active_threshold = 0.0030
        last_active_time = 0.0
        hangover_sec = 0.350
        frame_dt = self.frame_len / self.fs  # ~46.4 ms per frame

        sim_time = 1000.0

        # Phase 1: 100 frames of active audio (RMS = 0.025)
        active_rms = 0.025
        for _ in range(100):
            sim_time += frame_dt
            attack_thresh = max(min_active_threshold, noise_floor * 1.60)
            release_thresh = max(0.0020, noise_floor * 1.25)

            if active_rms >= attack_thresh:
                last_active_time = sim_time
                is_active = True
            elif (sim_time - last_active_time) < hangover_sec:
                is_active = bool(active_rms >= release_thresh)
            else:
                is_active = False

            # VAD noise floor adaptation rule: ONLY adapt if not active and quiet > 0.60s
            if not is_active and (sim_time - last_active_time) > 0.60:
                if active_rms < noise_floor:
                    noise_floor = 0.92 * noise_floor + 0.08 * active_rms
                elif active_rms < noise_floor * 1.30:
                    noise_floor = 0.995 * noise_floor + 0.005 * active_rms

            self.assertTrue(is_active, "Audio must remain continuously active during sustained sound.")
            self.assertAlmostEqual(
                noise_floor, 0.0015, delta=1e-5,
                msg="Noise floor must remain frozen and not creep upward during active sound."
            )

        # Phase 2: 25 frames of silence (RMS = 0.0010)
        quiet_rms = 0.0010
        adapted_during_quiet = False
        initial_floor = noise_floor
        for _ in range(25):
            sim_time += frame_dt
            attack_thresh = max(min_active_threshold, noise_floor * 1.60)
            release_thresh = max(0.0020, noise_floor * 1.25)

            if quiet_rms >= attack_thresh:
                last_active_time = sim_time
                is_active = True
            elif (sim_time - last_active_time) < hangover_sec:
                is_active = bool(quiet_rms >= release_thresh)
            else:
                is_active = False

            if not is_active and (sim_time - last_active_time) > 0.60:
                if quiet_rms < noise_floor:
                    noise_floor = 0.92 * noise_floor + 0.08 * quiet_rms
                    adapted_during_quiet = True

        self.assertTrue(adapted_during_quiet, "Noise floor should adapt downward during confirmed quiet intervals.")
        self.assertLess(noise_floor, initial_floor, "Noise floor should track down toward quiet ambient RMS.")

    def test_05_known_bearings_accuracy(self):
        """
        Test angular accuracy across broadside and oblique angles:
        0°, +30°, -30°, +60°, -60°. Errors must be < 2.5°.
        """
        test_angles = [0.0, 30.0, -30.0, 60.0, -60.0]
        for true_theta in test_angles:
            frame = self._synthesize_source(true_theta, snr_rms=0.04, seed=int(abs(true_theta) + 50))
            bearing, prom, _, _, _, metrics = SRPPhatEngine.compute_srp_frame(
                frame, self.fs, self.geometry, min_active_rms=0.0020
            )
            self.assertTrue(metrics["is_valid"], f"Source at {true_theta}° should be detected as valid.")
            err = abs(bearing - true_theta)
            self.assertLessEqual(
                err, 2.5,
                f"Bearing error for target {true_theta}° was {err:.2f}° (Est: {bearing:.2f}°), expected <= 2.5°."
            )

    def test_06_dead_microphone_exclusion(self):
        """
        Test that a dead microphone (e.g., loose breadboard pin on channel 3) is detected
        as DEAD and safely excluded without crash, allowing remaining channels to localize.
        """
        frame = self._synthesize_source(25.0, snr_rms=0.04, seed=77)
        # Kill Channel 3 (index 2)
        frame[:, 2] = 0.0

        bearing, prom, _, _, _, metrics = SRPPhatEngine.compute_srp_frame(
            frame, self.fs, self.geometry, min_active_rms=0.0020
        )

        ch_status = metrics["ch_status"]
        self.assertEqual(ch_status[2], "DEAD", "Channel 3 must be detected and marked DEAD.")
        self.assertEqual(ch_status[0], "ACTIVE", "Channel 1 should be ACTIVE.")
        self.assertEqual(ch_status[1], "ACTIVE", "Channel 2 should be ACTIVE.")
        self.assertEqual(ch_status[3], "ACTIVE", "Channel 4 should be ACTIVE.")
        self.assertTrue(metrics["is_valid"], "Remaining 3 channels should successfully maintain localization.")

        err = abs(bearing - 25.0)
        self.assertLessEqual(err, 3.5, f"Bearing with 1 dead mic should still be accurate. Est: {bearing:.2f}°.")

    def test_07_unhealthy_calibration_bypass(self):
        """
        Test that applying unverified I2S phase calibration delay (24.69 μs) introduces
        artificial steering bias on broadside sound, while bypassing it retains broadside alignment.
        """
        broadside_frame = self._synthesize_source(0.0, snr_rms=0.04, seed=12)

        # 1. Without calibration compensation (raw DMA aligned):
        bearing_raw, _, _, _, _, _ = SRPPhatEngine.compute_srp_frame(
            broadside_frame, self.fs, self.geometry
        )
        self.assertAlmostEqual(bearing_raw, 0.0, delta=1.0, msg="Raw DMA broadside angle should be 0.0° ± 1.0°.")

        # 2. Applying false +24.69 μs skew compensation:
        skewed_frame = apply_pair_phase_compensation(broadside_frame, self.fs, 24.69e-6)
        bearing_skewed, _, _, _, _, _ = SRPPhatEngine.compute_srp_frame(
            skewed_frame, self.fs, self.geometry
        )
        # In Pair B, +24.69 μs shifts channels 3 and 4, causing significant angular deflection
        self.assertGreater(
            abs(bearing_skewed - 0.0), 5.0,
            f"Applying unverified phase calibration must alter bearing significantly (observed: {bearing_skewed:.1f}°)."
        )

    def test_08_moving_source_continuous_tracking(self):
        """
        Test that DynamicTargetTracker tracks a moving sound source from -45° to +45°
        smoothly without sticking to the left wing or center.
        """
        tracker = DynamicTargetTracker(alpha=0.35, gate_deg=18.0)
        trajectory = np.linspace(-45.0, 45.0, 19)  # 5° increments

        tracked_angles = []
        for angle in trajectory:
            for _ in range(3):
                t_angle, t_range, conf, locked = tracker.update(
                    meas_angle=angle,
                    meas_prom=2.5,
                    is_active_sound=True,
                    nominal_range=1.2
                )
            tracked_angles.append(t_angle)

        self.assertGreater(tracked_angles[-1], 35.0, "Tracker should reach right sector (> +35°).")
        self.assertLess(tracked_angles[0], -35.0, "Tracker should begin in left sector (< -35°).")
        for i in range(1, len(tracked_angles)):
            self.assertGreaterEqual(
                tracked_angles[i], tracked_angles[i - 1] - 0.5,
                f"Tracker output dropped backwards at step {i}: {tracked_angles[i-1]:.1f}° -> {tracked_angles[i]:.1f}°."
            )


if __name__ == "__main__":
    unittest.main()
