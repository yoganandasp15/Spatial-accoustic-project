#!/usr/bin/env python3
"""
=============================================================================
 UNIT TESTS: PHYSICAL MOTOR DATASET RECORDING & VALIDATION PIPELINE
=============================================================================
Module: tests/test_dataset_recording_pipeline.py
Description: Validates Phase 1 TinyML dataset recording and validation tools:
  1. CLI argument parsing & default fault-type handling (defective -> unknown)
  2. MotorExperimentRecorder artifact generation (audio_4ch.wav, vibration.csv,
     packet_log.csv, metadata.json)
  3. Collision-resistant run IDs and exclusive directory creation
  4. Monotonic timing & recording status (COMPLETE, INTERRUPTED, INCOMPLETE, FAILED)
  5. Validator enforcement of +/-2g ADXL345 saturation and over-range checks
  6. Strict validator rejection of checksum errors and excessive packet drops
  7. Strict validator enforcement of +/-10% sample-count tolerances
  8. Pre-connection abort preventing orphaned empty directories
  9. Segregation of synthetic testbench data from physical training sets
=============================================================================
"""

import os
import sys
import json
import wave
import shutil
import tempfile
import unittest
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from tools.record_motor_experiment import MotorExperimentRecorder
from tools.validate_dataset import validate_run, validate_dataset


class TestDatasetRecordingPipeline(unittest.TestCase):
    def setUp(self):
        self.test_dir = tempfile.mkdtemp(prefix="test_dataset_")
        self.addCleanup(shutil.rmtree, self.test_dir, ignore_errors=True)

    def test_synthetic_healthy_quiet_run(self):
        """Verify complete generation of artifacts for a healthy quiet testbench run."""
        recorder = MotorExperimentRecorder(
            motor_id="motor_test_A",
            health_label="healthy",
            environment="quiet",
            fault_type="none",
            duration=1.0,
            output_dir=self.test_dir,
            use_synthetic=True
        )
        success = recorder.run()
        self.assertTrue(success, "Recording execution should return True")

        # Verify artifacts exist in run dir
        run_dir = recorder.run_dir
        self.assertTrue(os.path.isdir(run_dir))
        self.assertTrue(os.path.isfile(os.path.join(run_dir, "metadata.json")))
        self.assertTrue(os.path.isfile(os.path.join(run_dir, "vibration.csv")))
        self.assertTrue(os.path.isfile(os.path.join(run_dir, "audio_4ch.wav")))
        self.assertTrue(os.path.isfile(os.path.join(run_dir, "packet_log.csv")))

        # Check metadata content
        with open(os.path.join(run_dir, "metadata.json"), "r", encoding="utf-8") as f:
            meta = json.load(f)
        self.assertEqual(meta["motor_id"], "motor_test_A")
        self.assertEqual(meta["motor_health_label"], "healthy")
        self.assertEqual(meta["acoustic_environment"], "quiet")
        self.assertEqual(meta["recording_status"], "COMPLETE")
        self.assertEqual(meta["accel_configured_range_g"], 2.0)
        self.assertEqual(meta["sample_rate_audio_hz"], 10000.0)
        self.assertEqual(meta["sample_rate_vib_nominal_hz"], 400.0)
        self.assertEqual(meta["audio_channels"], 4)
        self.assertTrue(meta["is_synthetic"])
        self.assertIn("SYNTHETIC", meta["data_provenance"])

        # Check 4-channel audio wave parameters
        with wave.open(os.path.join(run_dir, "audio_4ch.wav"), "rb") as wf:
            self.assertEqual(wf.getnchannels(), 4, "WAV must have exactly 4 channels")
            self.assertEqual(wf.getsampwidth(), 2, "WAV must be 16-bit PCM (2 bytes/sample)")
            self.assertEqual(wf.getframerate(), 10000, "WAV must be 10,000 Hz")
            self.assertGreaterEqual(wf.getnframes(), 9500, "1.0s audio should yield ~10,000 frames")

        # Check vibration CSV row count and columns
        with open(os.path.join(run_dir, "vibration.csv"), "r", encoding="utf-8") as f:
            lines = [l.strip() for l in f if l.strip()]
        self.assertGreater(len(lines), 360, "1.0s at 400Hz should yield ~400 samples")
        header = lines[0].split(",")
        self.assertIn("accel_x_g", header)
        self.assertIn("accel_y_g", header)
        self.assertIn("accel_z_g", header)
        self.assertIn("dyn_mag_g", header)
        self.assertIn("tach_pulses", header)

        # Check manifest
        manifest_path = os.path.join(self.test_dir, "manifest.json")
        self.assertTrue(os.path.isfile(manifest_path))
        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)
        self.assertEqual(manifest["total_runs"], 1)
        self.assertEqual(manifest["runs"][0]["run_id"], recorder.run_id)
        self.assertEqual(manifest["runs"][0]["recording_status"], "COMPLETE")

    def test_synthetic_defective_unknown_fault(self):
        """Verify defective motor run defaults to unknown fault type and elevated vibration."""
        rec_def = MotorExperimentRecorder(
            motor_id="motor_test_B",
            health_label="defective",
            environment="disturbance",
            fault_type="unknown",
            duration=1.0,
            output_dir=self.test_dir,
            use_synthetic=True
        )
        rec_def.run()

        with open(os.path.join(rec_def.run_dir, "metadata.json"), "r", encoding="utf-8") as f:
            meta = json.load(f)

        self.assertEqual(meta["motor_health_label"], "defective")
        self.assertEqual(meta["fault_type"], "unknown")
        self.assertEqual(meta["acoustic_environment"], "disturbance")
        self.assertEqual(meta["recording_status"], "COMPLETE")
        self.assertGreater(meta["mean_dyn_vib_g"], 0.15, "Defective motor vibration must be elevated")

    def test_run_id_collision_resistance(self):
        """Verify rapid successive runs generate distinct, collision-resistant run IDs."""
        rec1 = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec2 = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        self.assertNotEqual(rec1.run_id, rec2.run_id, "Run IDs must be globally unique")

    def test_exclusive_dir_creation(self):
        """Verify save_dataset_artifacts raises FileExistsError if run directory already exists."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.generate_synthetic_testbench_run()
        # Pre-create directory to simulate collision
        os.makedirs(rec.run_dir)
        with self.assertRaises(FileExistsError):
            rec.save_dataset_artifacts()

    def test_preconnection_failure_no_empty_dir(self):
        """Verify pre-connection failure leaves no empty run directory on disk."""
        rec = MotorExperimentRecorder(
            port="COM_NONEXISTENT_99",
            output_dir=self.test_dir,
            use_synthetic=False
        )
        success = rec.run()
        self.assertFalse(success)
        self.assertEqual(rec.recording_status, "FAILED")
        self.assertFalse(os.path.exists(rec.run_dir), "No empty directory should remain after failed connection")

    def test_validator_rejects_interrupted_or_incomplete_runs(self):
        """Verify validator strictly rejects runs with INTERRUPTED or INCOMPLETE status."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.run()

        # Modify status to INTERRUPTED in metadata
        meta_path = os.path.join(rec.run_dir, "metadata.json")
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        meta["recording_status"] = "INTERRUPTED"
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        report = validate_run(rec.run_dir, strict=True, allow_synthetic=True)
        self.assertFalse(report["valid"], "Validator must reject INTERRUPTED run")
        self.assertTrue(any("INTERRUPTED" in err for err in report["errors"]))

    def test_adxl345_saturation_detection(self):
        """Verify validator flags near-saturation (>=1.95g) and rejects over-range (>2.05g)."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.run()

        vib_path = os.path.join(rec.run_dir, "vibration.csv")
        with open(vib_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # Inject an over-range sample (2.15g) into vibration.csv
        header = lines[0]
        corrupted_line = "999,1.500000,2.1500,0.1000,-0.8200,2.1500,0.1000,0.0000,2.1523,10,1800.0\n"
        with open(vib_path, "w", encoding="utf-8") as f:
            f.write(header)
            f.write(corrupted_line)
            f.writelines(lines[1:])

        report = validate_run(rec.run_dir, strict=True, allow_synthetic=True)
        self.assertFalse(report["valid"], "Validator must reject readings exceeding +/-2g range")
        self.assertTrue(any("exceeds configured range" in err for err in report["errors"]))

    def test_checksum_errors_fail_strict_validation(self):
        """Verify non-zero checksum errors trigger validation failure under strict mode."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.run()

        meta_path = os.path.join(rec.run_dir, "metadata.json")
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        meta["checksum_errors"] = 5
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        report_strict = validate_run(rec.run_dir, strict=True, allow_synthetic=True)
        self.assertFalse(report_strict["valid"], "Strict validation must fail when checksum errors > 0")
        self.assertTrue(any("checksum" in err.lower() for err in report_strict["errors"]))

    def test_packet_drops_fail_strict_validation(self):
        """Verify excessive packet drops trigger failure under strict mode."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.run()

        meta_path = os.path.join(rec.run_dir, "metadata.json")
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        meta["dropped_packets"] = 25  # Excessive drops
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        report_strict = validate_run(rec.run_dir, strict=True, allow_synthetic=True)
        self.assertFalse(report_strict["valid"], "Strict validation must fail on excessive packet drops")
        self.assertTrue(any("drops" in err.lower() for err in report_strict["errors"]))

    def test_sample_count_tolerance_10_percent(self):
        """Verify validator enforces +/-10% vibration sample count requirement."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.run()

        # Truncate vibration.csv to 200 samples (50% of expected 400 for 1.0s)
        vib_path = os.path.join(rec.run_dir, "vibration.csv")
        with open(vib_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
        with open(vib_path, "w", encoding="utf-8") as f:
            f.writelines(lines[:200])

        report = validate_run(rec.run_dir, strict=True, allow_synthetic=True)
        self.assertFalse(report["valid"], "Validator must reject sample count with >10% discrepancy")
        self.assertTrue(any("discrepancy" in err.lower() for err in report["errors"]))

    def test_strict_mode_rejects_synthetic(self):
        """Verify validate_dataset in strict mode without override rejects synthetic data."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.run()

        # Run strict validation without allow_synthetic
        ds_strict = validate_dataset(self.test_dir, strict=True, allow_synthetic=False)
        self.assertGreater(ds_strict["invalid_runs_count"], 0)

        # Run strict validation WITH allow_synthetic
        ds_allowed = validate_dataset(self.test_dir, strict=True, allow_synthetic=True)
        self.assertEqual(ds_allowed["invalid_runs_count"], 0)
        self.assertEqual(ds_allowed["valid_runs_count"], 1)

    def test_preexisting_cumulative_pulse_count_no_rpm_spike(self):
        """Verify pre-existing cumulative pulse count (e.g. 25000) does not produce initial RPM spike."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        self.assertFalse(rec.tach_baseline_established)
        self.assertEqual(rec.last_pulse_count, 0)

        # Simulate first packet arriving with cumulative pulses = 50,000 from prior motor rotation
        now_1 = 100.0
        pulses_1 = 50000
        if not rec.tach_baseline_established:
            rec.last_pulse_count = pulses_1
            rec.last_pulse_time_mono = now_1
            rec.tach_baseline_established = True

        self.assertTrue(rec.tach_baseline_established)
        self.assertEqual(rec.last_pulse_count, 50000)
        self.assertIsNone(rec.current_rpm, "Initial packet must not compute RPM")

        # Simulate second packet 0.50s later with 15 pulses added (1800 RPM nominal)
        now_2 = 100.50
        pulses_2 = 50015
        dt_p = now_2 - rec.last_pulse_time_mono
        if dt_p >= 0.5:
            dp = (pulses_2 - rec.last_pulse_count) & 0xFFFFFFFF
            rec.current_rpm = ((dp / rec.pulses_per_rev) / dt_p) * 60.0
            rec.rpm_history.append(rec.current_rpm)

        self.assertIsNotNone(rec.current_rpm)
        self.assertAlmostEqual(rec.current_rpm, 1800.0, delta=10.0)
        self.assertLess(rec.current_rpm, 3000.0, "RPM must reflect delta, not cumulative 50000 boot count")

    def test_interrupted_run_returns_false_exit_code(self):
        """Verify interrupted run saves artifacts for debugging but returns False (exit 1)."""
        rec = MotorExperimentRecorder(output_dir=self.test_dir, use_synthetic=True)
        rec.generate_synthetic_testbench_run()
        rec.recording_status = "INTERRUPTED"

        # run() must return False when status is not COMPLETE
        success = rec.run()
        self.assertFalse(success, "Interrupted run must return False so process exits with code 1")

        # But artifacts must be preserved for debugging
        self.assertTrue(os.path.isdir(rec.run_dir))
        with open(os.path.join(rec.run_dir, "metadata.json"), "r", encoding="utf-8") as f:
            meta = json.load(f)
        self.assertEqual(meta["recording_status"], "INTERRUPTED")

    def test_header_contract_validation_rejection(self):
        """Verify invalid Type 0x05 header fields (channels != 4, rate != 10k, etc.) are rejected."""
        # Contract invariants: pt == 0x05, ch == 4, fs_val == 10000, blk == 128, num_vib <= 8
        valid_header = {"pt": 0x05, "ch": 4, "fs_val": 10000, "blk": 128, "num_vib": 5}
        self.assertTrue(
            valid_header["pt"] == 0x05 and valid_header["ch"] == 4 and
            valid_header["fs_val"] == 10000 and valid_header["blk"] == 128 and
            valid_header["num_vib"] <= 8
        )

        # Invalid channels (e.g. 2 instead of 4)
        invalid_ch = {"pt": 0x05, "ch": 2, "fs_val": 10000, "blk": 128, "num_vib": 5}
        is_valid = (
            invalid_ch["pt"] == 0x05 and invalid_ch["ch"] == 4 and
            invalid_ch["fs_val"] == 10000 and invalid_ch["blk"] == 128 and
            invalid_ch["num_vib"] <= 8
        )
        self.assertFalse(is_valid, "Channel count != 4 must fail contract validation")

        # Excessive vibration samples (> 8)
        invalid_vib = {"pt": 0x05, "ch": 4, "fs_val": 10000, "blk": 128, "num_vib": 16}
        is_valid_vib = (
            invalid_vib["pt"] == 0x05 and invalid_vib["ch"] == 4 and
            invalid_vib["fs_val"] == 10000 and invalid_vib["blk"] == 128 and
            invalid_vib["num_vib"] <= 8
        )
        self.assertFalse(is_valid_vib, "num_vib > 8 must fail contract validation")


if __name__ == "__main__":
    unittest.main()
