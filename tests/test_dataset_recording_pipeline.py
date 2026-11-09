#!/usr/bin/env python3
"""
=============================================================================
 UNIT TESTS: PHYSICAL MOTOR DATASET RECORDING & VALIDATION PIPELINE
=============================================================================
Module: tests/test_dataset_recording_pipeline.py
Description: Validates Phase 1 TinyML dataset recording tools:
  1. CLI argument parsing & legacy --mode compatibility
  2. MotorExperimentRecorder artifact generation (audio_4ch.wav, vibration.csv,
     packet_log.csv, metadata.json)
  3. Manifest registry creation & incremental updates
  4. validate_dataset.py run inspection (audio channels, sample rate, finite math)
  5. Error detection on missing artifacts or corrupted channels
  6. Synthetic vs Physical data provenance segregation
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
        self.assertGreater(len(lines), 350, "1.0s at 400Hz should yield ~400 samples")
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

    def test_synthetic_defective_elevated_vibration(self):
        """Verify defective motor run produces elevated vibration dynamics."""
        rec_def = MotorExperimentRecorder(
            motor_id="motor_test_B",
            health_label="defective",
            environment="disturbance",
            fault_type="unbalance",
            duration=1.0,
            output_dir=self.test_dir,
            use_synthetic=True
        )
        rec_def.run()

        with open(os.path.join(rec_def.run_dir, "metadata.json"), "r", encoding="utf-8") as f:
            meta = json.load(f)

        self.assertEqual(meta["motor_health_label"], "defective")
        self.assertEqual(meta["acoustic_environment"], "disturbance")
        # Defective motor model produces mean dynamic acceleration > 0.15g
        self.assertGreater(meta["mean_dyn_vib_g"], 0.15, "Defective motor vibration must be elevated")

    def test_validate_dataset_pass(self):
        """Verify validate_run and validate_dataset report PASS on a compliant run."""
        recorder = MotorExperimentRecorder(
            motor_id="motor_test_C",
            health_label="healthy",
            environment="quiet",
            duration=1.0,
            output_dir=self.test_dir,
            use_synthetic=True
        )
        recorder.run()

        # Test single run validation
        report = validate_run(recorder.run_dir)
        self.assertTrue(report["valid"], f"Run should be valid, got errors: {report['errors']}")
        self.assertEqual(len(report["errors"]), 0)
        self.assertTrue(report["is_synthetic"])

        # Test full dataset validation
        ds_report = validate_dataset(self.test_dir)
        self.assertTrue(ds_report["manifest_valid"])
        self.assertEqual(ds_report["total_disk_runs"], 1)
        self.assertEqual(ds_report["valid_runs_count"], 1)
        self.assertEqual(ds_report["invalid_runs_count"], 0)
        self.assertEqual(ds_report["synthetic_runs_count"], 1)
        self.assertEqual(ds_report["physical_runs_count"], 0)

    def test_validate_dataset_catches_missing_artifact(self):
        """Verify validator fails when an essential artifact is missing."""
        recorder = MotorExperimentRecorder(
            motor_id="motor_test_D",
            health_label="healthy",
            environment="quiet",
            duration=1.0,
            output_dir=self.test_dir,
            use_synthetic=True
        )
        recorder.run()

        # Delete vibration.csv
        os.remove(os.path.join(recorder.run_dir, "vibration.csv"))

        report = validate_run(recorder.run_dir)
        self.assertFalse(report["valid"], "Run must fail validation when vibration.csv is missing")
        self.assertTrue(any("vibration.csv" in err for err in report["errors"]))

    def test_validate_dataset_catches_corrupted_audio(self):
        """Verify validator fails when audio has wrong channel count."""
        recorder = MotorExperimentRecorder(
            motor_id="motor_test_E",
            health_label="healthy",
            environment="quiet",
            duration=1.0,
            output_dir=self.test_dir,
            use_synthetic=True
        )
        recorder.run()

        # Overwrite audio_4ch.wav with a 2-channel audio file
        audio_path = os.path.join(recorder.run_dir, "audio_4ch.wav")
        fake_stereo = np.zeros((1000, 2), dtype=np.int16)
        with wave.open(audio_path, "wb") as wf:
            wf.setnchannels(2)
            wf.setsampwidth(2)
            wf.setframerate(10000)
            wf.writeframes(fake_stereo.tobytes())

        report = validate_run(recorder.run_dir)
        self.assertFalse(report["valid"], "Run must fail validation when audio channels != 4")
        self.assertTrue(any("channels" in err for err in report["errors"]))

    def test_manifest_incremental_update(self):
        """Verify manifest records multiple distinct runs without corruption."""
        for i in range(3):
            rec = MotorExperimentRecorder(
                motor_id=f"motor_{i}",
                health_label="healthy" if i % 2 == 0 else "defective",
                environment="quiet" if i < 2 else "disturbance",
                duration=0.5,
                output_dir=self.test_dir,
                use_synthetic=True
            )
            rec.run()

        manifest_path = os.path.join(self.test_dir, "manifest.json")
        with open(manifest_path, "r", encoding="utf-8") as f:
            manifest = json.load(f)

        self.assertEqual(manifest["total_runs"], 3)
        self.assertEqual(len(manifest["runs"]), 3)
        unique_ids = set(r["run_id"] for r in manifest["runs"])
        self.assertEqual(len(unique_ids), 3, "All run_ids must be unique")


if __name__ == "__main__":
    unittest.main()
