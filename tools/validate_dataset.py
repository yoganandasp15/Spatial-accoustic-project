#!/usr/bin/env python3
"""
=============================================================================
 DATASET QUALITY VALIDATOR & HEALTH AUDITOR (TINYML PHASE 1)
=============================================================================
Module: tools/validate_dataset.py
Description: Validates all recorded multimodal motor experiment runs in
             data/dataset/ for dataset integrity, completeness, and adherence
             to physical acquisition standards:
               1. Presence and schema of metadata.json
               2. 4-Channel 10,000 Hz 16-bit PCM audio_4ch.wav integrity (+/-10% duration)
               3. 400 Hz Signed ADXL345 (+/-2g Full Res) vibration.csv validity
                  - Enforces +/-10% sample count tolerance
                  - Flags near-saturation (>=1.95g) and over-range (>2.05g)
               4. Recording status enforcement: rejects INTERRUPTED, INCOMPLETE, FAILED
               5. Packet sequence continuity, drop rates, and checksum errors
               6. Manifest reconciliation (disk vs manifest.json)
               7. Physical vs Synthetic provenance segregation (strict prohibition
                  of synthetic testbench data in physical training sets)
               8. 2x2 Experimental Matrix balance (Healthy/Defective x Quiet/Disturbance)

Can be executed as CLI or imported programmatically.
=============================================================================
"""

import os
import sys
import json
import wave
import argparse
from typing import Dict, List, Any, Optional, Tuple
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def validate_run(run_dir: str, strict: bool = False, allow_synthetic: bool = False) -> Dict[str, Any]:
    """
    Validates an individual experiment run directory.
    Returns a dictionary of diagnostics and a boolean pass/fail status.
    """
    run_id = os.path.basename(os.path.normpath(run_dir))
    report: Dict[str, Any] = {
        "run_id": run_id,
        "run_dir": run_dir,
        "valid": True,
        "errors": [],
        "warnings": [],
        "provenance": "UNKNOWN",
        "recording_status": "UNKNOWN",
        "is_synthetic": False,
        "metadata": {},
        "audio_stats": {},
        "vibration_stats": {},
        "packet_stats": {},
    }

    # 1. Required Files Presence Check
    required_files = ["metadata.json", "vibration.csv", "audio_4ch.wav", "packet_log.csv"]
    for f in required_files:
        fpath = os.path.join(run_dir, f)
        if not os.path.isfile(fpath):
            report["errors"].append(f"Missing required artifact: {f}")
            report["valid"] = False

    if not report["valid"]:
        return report

    # 2. Metadata Schema & Content Check
    meta_path = os.path.join(run_dir, "metadata.json")
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        report["metadata"] = meta
        report["is_synthetic"] = bool(meta.get("is_synthetic", False) or "SYNTHETIC" in meta.get("data_provenance", ""))
        report["provenance"] = meta.get("data_provenance", "UNKNOWN")
        report["recording_status"] = meta.get("recording_status", "UNKNOWN")

        # Required fields in metadata
        req_meta_fields = [
            "run_id", "session_id", "motor_id", "motor_health_label",
            "acoustic_environment", "target_duration_s", "actual_duration_s",
            "sample_rate_audio_hz", "sample_rate_vib_nominal_hz", "audio_channels",
            "recording_status", "accel_configured_range_g"
        ]
        for rf in req_meta_fields:
            if rf not in meta:
                report["errors"].append(f"metadata.json missing field: {rf}")
                report["valid"] = False

        # Validate categorical values
        health_label = meta.get("motor_health_label", "").lower()
        if health_label not in ("healthy", "defective"):
            report["errors"].append(f"Invalid motor_health_label: '{health_label}' (expected 'healthy' or 'defective')")
            report["valid"] = False

        env = meta.get("acoustic_environment", "").lower()
        if env not in ("quiet", "disturbance"):
            report["errors"].append(f"Invalid acoustic_environment: '{env}' (expected 'quiet' or 'disturbance')")
            report["valid"] = False

        # Recording Status Enforcement
        rec_status = meta.get("recording_status", "UNKNOWN").upper()
        if rec_status != "COMPLETE":
            report["errors"].append(f"Run recording_status is '{rec_status}' (must be 'COMPLETE' for valid model training)")
            report["valid"] = False

        # Sample rates
        if meta.get("sample_rate_audio_hz") != 10000.0:
            report["errors"].append(f"Expected sample_rate_audio_hz 10000.0, got {meta.get('sample_rate_audio_hz')}")
            report["valid"] = False

        if meta.get("sample_rate_vib_nominal_hz") != 400.0:
            report["errors"].append(f"Expected sample_rate_vib_nominal_hz 400.0, got {meta.get('sample_rate_vib_nominal_hz')}")
            report["valid"] = False

        if meta.get("audio_channels") != 4:
            report["errors"].append(f"Expected audio_channels 4, got {meta.get('audio_channels')}")
            report["valid"] = False

        # Checksum errors and dropped packets inspection
        chk_errs = int(meta.get("checksum_errors", 0))
        drops = int(meta.get("dropped_packets", 0))
        total_pkts = int(meta.get("total_packets_received", 0))

        if chk_errs > 0:
            msg = f"Run recorded {chk_errs} checksum error(s) in metadata"
            if strict:
                report["errors"].append(msg)
                report["valid"] = False
            else:
                report["warnings"].append(msg)

        if drops > max(3, int(0.01 * max(1, total_pkts))):
            msg = f"Run observed excessive packet drops: {drops} drops out of {total_pkts} packets"
            if strict:
                report["errors"].append(msg)
                report["valid"] = False
            else:
                report["warnings"].append(msg)

        # Synthetic segregation under strict mode
        if strict and not allow_synthetic and report["is_synthetic"]:
            report["errors"].append("Synthetic data detected. Synthetic runs are prohibited in physical training datasets.")
            report["valid"] = False

    except Exception as e:
        report["errors"].append(f"Failed to parse metadata.json: {str(e)}")
        report["valid"] = False
        return report

    # 3. Audio 4-Channel WAV Validation
    audio_path = os.path.join(run_dir, "audio_4ch.wav")
    try:
        with wave.open(audio_path, "rb") as wf:
            n_ch = wf.getnchannels()
            sampwidth = wf.getsampwidth()
            framerate = wf.getframerate()
            n_frames = wf.getnframes()
            frames_raw = wf.readframes(n_frames)

        if n_ch != 4:
            report["errors"].append(f"audio_4ch.wav has {n_ch} channels (expected 4)")
            report["valid"] = False

        if sampwidth != 2:
            report["errors"].append(f"audio_4ch.wav has sample width {sampwidth} bytes (expected 2 for 16-bit PCM)")
            report["valid"] = False

        if framerate != 10000:
            report["errors"].append(f"audio_4ch.wav has framerate {framerate} Hz (expected 10000 Hz)")
            report["valid"] = False

        duration_wav_s = n_frames / float(framerate) if framerate > 0 else 0.0
        expected_dur = meta.get("actual_duration_s", meta.get("target_duration_s", 0.0))
        audio_dur_tol = max(0.5, 0.10 * expected_dur)  # Enforce +/-10% tolerance
        if abs(duration_wav_s - expected_dur) > audio_dur_tol:
            msg = f"Audio duration mismatch: WAV has {duration_wav_s:.2f}s, expected {expected_dur:.2f}s (+/-10% tol: {audio_dur_tol:.2f}s)"
            if strict:
                report["errors"].append(msg)
                report["valid"] = False
            else:
                report["warnings"].append(msg)

        # Numerical checks on audio
        audio_i16 = np.frombuffer(frames_raw, dtype=np.int16).reshape(-1, n_ch)
        audio_f32 = audio_i16.astype(np.float32) / 32768.0

        rms_per_ch = [float(np.sqrt(np.mean(audio_f32[:, c] ** 2))) for c in range(n_ch)]
        peak_per_ch = [float(np.max(np.abs(audio_f32[:, c]))) for c in range(n_ch)]

        report["audio_stats"] = {
            "total_frames": n_frames,
            "duration_s": round(duration_wav_s, 3),
            "rms_per_ch": [round(r, 5) for r in rms_per_ch],
            "peak_per_ch": [round(p, 5) for p in peak_per_ch],
            "is_silent": all(r < 0.0001 for r in rms_per_ch)
        }

        if report["audio_stats"]["is_silent"]:
            report["warnings"].append("audio_4ch.wav is silent (RMS < 0.0001 on all 4 channels)")

    except Exception as e:
        report["errors"].append(f"Failed to read/validate audio_4ch.wav: {str(e)}")
        report["valid"] = False

    # 4. Vibration CSV Validation (Configured +/-2g Full Resolution Range)
    vib_path = os.path.join(run_dir, "vibration.csv")
    try:
        with open(vib_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]

        if len(lines) < 2:
            report["errors"].append("vibration.csv is empty or missing data rows")
            report["valid"] = False
        else:
            header = [c.strip() for c in lines[0].split(",")]
            req_cols = ["sample_idx", "host_time_s", "accel_x_g", "accel_y_g", "accel_z_g", "dyn_mag_g", "tach_pulses"]
            for col in req_cols:
                if col not in header:
                    report["errors"].append(f"vibration.csv missing required column: '{col}'")
                    report["valid"] = False

            if report["valid"]:
                data_rows = [line.split(",") for line in lines[1:]]
                col_map = {name: i for i, name in enumerate(header)}

                # Sample count tolerance: strictly +/-10%
                expected_samples = int(meta.get("actual_duration_s", 0.0) * 400.0)
                sample_tol = max(20, int(0.10 * expected_samples))
                if abs(len(data_rows) - expected_samples) > sample_tol:
                    msg = f"Vibration sample count discrepancy: got {len(data_rows)}, expected ~{expected_samples} (+/-10% tol: {sample_tol})"
                    if strict:
                        report["errors"].append(msg)
                        report["valid"] = False
                    else:
                        report["warnings"].append(msg)

                # Parse acceleration columns
                ax = np.array([float(r[col_map["accel_x_g"]]) for r in data_rows])
                ay = np.array([float(r[col_map["accel_y_g"]]) for r in data_rows])
                az = np.array([float(r[col_map["accel_z_g"]]) for r in data_rows])
                dyn = np.array([float(r[col_map["dyn_mag_g"]]) for r in data_rows])
                tach = np.array([int(r[col_map["tach_pulses"]]) for r in data_rows])

                # Check for NaNs or Infs
                for name, arr in [("accel_x", ax), ("accel_y", ay), ("accel_z", az), ("dyn_mag", dyn)]:
                    if not np.all(np.isfinite(arr)):
                        report["errors"].append(f"vibration.csv contains non-finite (NaN/Inf) values in {name}")
                        report["valid"] = False

                # Physical range and saturation check against configured range (+/-2g)
                configured_range_g = float(meta.get("accel_configured_range_g", 2.0))
                near_sat_thresh = configured_range_g * 0.975   # 1.95g for 2.0g range
                over_range_thresh = configured_range_g * 1.025 # 2.05g for 2.0g range

                for name, arr in [("accel_x", ax), ("accel_y", ay), ("accel_z", az)]:
                    max_abs = float(np.max(np.abs(arr)))
                    if max_abs > over_range_thresh:
                        report["errors"].append(
                            f"{name} exceeds configured range +/-{configured_range_g:.1f}g: peak observed {max_abs:.3f}g"
                        )
                        report["valid"] = False
                    elif max_abs >= near_sat_thresh:
                        msg = f"{name} approaches sensor saturation: peak observed {max_abs:.3f}g (>= {near_sat_thresh:.2f}g)"
                        if strict and max_abs >= (configured_range_g * 0.99):
                            report["errors"].append(msg)
                            report["valid"] = False
                        else:
                            report["warnings"].append(msg)

                report["vibration_stats"] = {
                    "sample_count": len(data_rows),
                    "mean_accel_x_g": round(float(np.mean(ax)), 4),
                    "mean_accel_y_g": round(float(np.mean(ay)), 4),
                    "mean_accel_z_g": round(float(np.mean(az)), 4),
                    "mean_dyn_mag_g": round(float(np.mean(dyn)), 4),
                    "max_dyn_mag_g": round(float(np.max(dyn)), 4),
                    "total_tach_pulses": int(np.max(tach) - np.min(tach)) if len(tach) > 0 else 0
                }

    except Exception as e:
        report["errors"].append(f"Failed to read/validate vibration.csv: {str(e)}")
        report["valid"] = False

    # 5. Packet Log CSV Validation
    pkt_path = os.path.join(run_dir, "packet_log.csv")
    try:
        with open(pkt_path, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]

        if len(lines) < 2:
            report["errors"].append("packet_log.csv is empty or missing data rows")
            report["valid"] = False
        else:
            header = [c.strip() for c in lines[0].split(",")]
            data_rows = [line.split(",") for line in lines[1:]]
            col_map = {name: i for i, name in enumerate(header)}

            total_pkts = len(data_rows)
            corrupt_count = 0
            if "checksum_valid" in col_map:
                for r in data_rows:
                    if r[col_map["checksum_valid"]].strip().lower() in ("false", "0"):
                        corrupt_count += 1

            total_seq_gaps = 0
            if "seq_gap" in col_map:
                for r in data_rows:
                    try:
                        total_seq_gaps += max(0, int(r[col_map["seq_gap"]]))
                    except Exception:
                        pass

            report["packet_stats"] = {
                "total_packets": total_pkts,
                "corrupt_packets": corrupt_count,
                "seq_gaps": total_seq_gaps
            }

            if corrupt_count > 0:
                msg = f"packet_log.csv logged {corrupt_count} corrupt packet(s) with invalid checksums"
                if strict:
                    report["errors"].append(msg)
                    report["valid"] = False
                else:
                    report["warnings"].append(msg)

            if total_seq_gaps > max(3, int(0.01 * max(1, total_pkts))):
                msg = f"packet_log.csv logged {total_seq_gaps} sequence gaps (packet drops)"
                if strict:
                    report["errors"].append(msg)
                    report["valid"] = False
                else:
                    report["warnings"].append(msg)

    except Exception as e:
        report["errors"].append(f"Failed to read/validate packet_log.csv: {str(e)}")
        report["valid"] = False

    return report


def validate_dataset(dataset_dir: str, strict: bool = False, allow_synthetic: bool = False) -> Dict[str, Any]:
    """
    Validates all run folders in dataset_dir and verifies reconciliation
    against manifest.json.
    """
    dataset_dir = os.path.abspath(dataset_dir)
    manifest_path = os.path.join(dataset_dir, "manifest.json")

    results: Dict[str, Any] = {
        "dataset_dir": dataset_dir,
        "manifest_path": manifest_path,
        "manifest_valid": False,
        "strict_mode": strict,
        "allow_synthetic": allow_synthetic,
        "total_disk_runs": 0,
        "total_manifest_runs": 0,
        "valid_runs_count": 0,
        "invalid_runs_count": 0,
        "physical_runs_count": 0,
        "synthetic_runs_count": 0,
        "matrix_counts": {
            "healthy_quiet": 0,
            "healthy_disturbance": 0,
            "defective_quiet": 0,
            "defective_disturbance": 0,
        },
        "physical_matrix_counts": {
            "healthy_quiet": 0,
            "healthy_disturbance": 0,
            "defective_quiet": 0,
            "defective_disturbance": 0,
        },
        "runs": {},
        "global_errors": [],
        "global_warnings": []
    }

    if not os.path.isdir(dataset_dir):
        results["global_errors"].append(f"Dataset directory not found: {dataset_dir}")
        return results

    # Reconcile Manifest
    manifest_runs_map = {}
    if os.path.isfile(manifest_path):
        try:
            with open(manifest_path, "r", encoding="utf-8") as f:
                manifest_data = json.load(f)
            results["manifest_valid"] = True
            runs_list = manifest_data.get("runs", [])
            results["total_manifest_runs"] = len(runs_list)
            for r in runs_list:
                if "run_id" in r:
                    manifest_runs_map[r["run_id"]] = r
        except Exception as e:
            results["global_errors"].append(f"Failed to parse manifest.json: {str(e)}")
    else:
        results["global_warnings"].append(f"manifest.json not found in {dataset_dir}")

    # Discover run directories on disk
    subdirs = [
        d for d in os.listdir(dataset_dir)
        if os.path.isdir(os.path.join(dataset_dir, d)) and not d.startswith(".")
    ]
    results["total_disk_runs"] = len(subdirs)

    # Check for orphaned entries in manifest
    for m_id in manifest_runs_map:
        if m_id not in subdirs:
            results["global_warnings"].append(f"Manifest lists run '{m_id}' but directory does not exist on disk")

    # Validate each run on disk
    for d in sorted(subdirs):
        r_dir = os.path.join(dataset_dir, d)
        run_rep = validate_run(r_dir, strict=strict, allow_synthetic=allow_synthetic)
        results["runs"][d] = run_rep

        if d not in manifest_runs_map:
            run_rep["warnings"].append(f"Run directory '{d}' is not registered in manifest.json")

        if run_rep["valid"]:
            results["valid_runs_count"] += 1
        else:
            results["invalid_runs_count"] += 1

        if run_rep["is_synthetic"]:
            results["synthetic_runs_count"] += 1
        else:
            results["physical_runs_count"] += 1

        # Matrix binning
        meta = run_rep.get("metadata", {})
        h_label = meta.get("motor_health_label", "").lower()
        env = meta.get("acoustic_environment", "").lower()
        k = f"{h_label}_{env}"
        if k in results["matrix_counts"]:
            results["matrix_counts"][k] += 1
            if not run_rep["is_synthetic"]:
                results["physical_matrix_counts"][k] += 1

    # Check 2x2 Matrix completeness
    pm = results["physical_matrix_counts"]
    missing_physical = [cond for cond, count in pm.items() if count == 0]
    results["matrix_complete"] = (len(missing_physical) == 0 and results["physical_runs_count"] >= 4)
    results["missing_conditions"] = missing_physical

    return results


def print_validation_report(results: Dict[str, Any]):
    """Pretty prints the dataset validation scorecard to console."""
    print("\n" + "=" * 80)
    print("            SPATIAL MULTIMODAL MOTOR DATASET QUALITY REPORT")
    print("=" * 80)
    print(f" Dataset Directory : {results['dataset_dir']}")
    print(f" Strict Mode       : {'ENABLED' if results.get('strict_mode') else 'DISABLED'}")
    print(f" Manifest Status   : {'VALID' if results['manifest_valid'] else 'INVALID / MISSING'}")
    print(f" Total Runs On Disk: {results['total_disk_runs']}")
    print(f"   - Valid Runs    : {results['valid_runs_count']}")
    print(f"   - Invalid Runs  : {results['invalid_runs_count']}")
    print(f" Provenance Split  : {results['physical_runs_count']} Physical | {results['synthetic_runs_count']} Synthetic")
    print("-" * 80)
    print(" 2x2 EXPERIMENTAL MATRIX (Physical / Total):")
    pm = results["physical_matrix_counts"]
    tm = results["matrix_counts"]
    print(f"   [1] HEALTHY   + QUIET       : {pm['healthy_quiet']} physical / {tm['healthy_quiet']} total")
    print(f"   [2] HEALTHY   + DISTURBANCE : {pm['healthy_disturbance']} physical / {tm['healthy_disturbance']} total")
    print(f"   [3] DEFECTIVE + QUIET       : {pm['defective_quiet']} physical / {tm['defective_quiet']} total")
    print(f"   [4] DEFECTIVE + DISTURBANCE : {pm['defective_disturbance']} physical / {tm['defective_disturbance']} total")
    cov_str = "COMPLETE (All 4 physical cells observed)" if results.get("matrix_complete") else f"INCOMPLETE (Missing: {', '.join(results.get('missing_conditions', []))})"
    print(f"   --> Matrix Coverage         : {cov_str}")
    print("-" * 80)

    # Detailed per-run breakdown
    print(f"{'RUN ID':<46} {'PROVENANCE':<12} {'HEALTH':<10} {'STATUS':<10} {'VALID'}")
    print("-" * 80)
    for rid, rep in results["runs"].items():
        meta = rep.get("metadata", {})
        prov = "SYNTHETIC" if rep["is_synthetic"] else "PHYSICAL"
        h = meta.get("motor_health_label", "???").upper()
        st = meta.get("recording_status", "???").upper()
        v_str = "PASS" if rep["valid"] else "FAIL"
        print(f"{rid:<46} {prov:<12} {h:<10} {st:<10} {v_str}")
        if rep["errors"]:
            for err in rep["errors"]:
                print(f"   [!] ERROR: {err}")
        if rep["warnings"]:
            for warn in rep["warnings"]:
                print(f"   [*] WARN : {warn}")

    print("=" * 80)
    if results["global_errors"]:
        print("[CRITICAL DATASET ERRORS]")
        for ge in results["global_errors"]:
            print(f"  [X] {ge}")
        print("=" * 80)

    if results["synthetic_runs_count"] > 0 and not results.get("allow_synthetic"):
        print("[!] NOTICE: Synthetic runs detected. Physical dataset collection must be completed")
        print("    before model training. Synthetic data must NOT be mixed with physical training sets.")
        print("=" * 80 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Audit and Validate Multimodal Motor Dataset")
    parser.add_argument("--dataset-dir", type=str, default=os.path.join(BASE_DIR, "data", "dataset"),
                        help="Path to data/dataset directory (default: data/dataset)")
    parser.add_argument("--report", type=str, default=None,
                        help="Optional JSON report output file")
    parser.add_argument("--strict", action="store_true",
                        help="Exit with code 1 if any invalid runs, corruptions, interruptions, or synthetic data exist")
    parser.add_argument("--allow-synthetic", action="store_true",
                        help="Permit synthetic data in strict mode (for CI/unit test verification)")
    parser.add_argument("--require-matrix", action="store_true",
                        help="Exit with code 1 if any of the 4 physical matrix cells is missing")

    args = parser.parse_args()
    res = validate_dataset(args.dataset_dir, strict=args.strict, allow_synthetic=args.allow_synthetic)
    print_validation_report(res)

    # Write JSON report if requested or default
    report_path = args.report if args.report else os.path.join(args.dataset_dir, "validation_report.json")
    if os.path.isdir(args.dataset_dir):
        try:
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(res, f, indent=2)
            print(f"[+] Validation report saved to: {os.path.relpath(report_path, BASE_DIR)}")
        except Exception as e:
            print(f"[!] Failed to write validation report: {e}")

    # Strict mode exit code evaluation
    if args.strict:
        if res["invalid_runs_count"] > 0 or res["global_errors"]:
            sys.exit(1)
        if not args.allow_synthetic and res["synthetic_runs_count"] > 0:
            print("[STRICT FAIL] Synthetic data present in physical dataset.")
            sys.exit(1)
        if args.require_matrix and not res.get("matrix_complete"):
            print(f"[STRICT FAIL] Incomplete 2x2 matrix: missing {res.get('missing_conditions')}")
            sys.exit(1)


if __name__ == "__main__":
    main()
