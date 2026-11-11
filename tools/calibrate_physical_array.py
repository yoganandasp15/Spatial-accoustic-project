#!/usr/bin/env python3
"""
=============================================================================
 1-CLICK PHYSICAL ACOUSTIC ARRAY BENCHMARK & GROUND-TRUTH CALIBRATOR
=============================================================================
Module: tools/calibrate_physical_array.py
Target: Validates physical 4-microphone INMP441 array localization accuracy
        across known physical ground-truth angles (0°, +30°, -30°, +45°, -45°).
        Calculates GCC-PHAT TDOA, SRP-PHAT angular spectrum, and angular error.
        Generates publication plot: docs/evidence/physical_acoustic_localization_benchmark.png
        and updates stage2_acoustic_localization/evidence/physical_acoustic_calibration.json.
Author: Antigravity Autonomous Agent (Minor Project Suite)
=============================================================================
"""

import os
import sys
import time
import struct
import argparse
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAGE4_SRC = os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src")
if STAGE4_SRC not in sys.path:
    sys.path.insert(0, STAGE4_SRC)

from spatial_acoustic_radar_pro import SRPPhatEngine, PRESETS

EVIDENCE_DIR = os.path.join(BASE_DIR, "docs", "evidence")
STAGE2_DIR = os.path.join(BASE_DIR, "stage2_acoustic_localization", "evidence")
os.makedirs(EVIDENCE_DIR, exist_ok=True)
os.makedirs(STAGE2_DIR, exist_ok=True)


def record_physical_audio(duration: float = 3.0, port: str = "auto", baud: int = 921600):
    """Attempts to record 4-channel audio from ESP32; falls back cleanly if unpowered."""
    if port == "auto" or not port:
        try:
            from find_port import detect_port
            port = detect_port()
        except Exception:
            port = "COM8"
    try:
        import serial
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.25)
        ser.dtr = False
        ser.rts = False
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.write(b"S\n")
        ser.flush()

        HEADER_FORMAT = "<BBBBHHHBBI"
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
        rx_buf = bytearray()
        frames = []
        t0 = time.time()

        while time.time() - t0 < duration:
            n = ser.in_waiting
            if n > 0:
                rx_buf.extend(ser.read(n))

            while len(rx_buf) >= HEADER_SIZE:
                sync_idx = -1
                for i in range(len(rx_buf) - 1):
                    if rx_buf[i] == 0xAA and rx_buf[i+1] == 0x55:
                        sync_idx = i
                        break
                if sync_idx == -1:
                    rx_buf.clear()
                    break
                if sync_idx > 0:
                    del rx_buf[:sync_idx]
                if len(rx_buf) < HEADER_SIZE:
                    break

                ptype = rx_buf[2]
                if ptype == 0x05:
                    s1, s2, pt, ch, seq, fs_val, blk, num_vib, res, pulses = struct.unpack_from(HEADER_FORMAT, rx_buf, 0)
                    total_len = HEADER_SIZE + num_vib * 6 + blk * ch * 2 + 2
                    if len(rx_buf) < total_len:
                        break
                    audio_bytes = bytes(rx_buf[HEADER_SIZE + num_vib * 6 : HEADER_SIZE + num_vib * 6 + blk * ch * 2])
                    raw_audio = np.frombuffer(audio_bytes, dtype=np.int16).reshape((blk, ch)).astype(np.float32) / 32768.0
                    frames.append(raw_audio)
                    del rx_buf[:total_len]
                else:
                    del rx_buf[:2]

        ser.close()
        if len(frames) >= 10:
            return np.vstack(frames), 10000.0
    except Exception:
        pass
    return None, 10000.0


def simulate_known_angle_audio(target_angle_deg: float, duration: float = 3.0, fs: float = 10000.0):
    """Simulates real physical audio arriving at the 4-mic INMP441 ULA with fractional acoustic delay."""
    n_samples = int(duration * fs)
    t = np.arange(n_samples) / fs
    d_pitch = 0.02032  # 20.32 mm inter-element pitch
    mic_x = np.array([-1.5, -0.5, 0.5, 1.5]) * d_pitch
    c_sound = 343.0
    rad = np.deg2rad(target_angle_deg)

    # Broadband acoustic test signal (speech & motor harmonics in 500-3000 Hz band)
    base_sig = (
        0.50 * np.sin(2 * np.pi * 950.0 * t) +
        0.30 * np.sin(2 * np.pi * 1850.0 * t) +
        0.15 * np.sin(2 * np.pi * 2600.0 * t) +
        0.05 * np.random.randn(n_samples)
    )

    out = np.zeros((n_samples, 4), dtype=np.float32)
    for m in range(4):
        tau = (mic_x[m] * np.sin(rad)) / c_sound
        freqs = np.fft.rfftfreq(n_samples, d=1.0 / fs)
        phase_shift = np.exp(+2j * np.pi * freqs * tau)
        sig_fft = np.fft.rfft(base_sig)
        m_sig = np.fft.irfft(sig_fft * phase_shift, n=n_samples)
        # Add slight room reflection and sensor noise
        m_sig += 0.03 * np.random.randn(n_samples)
        out[:, m] = np.clip(m_sig, -0.95, 0.95)

    return out, fs


def run_benchmark(test_angles: List[float], port: str = "auto", baud: int = 921600, use_synthetic: bool = False):
    if port == "auto" or not port:
        try:
            from find_port import detect_port
            port = detect_port()
        except Exception:
            port = "COM8"
    print("=" * 75)
    print(" 4-MICROPHONE INMP441 PHYSICAL ACOUSTIC LOCALIZATION BENCHMARK")
    print("=" * 75)
    print(f" Array Geometry : 4-Mic Uniform Linear Array (ULA), Pitch: 20.32 mm (Span: 60.96 mm)")
    print(f" Target Angles  : {test_angles}")
    print(f" Target Serial  : {port} @ {baud} Baud")
    print("=" * 75 + "\n")

    geom = PRESETS["ESP32_4MIC_LINEAR"]
    results = []
    srp_profiles = {}

    for theta_true in test_angles:
        print(f"[*] Testing Angle: {theta_true:+.1f}°")
        audio, fs = None, 10000.0
        if not use_synthetic:
            audio, fs = record_physical_audio(duration=2.5, port=port, baud=baud)
            if audio is None:
                print(f"    [-] ERROR: ESP32 on {port} is disconnected or not streaming.")
                print(f"        Physical calibration requires live acoustic recordings at {theta_true:+.1f}°.")
                print(f"        Benchmark aborted without generating or writing fake calibration data.")
                return None
        else:
            print(f"    [!] NOTE: Explicit --synthetic simulation mode active.")
            audio, fs = simulate_known_angle_audio(theta_true, duration=2.5, fs=10000.0)

        # Run SRP-PHAT beamforming engine across all 6 pairs
        best_bearing, comp_prom, angles, srp_norm, heatmap, side = SRPPhatEngine.compute_srp_frame(
            audio, fs, geom, min_active_rms=0.0010
        )

        error_deg = abs(best_bearing - theta_true)
        results.append({
            "true_deg": theta_true,
            "measured_deg": round(best_bearing, 2),
            "error_deg": round(error_deg, 2),
            "prominence": round(comp_prom, 2),
            "dominant_side": side.get("dom_side", "CENTER")
        })
        srp_profiles[theta_true] = (angles, srp_norm)

        print(f"    [+] Measured: {best_bearing:+6.2f}° | Error: {error_deg:4.2f}° | Prominence: {comp_prom:4.2f} | Side: {side.get('dom_side')}")

    # Compute Summary Statistics
    errors = [r["error_deg"] for r in results]
    mean_err = float(np.mean(errors))
    median_err = float(np.median(errors))
    max_err = float(np.max(errors))

    print("\n" + "=" * 75)
    print(f" ACOUSTIC LOCALIZATION ACCURACY SUMMARY")
    print(f"   Mean Angular Error   : {mean_err:.2f}°")
    print(f"   Median Angular Error : {median_err:.2f}°")
    print(f"   Max Angular Error    : {max_err:.2f}°")
    print("=" * 75 + "\n")

    # Plot Publication Figure
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(13, 8), facecolor="#0a0f18")

    # Colors
    c_cyan = "#00ffcc"
    c_pink = "#ff0055"
    c_grid = "#1c2e4a"

    # Panel 1: True vs Measured Angle
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_facecolor("#0e1726")
    true_arr = [r["true_deg"] for r in results]
    meas_arr = [r["measured_deg"] for r in results]
    ax1.plot([-60, 60], [-60, 60], color="#64748b", linestyle="--", label="Ideal $y = x$")
    ax1.scatter(true_arr, meas_arr, color=c_cyan, s=90, edgecolors="#ffffff", lw=1.5, zorder=5, label="Measured Bearing")
    ax1.fill_between([-60, 60], [-62, 58], [-58, 62], color=c_cyan, alpha=0.10, label="±2° Acceptance Band")
    ax1.set_title("1. Ground Truth vs. Measured Acoustic Bearing", fontsize=11, color="#ffffff", pad=8)
    ax1.set_xlabel("True Physical Angle (°)", fontsize=9, color="#94a3b8")
    ax1.set_ylabel("Measured DOA (°)", fontsize=9, color="#94a3b8")
    ax1.set_xlim(-60, 60)
    ax1.set_ylim(-60, 60)
    ax1.grid(True, color=c_grid, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left", facecolor="#0a0f18", edgecolor="#334155", fontsize=8)

    # Panel 2: Angular Error Bar Chart
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_facecolor("#0e1726")
    bar_colors = [c_cyan if e < 1.5 else "#ffaa00" for e in errors]
    ax2.bar([str(t) + "°" for t in true_arr], errors, color=bar_colors, width=0.45, alpha=0.9)
    ax2.axhline(mean_err, color=c_pink, linestyle="--", label=f"Mean Error ({mean_err:.2f}°)")
    ax2.set_title("2. Absolute Angular Localization Error (|$\\theta_{meas} - \\theta_{true}$|)", fontsize=11, color="#ffffff", pad=8)
    ax2.set_ylabel("Error (°)", fontsize=9, color="#94a3b8")
    ax2.grid(True, color=c_grid, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper right", facecolor="#0a0f18", edgecolor="#334155", fontsize=8)

    # Panel 3: SRP-PHAT Angular Profiles across Tested Angles
    ax3 = fig.add_subplot(2, 1, 2)
    ax3.set_facecolor("#0e1726")
    colors = ["#00ffcc", "#38bdf8", "#a855f7", "#ff0055", "#ffaa00"]
    for idx, theta in enumerate(test_angles):
        ang, srp = srp_profiles[theta]
        ax3.plot(ang, srp, color=colors[idx % len(colors)], lw=1.5, label=f"Ground Truth {theta:+.0f}°")
    ax3.set_title("3. Multi-Pair SRP-PHAT Steered Response Power Angular Spectra", fontsize=11, color="#ffffff", pad=8)
    ax3.set_xlabel("Scanning Angle (deg)", fontsize=9, color="#94a3b8")
    ax3.set_ylabel("Normalized Response Power", fontsize=9, color="#94a3b8")
    ax3.set_xlim(-90, 90)
    ax3.grid(True, color=c_grid, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper right", facecolor="#0a0f18", edgecolor="#334155", fontsize=8)

    fig.tight_layout()
    out_png = os.path.join(EVIDENCE_DIR, "physical_acoustic_localization_benchmark.png")
    fig.savefig(out_png, dpi=200, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    print(f"[+] Saved physical acoustic benchmark plot: {os.path.basename(out_png)}")

    # Save JSON summary
    bench_data = {
        "array": "4-Mic INMP441 Linear Array",
        "provenance": "SYNTHETIC_SIMULATION (EXPLICIT USER FLAG)" if use_synthetic else "PHYSICAL_HARDWARE (4-Mic INMP441 COM8)",
        "pitch_mm": 20.32,
        "span_mm": 60.96,
        "mean_error_deg": round(mean_err, 2),
        "median_error_deg": round(median_err, 2),
        "max_error_deg": round(max_err, 2),
        "tested_angles": results
    }
    json_path = os.path.join(STAGE2_DIR, "physical_acoustic_calibration.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(bench_data, f, indent=2)
    print(f"[+] Saved acoustic calibration metrics: {os.path.relpath(json_path, BASE_DIR)}")

    return bench_data


def main():
    parser = argparse.ArgumentParser(description="Calibrate & Benchmark Physical 4-Mic Acoustic Array")
    parser.add_argument("--angles", nargs="+", type=float, default=[0.0, 30.0, -30.0, 45.0, -45.0],
                        help="List of test angles in degrees (default: 0 30 -30 45 -45)")
    parser.add_argument("--port", type=str, default="auto", help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baudrate (default: 921600)")
    parser.add_argument("--synthetic", action="store_true", help="Force synthetic simulation of physical array")
    args = parser.parse_args()

    run_benchmark(args.angles, port=args.port, baud=args.baud, use_synthetic=args.synthetic)


if __name__ == "__main__":
    main()
