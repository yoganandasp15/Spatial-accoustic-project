"""
=============================================================================
 OFFLINE MULTIMODAL FAULT EVALUATION & ACOUSTIC LOCALIZATION ENGINE
=============================================================================
Module: offline_multimodal_evaluator.py
Purpose: Complete non-blocking offline evaluation of real 4-channel acoustic
         and 3-axis vibration records, co-fusing Steered Response Power (SRP-PHAT)
         with ISO 10816 vibration severity and rotational order tracking.
Author: Antigravity Autonomous Agent (Pair Programming with Yogi)
Target Milestone: ECE Minor Project Evaluation (7 November 2026)
=============================================================================
"""

import os
import sys
import wave
import json
import argparse
import numpy as np
import pandas as pd
from scipy import signal
from scipy.stats import kurtosis
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# Add cross-stage source directories to sys.path
CUR_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(CUR_DIR))
for p in [
    CUR_DIR,
    os.path.join(PROJECT_ROOT, "stage1_vibration_and_tachometer", "src"),
    os.path.join(PROJECT_ROOT, "stage2_acoustic_localization", "src"),
    os.path.join(PROJECT_ROOT, "src"),
]:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

# Import local order tracking and phase calibration modules
from order_tracking import compute_order_tracking
from calibrate_i2s_phase import gcc_phat, load_wav_multichannel


SPEED_OF_SOUND = 343.0  # m/s in air at 20°C

# 4-Microphone Uniform Linear Array Geometry (matching physical breadboard)
DEFAULT_ARRAY_GEOMETRY = {
    "spacing_m": 0.02032,  # 20.32 mm spacing
    "mic_x": np.array([-0.03048, -0.01016, +0.01016, +0.03048], dtype=np.float64),
    "mic_y": np.zeros(4, dtype=np.float64)
}

# 2D Spatial Search Grid (meters)
X_MIN, X_MAX = -2.0, 2.0
Y_MIN, Y_MAX =  0.2, 3.5
GRID_X = 51
GRID_Y = 45


def apply_pair_phase_compensation(
    audio_4ch: np.ndarray,
    fs: float,
    comp_delay_sec: float
) -> np.ndarray:
    """
    Applies exact frequency-domain fractional delay phase compensation to Pair B (channels 2 & 3):
    H(f) = exp(-j * 2 * pi * f * comp_delay_sec)
    """
    if abs(comp_delay_sec) < 1e-12:
        return audio_4ch.copy()
        
    n_samples, n_channels = audio_4ch.shape
    n_pad = 128
    n_fft = n_samples + n_pad
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / fs)
    phase_shift = np.exp(-2j * np.pi * freqs * comp_delay_sec)
    
    aligned = audio_4ch.copy()
    for ch in [2, 3]:
        if ch < n_channels:
            padded = np.pad(audio_4ch[:, ch], (0, n_pad), mode="constant")
            ch_fft = np.fft.rfft(padded, n=n_fft)
            compensated = np.fft.irfft(ch_fft * phase_shift, n=n_fft)
            aligned[:, ch] = compensated[:n_samples]
            
    return aligned


def compute_srp_phat_grid(
    audio_4ch: np.ndarray,
    fs: float,
    mic_x: np.ndarray,
    phase_comp_samples: int = 0,
    comp_delay_sec: float = 0.0
) -> dict:
    """
    Computes 2D Steered Response Power using Phase Transform (SRP-PHAT)
    over a 2D floor grid in front of the array.
    Applies frequency-domain fractional delay compensation on Pair B (channels 2 & 3).
    """
    n_samples, n_channels = audio_4ch.shape
    if n_channels < 4:
        raise ValueError(f"SRP-PHAT requires 4 channels, got {n_channels}.")
        
    # If integer sample shift specified and comp_delay_sec not given, convert to physical seconds
    if abs(comp_delay_sec) < 1e-12 and phase_comp_samples != 0:
        comp_delay_sec = float(phase_comp_samples) / fs
        
    audio_aligned = apply_pair_phase_compensation(audio_4ch, fs, comp_delay_sec)
            
    # Bandpass filter for audio (DC reject and anti-aliasing)
    nyq = 0.5 * fs
    low_cutoff = max(100.0, 0.01 * nyq)
    high_cutoff = min(5000.0, 0.45 * nyq)
    b, a = signal.butter(4, [low_cutoff / nyq, high_cutoff / nyq], btype='bandpass')
    filtered = np.zeros_like(audio_aligned)
    for ch in range(n_channels):
        filtered[:, ch] = signal.filtfilt(b, a, audio_aligned[:, ch])
        
    # Grid coordinates
    xs = np.linspace(X_MIN, X_MAX, GRID_X)
    ys = np.linspace(Y_MIN, Y_MAX, GRID_Y)
    XX, YY = np.meshgrid(xs, ys)
    
    # 6 Baseline Pairs
    pairs = [
        (0, 1), (0, 2), (0, 3),
        (1, 2), (1, 3), (2, 3)
    ]
    
    # Compute GCC-PHAT cross-correlation for all 6 pairs using frequency-domain sinc interpolation
    interp_factor = 8
    n_interp = n_samples * interp_factor
    n_freq = n_interp // 2 + 1
    dt = 1.0 / (fs * interp_factor)
    lags = np.arange(-n_interp // 2, n_interp // 2) * dt
    
    cc_pairs = {}
    for (m1, m2) in pairs:
        X1 = np.fft.rfft(filtered[:, m1])
        X2 = np.fft.rfft(filtered[:, m2])
        R = X1 * np.conj(X2)
        denom = np.abs(R)
        denom = np.maximum(denom, 1e-4 * np.max(denom))
        R_phat = np.where(denom > 1e-15, R / denom, 0.0)
        
        # Zero-pad in frequency domain for true time-domain sinc interpolation
        cross_pad = np.zeros(n_freq, dtype=complex)
        cross_pad[:len(R_phat)] = R_phat
        
        cc = np.fft.fftshift(np.fft.irfft(cross_pad, n=n_interp))
        cc_pairs[(m1, m2)] = cc
        
    # Accumulate Steered Response Power across the 2D grid
    srp_map = np.zeros((GRID_Y, GRID_X), dtype=np.float64)
    
    for (m1, m2), cc_data in cc_pairs.items():
        # Distance from grid points to mic m1 and mic m2
        d1 = np.sqrt((XX - mic_x[m1])**2 + YY**2)
        d2 = np.sqrt((XX - mic_x[m2])**2 + YY**2)
        # Propagation delay tau_12 = (d1 - d2) / c
        tau_grid = (d1 - d2) / SPEED_OF_SOUND
        
        # Convert delay to index in cc_data
        lag_idx = np.round((tau_grid - lags[0]) / dt).astype(np.int32)
        lag_idx = np.clip(lag_idx, 0, len(cc_data) - 1)
        
        srp_map += cc_data[lag_idx]
        
    # Normalize SRP map
    srp_min = np.min(srp_map)
    srp_max = np.max(srp_map)
    srp_norm = (srp_map - srp_min) / (srp_max - srp_min + 1e-12)
    
    # Locate peak coordinates
    max_idx = np.unravel_index(np.argmax(srp_norm), srp_norm.shape)
    peak_y = ys[max_idx[0]]
    peak_x = xs[max_idx[1]]
    
    # Calculate Angle of Arrival (AoA) in degrees
    # 0 deg is broadside (straight ahead along +Y axis)
    aoa_deg = np.degrees(np.arctan2(peak_x, peak_y))
    grid_peak_radius_m = np.sqrt(peak_x**2 + peak_y**2)
    
    # Calculate peak prominence (contrast against mean background)
    mean_bg = np.mean(srp_norm)
    prominence = (1.0 - mean_bg) / (np.std(srp_norm) + 1e-12)
    
    return {
        "srp_norm": srp_norm,
        "XX": XX,
        "YY": YY,
        "xs": xs,
        "ys": ys,
        "peak_x": float(peak_x),
        "peak_y": float(peak_y),
        "aoa_deg": float(aoa_deg),
        "grid_peak_radius_m": float(grid_peak_radius_m),
        "radial_dist_m": float(grid_peak_radius_m),  # Backwards compatibility
        "radial_note": "1D ULA cannot measure physical distance without standoff constraint; radial peak represents backprojection grid ray.",
        "prominence": float(prominence),
        "cc_pairs": cc_pairs,
        "lags": lags
    }


def evaluate_vibration_record(csv_path: str) -> dict:
    """
    Evaluates 3-axis vibration record with per-axis DC gravity removal,
    preserving signed directional dynamic vibration signals (ax, ay, az).
    Computes Project Dynamic Vibration Severity Index (Empirical Acceleration Proxy in g).
    """
    df = pd.read_csv(csv_path)
    col_map = {str(c).lower().strip(): c for c in df.columns}
    
    # Extract acceleration channels
    ax = df[col_map.get("accel_x_g", df.columns[4])].values.astype(np.float64)
    ay = df[col_map.get("accel_y_g", df.columns[5])].values.astype(np.float64)
    az = df[col_map.get("accel_z_g", df.columns[6])].values.astype(np.float64)
    
    # Per-axis gravity DC removal preserving signed directional dynamic signals
    ax_mean = float(np.mean(ax))
    ay_mean = float(np.mean(ay))
    az_mean = float(np.mean(az))

    ax_dyn = ax - ax_mean
    ay_dyn = ay - ay_mean
    az_dyn = az - az_mean

    rms_x = float(np.sqrt(np.mean(ax_dyn**2)))
    rms_y = float(np.sqrt(np.mean(ay_dyn**2)))
    rms_z = float(np.sqrt(np.mean(az_dyn**2)))

    # Instantaneous dynamic AC vector magnitude (Euclidean norm of AC deviations)
    a_dyn_mag = np.sqrt(ax_dyn**2 + ay_dyn**2 + az_dyn**2)
    rms_dyn = float(np.sqrt(np.mean(a_dyn_mag**2)))
    peak_dyn = float(np.max(a_dyn_mag))
    crest_factor_vib = float(peak_dyn / (rms_dyn + 1e-12))
    kurtosis_vib = float(kurtosis(az_dyn, fisher=False))  # On signed radial/dominant vertical channel
    dc_gravity = float(np.sqrt(ax_mean**2 + ay_mean**2 + az_mean**2))

    # Project Dynamic Vibration Severity Index (Empirical proxy in g)
    # Note: Formal ISO 10816-3 requires velocity RMS (mm/s) over 10 Hz - 1000 Hz.
    if rms_dyn < 0.15:
        severity_zone = "Zone A: Good / Smooth (Dynamic Accel Index < 0.15g)"
        vib_score = 95.0
    elif rms_dyn < 0.35:
        severity_zone = "Zone B: Permissible (Dynamic Accel Index 0.15g - 0.35g)"
        vib_score = 80.0
    elif rms_dyn < 0.70:
        severity_zone = "Zone C: Warning (Dynamic Accel Index 0.35g - 0.70g)"
        vib_score = 50.0
    else:
        severity_zone = "Zone D: Alarm / Stop (Dynamic Accel Index >= 0.70g)"
        vib_score = 20.0

    return {
        "rms_dyn_g": rms_dyn,
        "peak_dyn_g": peak_dyn,
        "crest_factor": crest_factor_vib,
        "kurtosis": kurtosis_vib,
        "dc_gravity_g": float(dc_gravity),
        "severity_zone": severity_zone,
        "iso_zone": severity_zone,  # Retained for backwards compatibility
        "vib_score": vib_score,
        "ax_mean": ax_mean,
        "ay_mean": ay_mean,
        "az_mean": az_mean,
        "ax_dyn": ax_dyn,
        "ay_dyn": ay_dyn,
        "az_dyn": az_dyn,
        "a_dyn_mag": a_dyn_mag,
        "a_dyn": a_dyn_mag
    }


def generate_multimodal_dashboard_plot(
    audio_4ch: np.ndarray,
    fs: float,
    srp_res: dict,
    vib_res: dict,
    ot_res: dict,
    output_png: str
) -> None:
    """Generates a comprehensive 4-panel multimodal diagnostic plot."""
    fig = plt.figure(figsize=(16, 12))
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.28, wspace=0.25)
    
    # Panel 1: 4-Channel Raw Audio Waveforms
    ax1 = fig.add_subplot(gs[0, 0])
    t_audio = np.arange(len(audio_4ch)) / fs * 1000.0  # ms
    colors = ['#0072B2', '#009E73', '#D55E00', '#CC79A7']
    labels = ['Mic 1 (Pair A Left)', 'Mic 2 (Pair A Right)', 'Mic 3 (Pair B Left)', 'Mic 4 (Pair B Right)']
    for ch in range(4):
        ax1.plot(t_audio, audio_4ch[:, ch] + ch * 0.4, color=colors[ch], label=labels[ch], lw=1.0)
    ax1.set_title("4-Channel I2S Audio Burst (Acoustic Array)", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Time (ms)", fontsize=9)
    ax1.set_ylabel("Normalized Channel Amplitude + Offset", fontsize=9)
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # Panel 2: 2D Steered Response Power (SRP-PHAT) Spatial Heatmap
    ax2 = fig.add_subplot(gs[0, 1])
    srp_norm = srp_res["srp_norm"]
    XX = srp_res["XX"]
    YY = srp_res["YY"]
    im = ax2.pcolormesh(XX, YY, srp_norm, cmap='inferno', shading='auto')
    plt.colorbar(im, ax=ax2, label="Normalized Acoustic Power", fraction=0.046, pad=0.04)
    
    # Mark mic positions
    mic_x = DEFAULT_ARRAY_GEOMETRY["mic_x"]
    ax2.scatter(mic_x, np.zeros_like(mic_x), color='cyan', marker='^', s=80, edgecolors='black', label='Mics 1-4', zorder=5)
    
    # Mark detected source peak
    peak_x = srp_res["peak_x"]
    peak_y = srp_res["peak_y"]
    aoa = srp_res["aoa_deg"]
    dist = srp_res.get("grid_peak_radius_m", srp_res["radial_dist_m"])
    ax2.scatter([peak_x], [peak_y], color='lime', marker='*', s=200, edgecolors='black', label=f'Peak ({aoa:+.1f}°, {dist:.2f}m ray)', zorder=6)
    
    # Direction vector line
    ax2.plot([0, peak_x * 1.2], [0, peak_y * 1.2], color='lime', linestyle='--', lw=1.5)
    
    ax2.set_title(f"2D Acoustic SRP-PHAT Backprojection (Bearing: {aoa:+.1f}° | Grid Radius: {dist:.2f} m [DOA Ray])", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Lateral Position X (m)", fontsize=9)
    ax2.set_ylabel("Forward Standoff Range Y (m)", fontsize=9)
    ax2.set_xlim(X_MIN, X_MAX)
    ax2.set_ylim(0.0, Y_MAX)
    ax2.legend(loc="upper left", fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # Panel 3: Dynamic Vibration Time Series & Project Dynamic Severity Index
    ax3 = fig.add_subplot(gs[1, 0])
    a_dyn = vib_res.get("a_dyn_mag", vib_res["a_dyn"])
    t_vib = np.arange(len(a_dyn)) / 400.0 * 1000.0  # ms
    ax3.plot(t_vib, a_dyn, color='#d95f02', lw=1.4, label='Dynamic Vib Vector |a_dyn|')
    ax3.axhline(vib_res["rms_dyn_g"], color='blue', linestyle='--', lw=1.5, label=f'RMS = {vib_res["rms_dyn_g"]:.3f} g')
    ax3.axhline(0.15, color='green', linestyle=':', lw=1.2, label='Zone A (<0.15g)')
    ax3.axhline(0.35, color='orange', linestyle=':', lw=1.2, label='Zone B/C (0.35g)')
    ax3.set_title(f"Project Vibration Severity: {vib_res['severity_zone']}", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Time (ms)", fontsize=9)
    ax3.set_ylabel("Dynamic Acceleration (g)", fontsize=9)
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    # Panel 4: Rotational Order Spectrum
    ax4 = fig.add_subplot(gs[1, 1])
    if ot_res.get("status") == "UNAVAILABLE" or ot_res.get("total_revolutions", 0) < 1:
        ax4.text(0.5, 0.5, "Tachometer Signal Unavailable\n(Static bench record with 0 tach pulses)\nOrder tracking requires active optical pulses.",
                 ha="center", va="center", transform=ax4.transAxes, fontsize=11, color="crimson",
                 bbox=dict(boxstyle="round,pad=0.5", facecolor="#fff0f0", edgecolor="crimson", lw=1.5))
        ax4.set_title("Shaft Order Analysis: RPM UNAVAILABLE", fontsize=11, fontweight='bold')
        ax4.set_xlabel("Rotational Order (Multiple of Shaft Speed 1X)", fontsize=9)
        ax4.set_ylabel("Order Amplitude (g)", fontsize=9)
        ax4.grid(True, alpha=0.3)
    else:
        orders = ot_res["orders"]
        order_mag = ot_res["order_mag"]
        order_mask = (orders <= 8.0)
        ax4.plot(orders[order_mask], order_mag[order_mask], color='#7570b3', lw=1.8, label='Order Spectrum')
        
        m = ot_res["metrics"]
        idx_1x = np.argmin(np.abs(orders - 1.0))
        idx_2x = np.argmin(np.abs(orders - 2.0))
        ax4.scatter([1.0], [order_mag[idx_1x]], color='crimson', s=70, zorder=5)
        ax4.annotate(f"1X Unbalance\n{m['e_1x_pct']:.1f}%", xy=(1.0, order_mag[idx_1x]),
                     xytext=(1.2, order_mag[idx_1x] * 1.2),
                     arrowprops=dict(arrowstyle="->", color='crimson', lw=1.5),
                     fontsize=9, fontweight='bold', color='crimson')
                     
        ax4.scatter([2.0], [order_mag[idx_2x]], color='darkorange', s=60, zorder=5)
        ax4.annotate(f"2X Misalign\n{m['e_2x_pct']:.1f}%", xy=(2.0, order_mag[idx_2x]),
                     xytext=(2.2, order_mag[idx_2x] * 1.2 + 0.005),
                     arrowprops=dict(arrowstyle="->", color='darkorange', lw=1.5),
                     fontsize=9, fontweight='bold', color='darkorange')
                     
        ax4.set_title(f"Shaft Order Analysis (Speed: {m['mean_rpm']:.1f} RPM | {m['primary_order_fault']})", fontsize=11, fontweight='bold')
        ax4.set_xlabel("Rotational Order (Multiple of Shaft Speed 1X)", fontsize=9)
        ax4.set_ylabel("Order Amplitude (g)", fontsize=9)
        ax4.set_xticks(np.arange(0, 9, 1))
        ax4.grid(True, alpha=0.3)
    
    plt.savefig(output_png, dpi=200)
    plt.close()


def run_full_offline_evaluation(wav_path: str, csv_path: str, out_dir: str) -> dict:
    """Executes end-to-end multimodal co-fusion on physical records."""
    print("="*85)
    print(" OFFLINE MULTIMODAL FAULT LOCALIZATION & CO-FUSION ENGINE")
    print("="*85)
    
    # 1. Ingest Audio
    audio_4ch, fs, n_ch = load_wav_multichannel(wav_path)
    print(f"[*] Ingested Audio: {os.path.basename(wav_path)} ({n_ch} Ch @ {fs} Hz, {len(audio_4ch)/fs:.3f} s)")
    
    # 2. Ingest I2S Phase Calibration if exists
    proj_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    calib_candidates = [
        os.path.join(out_dir, "i2s_phase_calibration.json"),
        os.path.join(proj_dir, "stage2_acoustic_localization", "evidence", "i2s_phase_calibration.json"),
        os.path.join(proj_dir, "evidence", "stage2", "i2s_phase_calibration.json")
    ]
    comp_delay_sec = 0.0
    for c_path in calib_candidates:
        if os.path.exists(c_path):
            try:
                with open(c_path, 'r') as f:
                    c_data = json.load(f)
                    if "inter_pair_comp_delay_sec" in c_data:
                        comp_delay_sec = float(c_data["inter_pair_comp_delay_sec"])
                    elif "inter_pair_mean_skew_us" in c_data:
                        comp_delay_sec = float(c_data["inter_pair_mean_skew_us"]) * 1e-6
                    elif "integer_sample_compensation" in c_data:
                        calib_fs = float(c_data.get("sample_rate_hz", fs))
                        comp_delay_sec = float(c_data["integer_sample_compensation"]) / calib_fs
                    print(f"[*] Loaded Phase Calibration: Compensation Delay = {comp_delay_sec*1e6:+.2f} μs from {os.path.basename(c_path)}")
                    break
            except Exception as e:
                print(f"[!] Warning reading phase calibration from {c_path}: {e}")
            
    # 3. Compute SRP-PHAT 2D Spatial Localization
    srp_res = compute_srp_phat_grid(
        audio_4ch,
        fs,
        DEFAULT_ARRAY_GEOMETRY["mic_x"],
        comp_delay_sec=comp_delay_sec
    )
    print(f"[+] Acoustic Angle of Arrival (AoA) : {srp_res['aoa_deg']:+6.2f}° (Primary Physical Observable)")
    print(f"[+] Backprojection Grid Peak Radius : {srp_res['grid_peak_radius_m']:6.2f} m (Unconstrained 1D ULA Ray)")
    print(f"[+] Spatial Peak Prominence         : {srp_res['prominence']:6.2f}")
    
    # Acoustic Kurtosis & Crest Factor
    kurt_ac = float(np.mean([kurtosis(audio_4ch[:, c], fisher=False) for c in range(4)]))
    crest_ac = float(np.mean([np.max(np.abs(audio_4ch[:, c])) / (np.std(audio_4ch[:, c]) + 1e-12) for c in range(4)]))
    print(f"[+] Mean Acoustic Kurtosis          : {kurt_ac:6.2f} (Norm=3.0)")
    print(f"[+] Mean Acoustic Crest Factor      : {crest_ac:6.2f}")
    
    # 4. Compute 3-Axis Dynamic Vibration Severity
    vib_res = evaluate_vibration_record(csv_path)
    print(f"[+] Dynamic Vibration RMS           : {vib_res['rms_dyn_g']:6.3f} g ({vib_res['severity_zone']})")
    print(f"[+] Static Gravity Offset           : {vib_res['dc_gravity_g']:6.3f} g (Expected ~1.0g)")
    
    # 5. Compute Rotational Order Tracking
    df_vib = pd.read_csv(csv_path)
    col_map = {str(c).lower().strip(): c for c in df_vib.columns}
    raw_us = df_vib[col_map.get("timestamp_us", df_vib.columns[0])].values.astype(np.float64)
    t_vib = (raw_us - raw_us[0]) * 1e-6
    vibration = df_vib[col_map.get("accel_z_g", df_vib.columns[6])].values.astype(np.float64)
    
    # Check if tachometer pulses exist in CSV
    rev_times = None
    if "tachometer" in col_map or "tach" in col_map:
        tach_col = col_map.get("tachometer", col_map.get("tach"))
        try:
            rev_times = extract_tachometer_revolutions(t_vib, df_vib[tach_col].values)
        except Exception:
            rev_times = None
    elif "pulse_count" in col_map:
        pulse_diff = np.diff(df_vib[col_map["pulse_count"]].values)
        edge_indices = np.where(pulse_diff > 0)[0]
        if len(edge_indices) >= 2:
            rev_times = t_vib[edge_indices]

    if rev_times is not None and len(rev_times) >= 2:
        ot_res = compute_order_tracking(t_vib, vibration, rev_times, samples_per_rev=256)
        m = ot_res["metrics"]
        print(f"[+] 1X Rotational Order Energy      : {m['e_1x_pct']:6.2f}% ({m['primary_order_fault']})")
        print(f"[+] 2X Rotational Order Energy      : {m['e_2x_pct']:6.2f}%")
    else:
        print("[!] Tachometer Signal Notice        : UNAVAILABLE (Static bench record with 0 tach pulses)")
        ot_res = {
            "status": "UNAVAILABLE",
            "mean_rpm": 0.0,
            "rpm_jitter": 0.0,
            "total_revolutions": 0,
            "orders": np.array([0.0]),
            "order_mag": np.array([0.0]),
            "metrics": {
                "status": "UNAVAILABLE",
                "mean_rpm": 0.0,
                "rpm_jitter": 0.0,
                "rpm_std": 0.0,
                "total_revolutions": 0,
                "e_1x_pct": 0.0,
                "e_2x_pct": 0.0,
                "e_3x_pct": 0.0,
                "e_sub_pct": 0.0,
                "e_harmonics_pct": 0.0,
                "primary_order_fault": "RPM UNAVAILABLE (NO TACHOMETER SIGNAL)"
            }
        }
        m = ot_res["metrics"]
    
    # 6. Composite Health Index Co-Fusion
    # Fuses Vibration RMS + 1X Order Energy + Acoustic Kurtosis + Spatial Prominence
    vib_component = vib_res["vib_score"]
    order_penalty = max(0.0, (m["e_1x_pct"] - 15.0) * 1.2) if m.get("status") == "VALID" else 0.0
    kurt_penalty = max(0.0, (kurt_ac - 3.5) * 8.0)
    composite_health = max(5.0, min(100.0, vib_component - order_penalty - kurt_penalty))
    
    if composite_health >= 85:
        health_verdict = "EXCELLENT / NOMINAL"
    elif composite_health >= 65:
        health_verdict = "PERMISSIBLE (MILD WEAR)"
    elif composite_health >= 40:
        health_verdict = "WARNING (SIGNIFICANT DEGRADATION)"
    else:
        health_verdict = "CRITICAL FAULT (SHUT DOWN REQUIRED)"
        
    print("\n" + "="*85)
    print(f" COMPOSITE MULTIMODAL HEALTH VERDICT: {health_verdict} (Score: {composite_health:.1f}%)")
    print(f" Dominant Acoustic Direction : {srp_res['aoa_deg']:+.1f}° (Backprojection Grid Radius: {srp_res['grid_peak_radius_m']:.2f} m)")
    print(f" Mechanical Diagnosis        : {m['primary_order_fault']} | Severity Index: {vib_res['severity_zone']}")
    print("="*85)
    
    # 7. Generate Master Diagnostic Report Plot
    plot_png = os.path.join(out_dir, "offline_multimodal_diagnostic_report.png")
    generate_multimodal_dashboard_plot(audio_4ch, fs, srp_res, vib_res, ot_res, plot_png)
    print(f"[+] Master Diagnostic Plot Saved    : {plot_png}")
    
    # 8. Export Summary JSON
    summary = {
        "timestamp_eval": "2026-11-07",
        "audio_source": os.path.basename(wav_path),
        "vibration_source": os.path.basename(csv_path),
        "acoustic_spatial": {
            "bearing_angle_deg": srp_res["aoa_deg"],
            "radial_grid_peak_m": srp_res["grid_peak_radius_m"],
            "radial_distance_m": srp_res["grid_peak_radius_m"],
            "prominence": srp_res["prominence"],
            "kurtosis": kurt_ac,
            "crest_factor": crest_ac,
            "phase_comp_delay_sec": comp_delay_sec
        },
        "vibration_dynamics": {
            "rms_g": vib_res["rms_dyn_g"],
            "peak_g": vib_res["peak_dyn_g"],
            "severity_zone": vib_res["severity_zone"],
            "iso_zone": vib_res["severity_zone"],
            "static_gravity_g": vib_res["dc_gravity_g"]
        },
        "rotational_orders": m,
        "multimodal_fusion": {
            "composite_health_score": float(composite_health),
            "health_verdict": health_verdict,
            "primary_mechanical_fault": m["primary_order_fault"],
            "acoustic_direction_deg": srp_res["aoa_deg"]
        }
    }
    
    summary_json = os.path.join(out_dir, "offline_multimodal_summary.json")
    with open(summary_json, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"[+] Summary JSON Saved              : {summary_json}")
    
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Offline Multimodal Fault Evaluation & Acoustic Camera")
    parser.add_argument("--wav", type=str, default="", help="Path to 4-channel WAV file")
    parser.add_argument("--csv", type=str, default="", help="Path to vibration CSV file")
    parser.add_argument("--outdir", type=str, default="", help="Output directory")
    args = parser.parse_args()
    
    proj_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # Locate WAV
    cand_wavs = [
        args.wav,
        os.path.join(proj_dir, "stage2_acoustic_localization", "evidence", "multimodal_4mic_audio.wav"),
        os.path.join(proj_dir, "evidence", "stage2", "multimodal_4mic_audio.wav")
    ]
    target_wav = next((w for w in cand_wavs if w and os.path.exists(w)), "")
    
    # Locate CSV
    cand_csvs = [
        args.csv,
        os.path.join(proj_dir, "stage1_vibration_and_tachometer", "evidence", "multimodal_4mic_vibration.csv"),
        os.path.join(proj_dir, "evidence", "stage2", "multimodal_4mic_vibration.csv")
    ]
    target_csv = next((c for c in cand_csvs if c and os.path.exists(c)), "")
    
    # Output directory
    target_out = args.outdir if args.outdir else os.path.join(proj_dir, "stage4_realtime_fusion_radar", "evidence")
    os.makedirs(target_out, exist_ok=True)
    
    run_full_offline_evaluation(target_wav, target_csv, target_out)
