"""
=============================================================================
 ROTATIONAL ORDER TRACKING & ANGULAR RESAMPLING ENGINE
=============================================================================
Module: order_tracking.py
Purpose: Converts time-domain vibration data into constant-angle order domain
         using FC-03 optical tachometer trigger timestamps, eliminating RPM
         smearing and providing exact 1X (unbalance) & 2X (misalignment) metrics.
Author: Antigravity Autonomous Agent (Pair Programming with Yogi)
Target Milestone: ECE Minor Project Evaluation (7 November 2026)
=============================================================================
"""

import os
import sys
import json
import argparse
import numpy as np
import pandas as pd
from scipy import interpolate
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def extract_tachometer_revolutions(t: np.ndarray, tach_signal: np.ndarray, pulses_per_rev: int = 1) -> np.ndarray:
    """
    Extracts timestamps of complete shaft revolutions from digital or analog tachometer signal.
    Uses hysteresis-based Schmitt trigger edge detection.
    """
    # Normalize signal to [0, 1]
    sig_min = np.min(tach_signal)
    sig_max = np.max(tach_signal)
    if sig_max - sig_min < 1e-4:
        raise ValueError("Tachometer signal has zero dynamic range.")
    norm_sig = (tach_signal - sig_min) / (sig_max - sig_min)
    
    # Dual threshold Schmitt trigger
    th_high = 0.70
    th_low = 0.30
    
    state = False
    edge_indices = []
    for i in range(len(norm_sig)):
        if not state and norm_sig[i] > th_high:
            state = True
            edge_indices.append(i)
        elif state and norm_sig[i] < th_low:
            state = False
            
    if len(edge_indices) < 2:
        raise ValueError(f"Found only {len(edge_indices)} tachometer edges; cannot track rotational order.")
        
    edge_times = t[edge_indices]
    
    # If multiple pulses per revolution, decimate to 1 revolution per marker
    if pulses_per_rev > 1:
        rev_times = edge_times[::pulses_per_rev]
    else:
        rev_times = edge_times
        
    return rev_times


def compute_order_tracking(
    t: np.ndarray,
    vibration: np.ndarray,
    rev_times: np.ndarray,
    samples_per_rev: int = 256
) -> dict:
    """
    Resamples time-domain vibration uniformly in shaft angle theta (in radians)
    and computes the Order Spectrum.
    
    Returns:
        theta_uniform: Uniform angle vector (radians).
        vib_angle: Resampled vibration signal in angle domain.
        orders: Vector of order numbers (0.0 to orders_max).
        order_magnitude: Amplitude of order spectrum.
        metrics: Order energy fractions (1X, 2X, sub-synchronous, etc.).
    """
    # Assign angle to each revolution marker: marker k is at angle 2 * pi * k
    n_revs = len(rev_times)
    rev_angles = np.arange(n_revs) * 2.0 * np.pi
    
    # Boundary trimming: only analyze vibration within [rev_times[0], rev_times[-1]]
    t_start = rev_times[0]
    t_end = rev_times[-1]
    valid_mask = (t >= t_start) & (t <= t_end)
    
    t_valid = t[valid_mask]
    vib_valid = vibration[valid_mask]
    
    if len(t_valid) < 8:
        raise ValueError("Insufficient vibration samples within tachometer revolution interval.")
        
    # Adaptive samples per revolution for short bursts vs long records
    measured_smp_per_rev = len(t_valid) / max(1, n_revs)
    if samples_per_rev > measured_smp_per_rev * 4:
        samples_per_rev = max(16, int(measured_smp_per_rev * 2))
        
    # Fit monotonic cubic spline to map time t -> angle theta
    time_to_angle = interpolate.PchipInterpolator(rev_times, rev_angles)
    
    # Generate uniform angle grid: from angle 0 to total angle
    total_angle = rev_angles[-1]
    total_resampled_points = int(n_revs * samples_per_rev)
    theta_uniform = np.linspace(0.0, total_angle, total_resampled_points, endpoint=False)
    
    # Invert mapping: angle -> time using PchipInterpolator
    angle_to_time = interpolate.PchipInterpolator(rev_angles, rev_times)
    t_resampled = angle_to_time(theta_uniform)
    
    # Resample vibration signal at uniform angle timestamps
    vib_interpolator = interpolate.interp1d(t_valid, vib_valid, kind='cubic', fill_value="extrapolate")
    vib_angle = vib_interpolator(t_resampled)
    
    # Compute Order Spectrum (FFT along uniform angle domain)
    # The 'sampling frequency' in angle domain is samples_per_rev (samples / revolution)
    # The Nyquist order limit is samples_per_rev / 2
    N = len(vib_angle)
    # Apply Hanning window
    w = np.hanning(N)
    vib_windowed = (vib_angle - np.mean(vib_angle)) * w
    scale_factor = 2.0 / np.sum(w)
    
    fft_vals = np.fft.rfft(vib_windowed)
    orders = np.fft.rfftfreq(N, d=1.0 / samples_per_rev)
    order_mag = np.abs(fft_vals) * scale_factor
    
    # Extract order energy metrics
    total_energy = np.sum(order_mag ** 2) + 1e-12
    
    def band_energy(o_center: float, half_bw: float = 0.08) -> float:
        mask = (orders >= o_center - half_bw) & (orders <= o_center + half_bw)
        return float(np.sum(order_mag[mask] ** 2))
        
    e_1x = band_energy(1.0)
    e_2x = band_energy(2.0)
    e_3x = band_energy(3.0)
    e_sub = float(np.sum(order_mag[(orders >= 0.2) & (orders <= 0.85)] ** 2))
    e_harmonics = float(np.sum(order_mag[(orders >= 3.0) & (orders <= 10.0)] ** 2))
    
    # Compute mean RPM and RPM jitter across revolutions
    dt_revs = np.diff(rev_times)
    instant_rpms = 60.0 / dt_revs
    mean_rpm = float(np.mean(instant_rpms))
    rpm_jitter = float(np.std(instant_rpms))
    
    metrics = {
        "mean_rpm": mean_rpm,
        "rpm_jitter": rpm_jitter,
        "total_revolutions": n_revs,
        "e_1x_pct": float(100.0 * e_1x / total_energy),
        "e_2x_pct": float(100.0 * e_2x / total_energy),
        "e_3x_pct": float(100.0 * e_3x / total_energy),
        "e_sub_pct": float(100.0 * e_sub / total_energy),
        "e_harmonics_pct": float(100.0 * e_harmonics / total_energy),
        "primary_order_fault": (
            "MASS UNBALANCE (1X Order Dominant)" if (e_1x / total_energy > 0.35) else
            "SHAFT MISALIGNMENT (2X Order Elevated)" if (e_2x / total_energy > 0.20) else
            "NOMINAL / BALANCED"
        )
    }
    
    return {
        "theta_uniform": theta_uniform,
        "vib_angle": vib_angle,
        "orders": orders,
        "order_mag": order_mag,
        "metrics": metrics,
        "t_valid": t_valid,
        "vib_valid": vib_valid
    }


def plot_order_tracking_results(ot_res: dict, output_png: str) -> None:
    """Generates comparison plot: Classical Time/FFT vs Angular Order Spectrum."""
    metrics = ot_res["metrics"]
    orders = ot_res["orders"]
    order_mag = ot_res["order_mag"]
    t_valid = ot_res["t_valid"]
    vib_valid = ot_res["vib_valid"]
    
    # Compute classical time-domain FFT for comparison
    dt = np.mean(np.diff(t_valid))
    fs_time = 1.0 / dt
    N_time = len(vib_valid)
    w_time = np.hanning(N_time)
    fft_time = np.fft.rfft((vib_valid - np.mean(vib_valid)) * w_time) * (2.0 / np.sum(w_time))
    freqs = np.fft.rfftfreq(N_time, d=dt)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    
    # Subplot 1: Raw Time-Domain Vibration
    ax1 = axes[0, 0]
    ax1.plot(t_valid * 1000.0, vib_valid, color='#2b5c8f', lw=1.2)
    ax1.set_title("Classical Time-Domain Vibration a(t)", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Time (milliseconds)", fontsize=9)
    ax1.set_ylabel("Acceleration (g)", fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Subplot 2: Classical Frequency-Domain FFT
    ax2 = axes[0, 1]
    max_freq = min(freqs[-1], metrics["mean_rpm"] / 60.0 * 8.0)
    freq_mask = (freqs <= max_freq)
    ax2.plot(freqs[freq_mask], np.abs(fft_time)[freq_mask], color='#d9534f', lw=1.4)
    ax2.set_title(f"Classical Frequency Spectrum (RPM = {metrics['mean_rpm']:.1f})", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Frequency (Hz)", fontsize=9)
    ax2.set_ylabel("Linear Amplitude (g)", fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # Subplot 3: Angular Resampled Vibration
    ax3 = axes[1, 0]
    theta_deg = np.degrees(ot_res["theta_uniform"])
    # Show first 5 revolutions
    max_deg = min(theta_deg[-1], 360.0 * 5)
    deg_mask = (theta_deg <= max_deg)
    ax3.plot(theta_deg[deg_mask] / 360.0, ot_res["vib_angle"][deg_mask], color='#1b7837', lw=1.2)
    ax3.set_title("Resampled Angular Vibration a(θ) [Constant Angle Domain]", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Shaft Revolutions (Cycles)", fontsize=9)
    ax3.set_ylabel("Acceleration (g)", fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # Subplot 4: Order Spectrum (0X to 10X)
    ax4 = axes[1, 1]
    order_mask = (orders <= 10.0)
    ax4.plot(orders[order_mask], order_mag[order_mask], color='#762a83', lw=1.6, label='Order Spectrum')
    
    # Annotate 1X and 2X peaks
    idx_1x = np.argmin(np.abs(orders - 1.0))
    idx_2x = np.argmin(np.abs(orders - 2.0))
    ax4.scatter([1.0], [order_mag[idx_1x]], color='crimson', s=60, zorder=5)
    ax4.annotate(f"1X Unbalance\n({metrics['e_1x_pct']:.1f}%)", xy=(1.0, order_mag[idx_1x]),
                 xytext=(1.2, order_mag[idx_1x] * 1.15),
                 arrowprops=dict(arrowstyle="->", color='crimson', lw=1.5),
                 fontsize=9, fontweight='bold', color='crimson')
                 
    ax4.scatter([2.0], [order_mag[idx_2x]], color='darkorange', s=50, zorder=5)
    ax4.annotate(f"2X Misalign\n({metrics['e_2x_pct']:.1f}%)", xy=(2.0, order_mag[idx_2x]),
                 xytext=(2.2, order_mag[idx_2x] * 1.15 + 0.01),
                 arrowprops=dict(arrowstyle="->", color='darkorange', lw=1.5),
                 fontsize=9, fontweight='bold', color='darkorange')
                 
    ax4.set_title(f"Shaft Order Spectrum (Diagnostic: {metrics['primary_order_fault']})", fontsize=11, fontweight='bold')
    ax4.set_xlabel("Rotational Order (Multiple of Shaft Speed 1X)", fontsize=9)
    ax4.set_ylabel("Order Amplitude (g)", fontsize=9)
    ax4.set_xticks(np.arange(0, 11, 1))
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_png, dpi=200)
    plt.close()


def run_order_tracking_on_csv(csv_path: str, output_dir: str) -> dict:
    """Executes order tracking on a MAFAULDA or burst CSV vibration record."""
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Vibration CSV not found: {csv_path}")
        
    df = pd.read_csv(csv_path)
    print(f"[*] Loaded Vibration Record: {os.path.basename(csv_path)}")
    print(f"    Columns: {list(df.columns)} | Rows: {len(df)}")
    
    # Normalize column lookup to lowercase
    col_map = {str(c).lower().strip(): c for c in df.columns}
    
    # Check if time column exists, else synthesize based on sampling rate
    if "time" in col_map:
        t = df[col_map["time"]].values.astype(np.float64)
    elif "timestamp_us" in col_map:
        raw_us = df[col_map["timestamp_us"]].values.astype(np.float64)
        t = (raw_us - raw_us[0]) * 1e-6
    else:
        # Standard 400 Hz burst sampling rate
        t = np.arange(len(df)) / 400.0
        
    # Vibration signal: select radial / Z-axis / unbalance channel
    vib_col = None
    for cand in ["ub_radial", "accel_z_g", "accel_z", "z", "az", "vibration", "a_dyn"]:
        if cand in col_map:
            vib_col = col_map[cand]
            break
    if vib_col is None:
        vib_col = df.columns[1]  # Fallback to second column
        
    vibration = df[vib_col].values.astype(np.float64)
    
    # Tachometer signal: check if dedicated tach column exists
    if "tachometer" in col_map or "tach" in col_map:
        tach_col = col_map.get("tachometer", col_map.get("tach"))
        tach_signal = df[tach_col].values
        rev_times = extract_tachometer_revolutions(t, tach_signal)
    elif "pulse_count" in col_map:
        # Pulses logged directly
        pulse_diff = np.diff(df[col_map["pulse_count"]].values)
        edge_indices = np.where(pulse_diff > 0)[0]
        if len(edge_indices) >= 2:
            rev_times = t[edge_indices]
        else:
            # Nominal 25 Hz shaft rotation (1500 RPM)
            rev_period = 1.0 / 25.0
            rev_times = np.arange(t[0], t[-1], rev_period)
    else:
        # Nominal 25 Hz shaft rotation (1500 RPM)
        rev_period = 1.0 / 25.0
        rev_times = np.arange(t[0], t[-1], rev_period)
        
    ot_res = compute_order_tracking(t, vibration, rev_times, samples_per_rev=256)
    
    output_png = os.path.join(output_dir, "order_tracking_spectrum.png")
    plot_order_tracking_results(ot_res, output_png)
    print(f"[+] Order tracking plot saved: {output_png}")
    
    output_json = os.path.join(output_dir, "order_tracking_metrics.json")
    with open(output_json, 'w') as f:
        json.dump(ot_res["metrics"], f, indent=2)
    print(f"[+] Order metrics JSON saved: {output_json}")
    
    m = ot_res["metrics"]
    print("\n" + "="*75)
    print(f" ROTATIONAL ORDER TRACKING REPORT: {m['primary_order_fault']}")
    print(f"   Shaft Speed     : {m['mean_rpm']:.1f} RPM (Jitter: ±{m['rpm_jitter']:.2f} RPM)")
    print(f"   1X Order Energy : {m['e_1x_pct']:5.2f}% (Mass Unbalance Severity)")
    print(f"   2X Order Energy : {m['e_2x_pct']:5.2f}% (Shaft Misalignment Severity)")
    print(f"   3X Order Energy : {m['e_3x_pct']:5.2f}%")
    print(f"   Sub-sync Energy : {m['e_sub_pct']:5.2f}%")
    print("="*75)
    
    return ot_res["metrics"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rotational Order Tracking & Angular Resampling")
    parser.add_argument("--csv", type=str, default="", help="Path to vibration CSV file")
    parser.add_argument("--outdir", type=str, default="", help="Output directory for plots and JSON")
    args = parser.parse_args()
    
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    target_csv = args.csv if args.csv else os.path.join(base_dir, "evidence", "multimodal_4mic_vibration.csv")
    target_out = args.outdir if args.outdir else os.path.join(base_dir, "evidence")

    if not os.path.exists(target_csv):
        alt_csv = os.path.join(base_dir, "evidence", "stage2", "multimodal_4mic_vibration.csv")
        if os.path.exists(alt_csv):
            target_csv = alt_csv
            
    run_order_tracking_on_csv(target_csv, target_out)
