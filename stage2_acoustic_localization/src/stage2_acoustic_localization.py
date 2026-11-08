"""
Stage 2 Acoustic Source Localization Simulation & Verification Suite
====================================================================
ECE Minor Project: Spatial Multimodal Edge-AI for Rotating Machinery
Date: 7 November 2026

Implements Stages 2A, 2B, 2C, and 2D from NEXT_STEPS.md:
  - Stage 2A: Known 4-microphone array geometry & geometric delay modeling.
  - Stage 2B: Pairwise GCC-PHAT TDOA estimation vs analytical ground truth.
  - Stage 2C: 2D SRP-PHAT grid search across 25 off-grid test positions.
  - Stage 2D: Robustness benchmarks across SNR (Clean, 20dB, 10dB, 0dB) & Multipath.
  - Generates publication-ready diagnostic plots and summary CSVs in evidence/stage2/.
"""

import os
import sys
import math
import time
import zipfile
import numpy as np
import matplotlib
matplotlib.use("Agg") # Headless generation
import matplotlib.pyplot as plt
from scipy import signal
from numpy.fft import rfft, irfft

# Output directories
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVIDENCE_DIR = os.path.join(BASE_DIR, "evidence", "stage2")
os.makedirs(EVIDENCE_DIR, exist_ok=True)

SPEED_OF_SOUND = 343.0  # m/s at 20 deg C dry air
FS = 16000              # Hz (High fidelity acoustic simulation)
T_LEN = 0.10            # 100 ms audio frames
N_SAMPLES = int(FS * T_LEN)

# =============================================================================
# 1. ARRAY GEOMETRY DEFINITIONS (Stage 2A)
# =============================================================================
class ArrayGeometry:
    """Manages physical microphone coordinates and precomputes spatial delay manifolds."""
    def __init__(self, name: str, mic_coords: np.ndarray, description: str = ""):
        self.name = name
        self.mic_coords = np.asarray(mic_coords, dtype=np.float64)
        self.num_mics = len(mic_coords)
        self.description = description
        self.pairs = [
            (i, j) for i in range(self.num_mics) for j in range(i + 1, self.num_mics)
        ]
        self.num_pairs = len(self.pairs)

# Breadboard 4-Mic Uniform Linear Array (d = 20.32 mm / 8 breadboard slots)
PITCH_M = 0.02032
ULA_4MIC = ArrayGeometry(
    "Breadboard_4Mic_ULA",
    np.array([
        [-0.03048, 0.0],  # Mic 1 (Row 20, Pair A Left)
        [-0.01016, 0.0],  # Mic 2 (Row 28, Pair A Right)
        [+0.01016, 0.0],  # Mic 3 (Row 36, Pair B Left)
        [+0.03048, 0.0],  # Mic 4 (Row 44, Pair B Right)
    ]),
    "4-Mic ULA Spaced at 20.32mm, Total Aperture = 60.96mm"
)

# Planar 4-Mic 2D Square Array (for 2D x-y localization demonstration)
SQUARE_4MIC = ArrayGeometry(
    "Planar_4Mic_Square",
    np.array([
        [-0.025, -0.025],  # Mic 1 (Bottom Left)
        [-0.025, +0.025],  # Mic 2 (Top Left)
        [+0.025, +0.025],  # Mic 3 (Top Right)
        [+0.025, -0.025],  # Mic 4 (Bottom Right)
    ]),
    "4-Mic Square Array 50mm x 50mm for 2D Coordinate Estimation"
)

# =============================================================================
# 2. ACOUSTIC EMISSION & MULTIPATH SIMULATION (Stage 2A & 2D)
# =============================================================================
def generate_source_signal(n_samples: int, fs: float, motor_rpm: float = 2400.0) -> np.ndarray:
    """Generates broadband motor acoustic emission (shaft harmonics + pink noise + brush arcing)."""
    t = np.arange(n_samples) / fs
    f0 = motor_rpm / 60.0  # Shaft rotational fundamental (e.g. 40 Hz)
    
    # Fundamental and mechanical harmonics
    sig = 0.35 * np.sin(2 * np.pi * f0 * t)
    sig += 0.25 * np.sin(2 * np.pi * (2 * f0) * t + 0.4)
    sig += 0.20 * np.sin(2 * np.pi * (3 * f0) * t + 1.1)
    sig += 0.15 * np.sin(2 * np.pi * (7 * f0) * t + 2.3)
    sig += 0.30 * np.sin(2 * np.pi * 1250.0 * t)  # Commutator slot pass frequency
    
    # Pink noise / broadband aerodynamic component
    white = np.random.randn(n_samples)
    b, a = signal.butter(1, 0.15)
    pink = signal.lfilter(b, a, white)
    sig += 0.25 * (pink / np.std(pink))
    
    # Occasional brush spark transients (high kurtosis)
    spark_locs = np.random.choice(n_samples, size=max(1, int(n_samples * 0.005)), replace=False)
    sig[spark_locs] += np.random.uniform(1.5, 3.0, size=len(spark_locs)) * np.random.choice([-1, 1], size=len(spark_locs))
    
    # Normalize to unit variance
    sig = (sig - np.mean(sig)) / (np.std(sig) + 1e-12)
    return sig

def fractional_delay(sig: np.ndarray, delay_sec: float, fs: float) -> np.ndarray:
    """
    Applies exact bandlimited fractional sample delay in frequency domain with
    zero-padding to eliminate circular wraparound edge effects.
    """
    n_orig = len(sig)
    delay_samples = int(np.ceil(abs(delay_sec) * fs)) + 64
    pad_len = max(128, delay_samples * 2)

    padded_sig = np.pad(sig, (pad_len, pad_len), mode="constant")
    n_padded = len(padded_sig)

    freqs = np.fft.rfftfreq(n_padded, d=1.0 / fs)
    phase_shift = np.exp(-2j * np.pi * freqs * delay_sec)
    delayed_padded = irfft(rfft(padded_sig) * phase_shift, n=n_padded)

    return delayed_padded[pad_len : pad_len + n_orig]

def simulate_array_signals(
    src_pos: np.ndarray,
    geometry: ArrayGeometry,
    fs: float,
    snr_db: float = float("inf"),
    add_multipath: bool = False
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulates multi-channel microphone signals from an arbitrary 2D source position.
    Returns:
        signals: (n_samples, num_mics) array
        true_delays: (num_pairs,) exact analytical TDOA delays
    """
    n_samples = int(FS * T_LEN)
    src_sig = generate_source_signal(n_samples, fs)
    num_mics = geometry.num_mics
    
    # 1. Direct path propagation
    dists = np.linalg.norm(geometry.mic_coords - src_pos, axis=1)
    delays = dists / SPEED_OF_SOUND
    
    # Compute true pairwise TDOA: tau_ij = delay_i - delay_j = (dist_i - dist_j) / c
    # Convention: positive tau_ij means sound arrived at Mic j earlier than Mic i (source on right)
    true_pair_tdoa = np.zeros(geometry.num_pairs)
    for p_idx, (i, j) in enumerate(geometry.pairs):
        true_pair_tdoa[p_idx] = delays[i] - delays[j]
        
    # Relative delay referenced to closest microphone
    min_delay = np.min(delays)
    rel_delays = delays - min_delay
    
    rx_signals = np.zeros((n_samples, num_mics), dtype=np.float64)
    for m in range(num_mics):
        # 1/r spherical spreading attenuation
        att = 1.0 / max(0.1, dists[m])
        rx_signals[:, m] = att * fractional_delay(src_sig, rel_delays[m], fs)
        
    # 2. Multipath / Wall reflections (Stage 2D)
    if add_multipath:
        # Simulate two virtual image sources representing reflective sidewalls
        wall_positions = [
            np.array([-src_pos[0] - 1.2, src_pos[1]]), # Left wall reflection
            np.array([src_pos[0] + 1.2, src_pos[1]])   # Right wall reflection
        ]
        wall_coeff = 0.35 # Absorption reflection coefficient
        for w_pos in wall_positions:
            w_dists = np.linalg.norm(geometry.mic_coords - w_pos, axis=1)
            w_delays = w_dists / SPEED_OF_SOUND - min_delay
            for m in range(num_mics):
                w_att = wall_coeff / max(0.1, w_dists[m])
                rx_signals[:, m] += w_att * fractional_delay(src_sig, w_delays[m], fs)
                
    # 3. Additive White Gaussian Noise calibrated to SNR target
    if not math.isinf(snr_db):
        for m in range(num_mics):
            sig_power = np.mean(rx_signals[:, m] ** 2)
            noise_power = sig_power / (10.0 ** (snr_db / 10.0))
            noise = np.random.normal(0.0, np.sqrt(noise_power), n_samples)
            rx_signals[:, m] += noise
            
    return rx_signals, true_pair_tdoa

# =============================================================================
# 3. PAIRWISE SUPER-RESOLUTION GCC-PHAT ENGINE (Stage 2B)
# =============================================================================
def gcc_phat(
    sig_i: np.ndarray,
    sig_j: np.ndarray,
    fs: float,
    d_ij: float,
    interp_factor: int = 16
) -> Tuple[float, float, np.ndarray, np.ndarray]:
    """
    Computes Sinc-interpolated Generalized Cross-Correlation with Phase Transform.
    Returns:
        est_tau: Estimated delay in seconds
        prominence: Peak-to-floor SNR prominence
        corr: Fine cross-correlation curve
        lags: Lag time vector in seconds
    """
    n = len(sig_i)
    w = signal.windows.tukey(n, alpha=0.20)
    si = (sig_i - np.mean(sig_i)) * w
    sj = (sig_j - np.mean(sig_j)) * w
    
    Si = rfft(si)
    Sj = rfft(sj)
    cross = Si * np.conj(Sj)
    mag = np.abs(cross)
    
    # Bandpass mask (80 Hz to 7500 Hz)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    mask = (freqs >= 80.0) & (freqs <= min(7500.0, fs / 2.0 - 50.0))
    cross[~mask] = 0.0
    
    denom = np.maximum(mag, 1e-4 * np.max(mag))
    cross_phat = np.where(denom > 1e-15, cross / denom, 0.0)
    
    # Sinc interpolation via zero-padding in frequency domain
    n_interp = n * interp_factor
    n_freq = n_interp // 2 + 1
    cross_pad = np.zeros(n_freq, dtype=complex)
    cross_pad[:len(cross_phat)] = cross_phat
    
    corr = np.fft.fftshift(irfft(cross_pad, n=n_interp))
    dt_fine = 1.0 / (fs * interp_factor)
    lags = np.arange(-n_interp // 2, n_interp // 2) * dt_fine
    
    # Physical windowing based on microphone spacing limit
    tau_limit = (d_ij / SPEED_OF_SOUND) * 1.05
    phys_mask = np.abs(lags) <= tau_limit
    c_phys = corr[phys_mask]
    l_phys = lags[phys_mask]
    
    if len(c_phys) < 3:
        return 0.0, 0.0, corr, lags
        
    k = int(np.argmax(c_phys))
    peak_val = float(c_phys[k])
    mean_val = float(np.mean(c_phys))
    std_val = float(np.std(c_phys)) + 1e-12
    prominence = max(0.0, (peak_val - mean_val) / std_val)
    
    # Parabolic sub-sample refinement
    if 0 < k < len(c_phys) - 1:
        y0, y1, y2 = float(c_phys[k-1]), float(c_phys[k]), float(c_phys[k+1])
        denom_p = 2.0 * (y0 - 2.0 * y1 + y2)
        if abs(denom_p) > 1e-12:
            delta = max(-0.5, min(0.5, (y0 - y2) / denom_p))
            est_tau = float(l_phys[k] + delta * dt_fine)
        else:
            est_tau = float(l_phys[k])
    else:
        est_tau = float(l_phys[k])
        
    return est_tau, prominence, c_phys, l_phys

# =============================================================================
# 4. 2D SRP-PHAT LOCALIZATION & ACOUSTIC HEATMAP ENGINE (Stage 2C)
# =============================================================================
class SRPLocalizer:
    """Performs spatial backprojection and grid-search localization."""
    def __init__(self, geometry: ArrayGeometry, grid_x: np.ndarray, grid_y: np.ndarray):
        self.geometry = geometry
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.GX, self.GY = np.meshgrid(grid_x, grid_y)
        self.num_points = self.GX.size
        
        # Precompute theoretical delays for every grid cell to each pair
        self.pair_delays = []
        for (i, j) in self.geometry.pairs:
            m_i = self.geometry.mic_coords[i]
            m_j = self.geometry.mic_coords[j]
            d_i = np.sqrt((self.GX - m_i[0])**2 + (self.GY - m_i[1])**2)
            d_j = np.sqrt((self.GX - m_j[0])**2 + (self.GY - m_j[1])**2)
            tau_ij = (d_i - d_j) / SPEED_OF_SOUND
            self.pair_delays.append(tau_ij)

    def locate_2d(self, signals: np.ndarray, fs: float) -> Tuple[np.ndarray, np.ndarray, float, float]:
        """
        Computes 2D Steered Response Power field across spatial grid.
        Returns:
            est_pos: [x, y] in meters
            heatmap: (len(grid_y), len(grid_x)) power distribution
            est_bearing_deg: Estimated angle in degrees
            confidence: Prominence metric
        """
        heatmap = np.zeros_like(self.GX, dtype=np.float64)
        pair_proms = []
        
        for p_idx, (i, j) in enumerate(self.geometry.pairs):
            sig_i = signals[:, i]
            sig_j = signals[:, j]
            d_ij = np.linalg.norm(self.geometry.mic_coords[j] - self.geometry.mic_coords[i])
            _, prom, c_phys, l_phys = gcc_phat(sig_i, sig_j, fs, d_ij)
            pair_proms.append(prom)
            
            if len(c_phys) >= 3 and prom >= 0.20:
                tau_grid = self.pair_delays[p_idx].ravel()
                pair_resp = np.interp(tau_grid, l_phys, c_phys, left=0.0, right=0.0).reshape(self.GX.shape)
                weight = math.sqrt(d_ij / 0.02032) * min(prom, 3.0)
                heatmap += weight * np.maximum(pair_resp, 0.0)
                
        if np.max(heatmap) > 1e-12:
            norm_heat = heatmap / np.max(heatmap)
            k_peak = np.unravel_index(np.argmax(norm_heat), norm_heat.shape)
            est_x = float(self.grid_x[k_peak[1]])
            est_y = float(self.grid_y[k_peak[0]])
            est_bearing_deg = float(np.degrees(np.arctan2(est_x, est_y)))
            mean_h = float(np.mean(norm_heat))
            std_h = float(np.std(norm_heat)) + 1e-12
            confidence = max(0.0, (1.0 - mean_h) / std_h)
        else:
            norm_heat = np.zeros_like(heatmap)
            est_x, est_y = 0.0, 1.0
            est_bearing_deg = 0.0
            confidence = 0.0
            
        return np.array([est_x, est_y]), norm_heat, est_bearing_deg, confidence

# =============================================================================
# 5. EXECUTE COMPREHENSIVE BENCHMARK (Stage 2C & 2D)
# =============================================================================
def run_stage2_evaluation():
    print("=" * 80)
    print(" EXECUTING STAGE 2 ACOUSTIC LOCALIZATION SIMULATION & BENCHMARK SUITE")
    print("=" * 80)
    
    # Spatial search grid: x in [-1.5m, +1.5m], y in [0.4m, 2.5m]
    grid_x = np.linspace(-1.5, 1.5, 121)
    grid_y = np.linspace(0.4, 2.5, 85)
    localizer = SRPLocalizer(ULA_4MIC, grid_x, grid_y)
    
    # -------------------------------------------------------------------------
    # PART 1: TDOA & GCC-PHAT Pairwise Accuracy Verification (Stage 2B)
    # -------------------------------------------------------------------------
    print("\n[+] Stage 2B: Verifying Pairwise GCC-PHAT against Analytical Ground Truth...")
    test_src = np.array([0.75, 1.25]) # 75cm right, 1.25m front (Angle = +30.96 deg)
    rx_sig, true_tdoa = simulate_array_signals(test_src, ULA_4MIC, FS, snr_db=30.0)
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), facecolor="#0a0e17")
    fig.suptitle(f"Stage 2B: GCC-PHAT Cross-Correlation vs Analytical Ground Truth (Source at [{test_src[0]:.2f}m, {test_src[1]:.2f}m])",
                 color="#00ffcc", fontsize=12, weight="bold")
    
    tdoa_verification_results = []
    for p_idx, (i, j) in enumerate(ULA_4MIC.pairs):
        d_ij = np.linalg.norm(ULA_4MIC.mic_coords[j] - ULA_4MIC.mic_coords[i])
        est_tau, prom, c_phys, l_phys = gcc_phat(rx_sig[:, i], rx_sig[:, j], FS, d_ij)
        tau_true = true_tdoa[p_idx]
        err_us = (est_tau - tau_true) * 1e6
        err_samples = (est_tau - tau_true) * FS
        
        tdoa_verification_results.append({
            "pair": f"M{i+1}-M{j+1}",
            "spacing_mm": d_ij * 1000.0,
            "true_tau_us": tau_true * 1e6,
            "est_tau_us": est_tau * 1e6,
            "err_us": err_us,
            "err_samples": err_samples,
            "prominence": prom
        })
        
        ax = axes[p_idx // 2, p_idx % 2]
        ax.set_facecolor("#101726")
        ax.plot(l_phys * 1e6, c_phys, color="#33ccff", linewidth=1.5, label="GCC-PHAT")
        ax.axvline(tau_true * 1e6, color="#ff0055", linestyle="--", linewidth=1.8, label=f"True: {tau_true*1e6:+.1f} µs")
        ax.axvline(est_tau * 1e6, color="#00ffcc", linestyle=":", linewidth=1.8, label=f"Est:  {est_tau*1e6:+.1f} µs")
        ax.set_title(f"Pair M{i+1}-M{j+1} (d = {d_ij*1000:.1f} mm) | Error: {err_us:+.2f} µs ({err_samples:+.3f} smp)",
                     color="#ffffff", fontsize=9.5)
        ax.set_xlabel("Time Lag (µs)", color="#b8c2d1", fontsize=8)
        ax.set_ylabel("Normalized PHAT", color="#b8c2d1", fontsize=8)
        ax.tick_params(colors="#b8c2d1", labelsize=7.5)
        ax.grid(alpha=0.15, color="#34496d")
        ax.legend(facecolor="#0a0e17", edgecolor="#2a3b5c", fontsize=7.5, loc="upper right")
        
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    fig_path_tdoa = os.path.join(EVIDENCE_DIR, "stage2_tdoa_gcc_phat_verification.png")
    fig.savefig(fig_path_tdoa, dpi=200)
    plt.close(fig)
    print(f"    [SAVED] {fig_path_tdoa}")
    
    # -------------------------------------------------------------------------
    # PART 2: 25-Point Off-Grid Benchmark Suite across 2D Room (Stage 2C)
    # -------------------------------------------------------------------------
    print("\n[+] Stage 2C: Evaluating SRP-PHAT Localization across 25 Off-Grid Test Positions...")
    np.random.seed(42) # Reproducible benchmark
    
    # Generate 25 off-grid test positions covering multiple ranges and angles
    angles_target = np.linspace(-65.0, 65.0, 25)
    ranges_target = np.random.uniform(0.65, 2.10, size=25)
    
    benchmark_records = []
    rep_heatmaps_dict = {}
    
    for idx, (ang_deg, rng_m) in enumerate(zip(angles_target, ranges_target)):
        ang_rad = np.radians(ang_deg)
        true_x = float(rng_m * np.sin(ang_rad))
        true_y = float(rng_m * np.cos(ang_rad))
        true_pos = np.array([true_x, true_y])
        
        # Simulate signals with moderate realistic noise (20 dB SNR)
        rx_signals, _ = simulate_array_signals(true_pos, ULA_4MIC, FS, snr_db=20.0)
        
        # Locate source via SRP-PHAT
        est_pos, heatmap, est_bearing_deg, conf = localizer.locate_2d(rx_signals, FS)
        
        # Compute errors
        ang_error = abs(est_bearing_deg - ang_deg)
        dist_error = float(np.linalg.norm(est_pos - true_pos))
        
        benchmark_records.append({
            "test_id": idx + 1,
            "true_x_m": true_x,
            "true_y_m": true_y,
            "true_range_m": rng_m,
            "true_angle_deg": ang_deg,
            "est_x_m": est_pos[0],
            "est_y_m": est_pos[1],
            "est_angle_deg": est_bearing_deg,
            "angle_error_deg": ang_error,
            "dist_error_m": dist_error,
            "confidence": conf
        })
        
        # Capture 4 key representative cases for multi-panel heatmap figure
        if idx in [2, 12, 22, 17]:
            rep_heatmaps_dict[idx] = (true_pos, est_pos, heatmap, ang_deg, est_bearing_deg)

    # Order explicitly matching titles: Case A (idx 2), Case B (idx 12), Case C (idx 22), Case D (idx 17)
    representative_heatmaps = [rep_heatmaps_dict[k] for k in [2, 12, 22, 17] if k in rep_heatmaps_dict]

    # Save benchmark CSV
    csv_path_bench = os.path.join(EVIDENCE_DIR, "stage2_localization_benchmarks.csv")
    with open(csv_path_bench, "w") as f:
        f.write("test_id,true_x_m,true_y_m,true_range_m,true_angle_deg,est_x_m,est_y_m,est_angle_deg,angle_error_deg,dist_error_m,confidence\n")
        for r in benchmark_records:
            f.write(f"{r['test_id']},{r['true_x_m']:.4f},{r['true_y_m']:.4f},{r['true_range_m']:.4f},{r['true_angle_deg']:.2f},"
                    f"{r['est_x_m']:.4f},{r['est_y_m']:.4f},{r['est_angle_deg']:.2f},{r['angle_error_deg']:.2f},{r['dist_error_m']:.4f},{r['confidence']:.2f}\n")
    print(f"    [SAVED] {csv_path_bench}")
    
    # Compute aggregate statistics
    ang_errors = [r["angle_error_deg"] for r in benchmark_records]
    dist_errors = [r["dist_error_m"] for r in benchmark_records]
    
    med_ang_err = float(np.median(ang_errors))
    mean_ang_err = float(np.mean(ang_errors))
    p90_ang_err = float(np.percentile(ang_errors, 90))
    med_dist_err = float(np.median(dist_errors))
    mean_dist_err = float(np.mean(dist_errors))
    p90_dist_err = float(np.percentile(dist_errors, 90))
    success_rate = float(np.mean(np.array(ang_errors) <= 5.0)) * 100.0
    
    print(f"    --> 25-Point Angular Error : Median={med_ang_err:.2f}°, Mean={mean_ang_err:.2f}°, 90th%={p90_ang_err:.2f}°")
    print(f"    --> 25-Point Radial Error  : Median={med_dist_err*100:.1f} cm, Mean={mean_dist_err*100:.1f} cm, 90th%={p90_dist_err*100:.1f} cm")
    print(f"    --> Angular Accuracy (<5°) : {success_rate:.1f}% Pass Rate")

    # Plot representative 4-quadrant thermal heatmaps
    fig, axes = plt.subplots(2, 2, figsize=(14, 11), facecolor="#0a0e17")
    fig.suptitle("Stage 2C: Representative 2D SRP-PHAT Acoustic Camera Heatmaps", color="#00ffcc", fontsize=13, weight="bold")
    
    titles = [
        "Case A: Left Flank Source (Target: -54.2°)",
        "Case B: Center Bore Source (Target: +0.0°)",
        "Case C: Right Flank Source (Target: +54.2°)",
        "Case D: Intermediate Off-Axis Source (Target: +27.1°)"
    ]
    
    for ax_idx, (t_pos, e_pos, h_map, true_ang, est_ang) in enumerate(representative_heatmaps):
        ax = axes[ax_idx // 2, ax_idx % 2]
        ax.set_facecolor("#101726")
        im = ax.imshow(h_map, extent=[-1.5, 1.5, 0.4, 2.5], origin="lower", cmap="inferno", aspect="auto")
        ax.plot(t_pos[0], t_pos[1], "wo", markersize=9, markeredgecolor="#00ffff", markeredgewidth=2, label="True Source")
        ax.plot(e_pos[0], e_pos[1], "rx", markersize=10, markeredgewidth=2.5, label=f"SRP Peak ({est_ang:+.1f}°)")
        
        # Plot array position at origin
        ax.plot(ULA_4MIC.mic_coords[:, 0], ULA_4MIC.mic_coords[:, 1], "c^", markersize=7, label="4-Mic Array")
        
        ax.set_title(f"{titles[ax_idx]} | Error: {abs(est_ang - true_ang):.1f}°", color="#ffffff", fontsize=10)
        ax.set_xlabel("X (meters)", color="#b8c2d1", fontsize=9)
        ax.set_ylabel("Y (meters)", color="#b8c2d1", fontsize=9)
        ax.tick_params(colors="#b8c2d1", labelsize=8)
        ax.legend(facecolor="#0a0e17", edgecolor="#2a3b5c", fontsize=8, loc="upper right")
        cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("SRP Energy", color="#b8c2d1", fontsize=8)
        cb.ax.tick_params(colors="#b8c2d1", labelsize=7.5)
        
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    fig_path_heat = os.path.join(EVIDENCE_DIR, "stage2_srp_heatmaps_sample.png")
    fig.savefig(fig_path_heat, dpi=200)
    plt.close(fig)
    print(f"    [SAVED] {fig_path_heat}")

    # -------------------------------------------------------------------------
    # PART 3: SNR Robustness & Multipath Sweeps (Stage 2D)
    # -------------------------------------------------------------------------
    print("\n[+] Stage 2D: Benchmarking Noise Robustness (Clean, 20dB, 10dB, 0dB) & Reverberation...")
    snr_levels = [float("inf"), 20.0, 10.0, 0.0]
    snr_labels = ["Clean (Inf dB)", "20 dB SNR", "10 dB SNR", "0 dB SNR", "Multipath/Reverb"]
    
    robustness_results = {}
    
    for snr in snr_levels:
        label = f"{snr:.0f} dB SNR" if not math.isinf(snr) else "Clean (Inf dB)"
        errors = []
        for (ang_deg, rng_m) in zip(angles_target, ranges_target):
            rad = np.radians(ang_deg)
            pos = np.array([rng_m * np.sin(rad), rng_m * np.cos(rad)])
            sig, _ = simulate_array_signals(pos, ULA_4MIC, FS, snr_db=snr, add_multipath=False)
            _, _, est_ang, _ = localizer.locate_2d(sig, FS)
            errors.append(abs(est_ang - ang_deg))
        robustness_results[label] = errors
        
    # Multipath evaluation (20 dB SNR + 2 wall reflections)
    reverb_errors = []
    for (ang_deg, rng_m) in zip(angles_target, ranges_target):
        rad = np.radians(ang_deg)
        pos = np.array([rng_m * np.sin(rad), rng_m * np.cos(rad)])
        sig, _ = simulate_array_signals(pos, ULA_4MIC, FS, snr_db=20.0, add_multipath=True)
        _, _, est_ang, _ = localizer.locate_2d(sig, FS)
        reverb_errors.append(abs(est_ang - ang_deg))
    robustness_results["Multipath/Reverb"] = reverb_errors
    
    # Save robustness CSV
    csv_path_robust = os.path.join(EVIDENCE_DIR, "stage2_snr_robustness.csv")
    with open(csv_path_robust, "w") as f:
        f.write("condition,median_ang_err_deg,mean_ang_err_deg,p90_ang_err_deg,success_pct_lt_5deg\n")
        for cond, errs in robustness_results.items():
            med = float(np.median(errs))
            mean = float(np.mean(errs))
            p90 = float(np.percentile(errs, 90))
            succ = float(np.mean(np.array(errs) <= 5.0)) * 100.0
            f.write(f"{cond},{med:.2f},{mean:.2f},{p90:.2f},{succ:.1f}\n")
            print(f"    --> {cond:<18}: Median={med:4.2f}°, Mean={mean:4.2f}°, 90th%={p90:4.2f}°, Pass Rate={succ:4.1f}%")
    print(f"    [SAVED] {csv_path_robust}")

    # Plot CDF and Error Distribution Curves
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), facecolor="#0a0e17")
    fig.suptitle("Stage 2D: Acoustic Source Localization Robustness under Noise & Reverberation",
                 color="#00ffcc", fontsize=12, weight="bold")
    
    colors = ["#00ffcc", "#33ccff", "#ffaa00", "#ff0077", "#bb66ff"]
    for idx, (cond, errs) in enumerate(robustness_results.items()):
        sorted_errs = np.sort(errs)
        cdf = np.arange(1, len(sorted_errs) + 1) / len(sorted_errs) * 100.0
        ax1.plot(sorted_errs, cdf, color=colors[idx], linewidth=2.0, label=cond)
        
    ax1.set_facecolor("#101726")
    ax1.axvline(5.0, color="#ffffff", linestyle="--", alpha=0.5, label="5° Threshold")
    ax1.set_title("Cumulative Error Distribution (CDF)", color="#ffffff", fontsize=10.5)
    ax1.set_xlabel("Absolute Angular Error (degrees)", color="#b8c2d1", fontsize=9)
    ax1.set_ylabel("Percentage of Test Cases (%)", color="#b8c2d1", fontsize=9)
    ax1.set_xlim(0, 20)
    ax1.set_ylim(0, 105)
    ax1.tick_params(colors="#b8c2d1", labelsize=8)
    ax1.grid(alpha=0.15, color="#34496d")
    ax1.legend(facecolor="#0a0e17", edgecolor="#2a3b5c", fontsize=8.5, loc="lower right")
    
    # Bar plot of median & 90th percentile errors
    ax2.set_facecolor("#101726")
    x_indices = np.arange(len(robustness_results))
    medians = [float(np.median(e)) for e in robustness_results.values()]
    p90s = [float(np.percentile(e, 90)) for e in robustness_results.values()]
    
    w = 0.35
    ax2.bar(x_indices - w/2, medians, width=w, color="#00ffcc", label="Median Error (°)")
    ax2.bar(x_indices + w/2, p90s, width=w, color="#ff0077", label="90th% Error (°)")
    ax2.set_xticks(x_indices)
    ax2.set_xticklabels(robustness_results.keys(), rotation=18, color="#b8c2d1", fontsize=8.5)
    ax2.set_title("Summary Error Metrics across Noise Levels", color="#ffffff", fontsize=10.5)
    ax2.set_ylabel("Angular Error (degrees)", color="#b8c2d1", fontsize=9)
    ax2.tick_params(colors="#b8c2d1", labelsize=8)
    ax2.grid(alpha=0.15, color="#34496d")
    ax2.legend(facecolor="#0a0e17", edgecolor="#2a3b5c", fontsize=8.5, loc="upper left")
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.88)
    fig_path_snr = os.path.join(EVIDENCE_DIR, "stage2_snr_robustness_curves.png")
    fig.savefig(fig_path_snr, dpi=200)
    plt.close(fig)
    print(f"    [SAVED] {fig_path_snr}")

    # -------------------------------------------------------------------------
    # PART 4: Package Evidence Zip
    # -------------------------------------------------------------------------
    zip_path = os.path.join(EVIDENCE_DIR, "stage2_localization_evidence.zip")
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        for f in [csv_path_bench, csv_path_robust, fig_path_tdoa, fig_path_heat, fig_path_snr]:
            z.write(f, os.path.basename(f))
    print(f"\n[+] Successfully created Evidence Archive: {zip_path}")
    print("=" * 80)
    print(" STAGE 2 ACOUSTIC LOCALIZATION SUITE COMPLETED SUCCESSFULLY!")
    print("=" * 80)

if __name__ == "__main__":
    run_stage2_evaluation()
