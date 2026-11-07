"""
=============================================================================
 DUAL-I2S INTER-PAIR PHASE CALIBRATION & CLOCK-SKEW AUDIT TOOL
=============================================================================
Module: calibrate_i2s_phase.py
Purpose: Quantifies and compensates for hardware DMA startup latency between
         I2S0 (Pair A: Mics 1 & 2) and I2S1 (Pair B: Mics 3 & 4) on classic ESP32.
Author: Antigravity Autonomous Agent (Pair Programming with Yogi)
Target Milestone: ECE Minor Project Evaluation (7 November 2026)
=============================================================================
"""

import os
import sys
import json
import wave
import argparse
import numpy as np
from scipy import signal
from scipy.fft import rfft, irfft
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


SPEED_OF_SOUND = 343.0  # m/s in air at 20°C
DEFAULT_ARRAY_GEOMETRY = {
    "spacing_m": 0.02032,  # 20.32 mm spacing (0.8 inches breadboard pitch)
    "mic_positions": [
        {"id": 0, "name": "Mic 1 (Pair A Left, GPIO 19)",  "x": -0.03048, "pair": "A"},
        {"id": 1, "name": "Mic 2 (Pair A Right, GPIO 19)", "x": -0.01016, "pair": "A"},
        {"id": 2, "name": "Mic 3 (Pair B Left, GPIO 23)",  "x": +0.01016, "pair": "B"},
        {"id": 3, "name": "Mic 4 (Pair B Right, GPIO 23)", "x": +0.03048, "pair": "B"},
    ]
}


def load_wav_multichannel(wav_path: str) -> tuple:
    """Loads a multi-channel PCM WAV file into float32 numpy array normalized to [-1.0, 1.0]."""
    if not os.path.exists(wav_path):
        raise FileNotFoundError(f"Target WAV file not found: {wav_path}")
    
    with wave.open(wav_path, 'rb') as wf:
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        n_frames = wf.getnframes()
        raw_bytes = wf.readframes(n_frames)
    
    if sample_width == 2:
        dtype = np.int16
        scale = 32768.0
    elif sample_width == 4:
        dtype = np.int32
        scale = 2147483648.0
    else:
        raise ValueError(f"Unsupported sample width: {sample_width} bytes")
    
    audio_data = np.frombuffer(raw_bytes, dtype=dtype).reshape(-1, n_channels)
    audio_float = audio_data.astype(np.float32) / scale
    return audio_float, sample_rate, n_channels


def gcc_phat(sig1: np.ndarray, sig2: np.ndarray, fs: float, max_tau: float = None, interp: int = 16) -> tuple:
    """
    Computes Generalized Cross-Correlation with Phase Transform (GCC-PHAT)
    using sinc frequency-domain zero-padding and parabolic sub-sample refinement.
    
    Returns:
        tau_seconds: Estimated time delay of sig1 relative to sig2.
        cross_corr: Interpolated cross-correlation vector.
        lags: Vector of lag times in seconds.
    """
    n1 = len(sig1)
    n2 = len(sig2)
    n = max(n1, n2)
    
    # Tukey window to reduce edge effects
    w = signal.windows.tukey(n, alpha=0.15)
    s1 = (sig1[:n] - np.mean(sig1[:n])) * w[:len(sig1[:n])]
    s2 = (sig2[:n] - np.mean(sig2[:n])) * w[:len(sig2[:n])]
    
    X1 = np.fft.rfft(s1, n=n)
    X2 = np.fft.rfft(s2, n=n)
    
    # R = X2 * conj(X1) so positive delay corresponds to sig2 delayed relative to sig1
    R = X2 * np.conj(X1)
    denom = np.abs(R)
    denom = np.maximum(denom, 1e-4 * np.max(denom))
    R_phat = np.where(denom > 1e-15, R / denom, 0.0)
    
    # Sinc interpolation via zero-padding in frequency domain
    n_interp = n * interp
    n_freq = n_interp // 2 + 1
    cross_pad = np.zeros(n_freq, dtype=complex)
    cross_pad[:len(R_phat)] = R_phat
    
    cc = np.fft.fftshift(np.fft.irfft(cross_pad, n=n_interp))
    dt_fine = 1.0 / (fs * interp)
    lags = np.arange(-n_interp // 2, n_interp // 2) * dt_fine
    
    if max_tau is not None:
        mask = np.abs(lags) <= max_tau
        cc_search = cc[mask]
        lags_search = lags[mask]
    else:
        cc_search = cc
        lags_search = lags
        
    peak_idx = int(np.argmax(cc_search))
    tau_seconds = lags_search[peak_idx]
    
    # Parabolic sub-sample refinement around peak
    if 0 < peak_idx < len(cc_search) - 1:
        y0 = float(cc_search[peak_idx - 1])
        y1 = float(cc_search[peak_idx])
        y2 = float(cc_search[peak_idx + 1])
        denom_p = 2.0 * (y0 - 2.0 * y1 + y2)
        if abs(denom_p) > 1e-12:
            delta = (y0 - y2) / denom_p
            tau_seconds += delta * dt_fine
            
    return tau_seconds, cc_search, lags_search


def audit_dual_i2s_phase(wav_path: str, output_dir: str, sound_incident_angle_deg: float = 0.0) -> dict:
    """
    Audits the cross-pair synchronization between I2S0 (Pair A) and I2S1 (Pair B).
    
    If sound is incident at angle theta (broadside = 0°):
      Theoretical tau between Mic i and Mic j: (x_j - x_i) * sin(theta) / c
      Any residual systematic delay between Pair A and Pair B is the hardware DMA clock offset.
    """
    audio, fs, channels = load_wav_multichannel(wav_path)
    print(f"[*] Loaded: {os.path.basename(wav_path)}")
    print(f"    Channels: {channels} | Sample Rate: {fs} Hz | Duration: {len(audio)/fs:.3f} s")
    
    if channels < 4:
        raise ValueError(f"Dual-I2S audit requires at least 4 channels, got {channels}.")
    
    mics = DEFAULT_ARRAY_GEOMETRY["mic_positions"]
    theta_rad = np.radians(sound_incident_angle_deg)
    
    # Analyze all 6 microphone pairs
    pairs = [
        (0, 1, "Intra-Pair A (Mic1-Mic2, I2S0)"),
        (2, 3, "Intra-Pair B (Mic3-Mic4, I2S1)"),
        (1, 2, "Inter-Pair Center (Mic2-Mic3, I2S0 vs I2S1)"),
        (0, 3, "Inter-Pair Outer (Mic1-Mic4, I2S0 vs I2S1)"),
        (0, 2, "Cross-Pair 1-3 (I2S0 vs I2S1)"),
        (1, 3, "Cross-Pair 2-4 (I2S0 vs I2S1)")
    ]
    
    # Max physically possible acoustic transit time across array (60.96 mm / 343 m/s ≈ 178 μs)
    max_acoustic_tau = 0.06096 / SPEED_OF_SOUND * 1.5
    
    results = []
    print("\n" + "="*85)
    print(" MICROPHONE PAIR CROSS-CORRELATION & SKEW ANALYSIS")
    print("="*85)
    print(f" Reference Incident Angle: {sound_incident_angle_deg:+.1f}°")
    print(f"{'Pair':<35} | {'Theo Delay (μs)':<15} | {'Meas Delay (μs)':<15} | {'Skew Error (μs)':<15}")
    print("-"*85)
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    axes = axes.flatten()
    
    skew_inter_pair_samples = []
    
    for idx, (m1, m2, label) in enumerate(pairs):
        sig1 = audio[:, m1]
        sig2 = audio[:, m2]
        
        # Theoretical geometric delay: sound arrives at mic m1 vs mic m2
        # tau = (x_m2 - x_m1) * sin(theta) / c
        theo_tau = (mics[m2]["x"] - mics[m1]["x"]) * np.sin(theta_rad) / SPEED_OF_SOUND
        
        # Measure via GCC-PHAT
        meas_tau, cc, lags = gcc_phat(sig1, sig2, fs, max_tau=max_acoustic_tau, interp=32)
        skew_sec = meas_tau - theo_tau
        skew_samples = skew_sec * fs
        
        theo_us = theo_tau * 1e6
        meas_us = meas_tau * 1e6
        skew_us = skew_sec * 1e6
        
        print(f"{label:<35} | {theo_us:+14.2f}  | {meas_us:+14.2f}  | {skew_us:+14.2f}")
        
        results.append({
            "pair": [m1, m2],
            "label": label,
            "theo_tau_us": float(theo_us),
            "meas_tau_us": float(meas_us),
            "skew_us": float(skew_us),
            "skew_samples": float(skew_samples)
        })
        
        if "Inter-Pair" in label or "Cross-Pair" in label:
            skew_inter_pair_samples.append(skew_samples)
            
        # Plot GCC-PHAT lag curve
        ax = axes[idx]
        ax.plot(lags * 1e6, cc / np.max(np.abs(cc) + 1e-12), color='royalblue', lw=1.5, label='GCC-PHAT')
        ax.axvline(theo_us, color='green', linestyle='--', lw=1.8, label=f'Theoretical ({theo_us:.1f} μs)')
        ax.axvline(meas_us, color='crimson', linestyle=':', lw=1.8, label=f'Measured ({meas_us:.1f} μs)')
        ax.set_title(f"{label}\nSkew = {skew_us:+.2f} μs ({skew_samples:+.2f} smp)", fontsize=10, fontweight='bold')
        ax.set_xlabel("Lag (microseconds)", fontsize=9)
        ax.set_ylabel("Normalized PHAT", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')
    
    plt.tight_layout()
    plot_path = os.path.join(output_dir, "dual_i2s_phase_skew_audit.png")
    plt.savefig(plot_path, dpi=200)
    plt.close()
    print(f"\n[+] Diagnostic plot saved: {plot_path}")
    
    mean_inter_pair_skew_samples = float(np.mean(skew_inter_pair_samples))
    mean_inter_pair_skew_us = mean_inter_pair_skew_samples / fs * 1e6
    
    # Assess synchronization status
    # Tolerable threshold: < 0.5 sample error at sample rate
    is_sync_healthy = abs(mean_inter_pair_skew_samples) < 0.75
    
    calibration_data = {
        "audio_file": os.path.basename(wav_path),
        "sample_rate_hz": fs,
        "incident_angle_deg": sound_incident_angle_deg,
        "inter_pair_mean_skew_us": mean_inter_pair_skew_us,
        "inter_pair_mean_skew_samples": mean_inter_pair_skew_samples,
        "integer_sample_compensation": int(np.round(mean_inter_pair_skew_samples)),
        "fractional_sample_compensation": float(mean_inter_pair_skew_samples - np.round(mean_inter_pair_skew_samples)),
        "is_sync_healthy": is_sync_healthy,
        "status_diagnosis": "LOCKED (Clocks aligned within <1 sample)" if is_sync_healthy else "SKEWED (Inter-pair DMA delay detected)",
        "pair_details": results
    }
    
    json_path = os.path.join(output_dir, "i2s_phase_calibration.json")
    with open(json_path, 'w') as f:
        json.dump(calibration_data, f, indent=2)
    print(f"[+] Calibration config saved: {json_path}")
    
    print("\n" + "="*85)
    print(f" DUAL-I2S SYNCHRONIZATION DIAGNOSIS: {calibration_data['status_diagnosis']}")
    print(f" Inter-Pair DMA Mean Skew: {mean_inter_pair_skew_us:+.2f} μs ({mean_inter_pair_skew_samples:+.2f} samples)")
    print(f" Recommended Compensation: Shift Pair B by {-calibration_data['integer_sample_compensation']} integer samples")
    print("="*85)
    
    return calibration_data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Audit and Calibrate Dual-I2S Clock Skew on Classic ESP32")
    parser.add_argument("--wav", type=str, default="", help="Path to 4-channel WAV file")
    parser.add_argument("--angle", type=float, default=0.0, help="Sound incidence angle in degrees (default: 0.0 broadside)")
    parser.add_argument("--outdir", type=str, default="", help="Output directory for plots and JSON")
    args = parser.parse_args()
    
    proj_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    stage2_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    cand_wavs = [
        args.wav,
        os.path.join(stage2_dir, "evidence", "multimodal_4mic_audio.wav"),
        os.path.join(proj_dir, "evidence", "stage2", "multimodal_4mic_audio.wav")
    ]
    target_wav = next((w for w in cand_wavs if w and os.path.exists(w)), "")
    target_out = args.outdir if args.outdir else os.path.join(stage2_dir, "evidence")
    os.makedirs(target_out, exist_ok=True)
    
    audit_dual_i2s_phase(target_wav, target_out, sound_incident_angle_deg=args.angle)
