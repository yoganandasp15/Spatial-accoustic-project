"""
=============================================================================
SPATIAL MULTIMODAL ACOUSTIC RADAR & FAULT LOCALIZATION ENGINE (PRO V6)
=============================================================================
Industrial-Grade 4-Microphone Acoustic Camera & Multimodal Telemetry Suite
Integrated for ESP32 Dual-I2S Master/Slave Array + ADXL345 + FC-03 Tachometer

Features:
  1. Full 4-Microphone Uniform Linear Array (ULA) Support:
     - Mic 1: Pair A Left  (GPIO 19, Row 20) -> x = -30.48 mm
     - Mic 2: Pair A Right (GPIO 19, Row 28) -> x = -10.16 mm
     - Mic 3: Pair B Left  (GPIO 23, Row 36) -> x = +10.16 mm
     - Mic 4: Pair B Right (GPIO 23, Row 44) -> x = +30.48 mm
  2. Multi-Pair Steered Response Power (SRP-PHAT) with 6 acoustic baseline pairs.
  3. Live 4-Channel Input Oscilloscope & Dynamic VU Meters (Color-Coded).
  4. Real-time ADXL345 3-Axis Vibration Gauges & ISO 10816 Severity Indicator.
  5. FC-03 Optical Tachometer Shaft RPM & Pulse Monitor.
  6. Real-Time Rotating Machinery Acoustic Fault Classifier.
  7. Dual-Mode Hardware Ingestion (Binary 4-CH PCM + Live ASCII Telemetry + Auto-Demo Fallback).
=============================================================================
"""

import collections
import json
import math
import os
import struct
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import sounddevice as sd
from matplotlib.animation import FuncAnimation
from scipy import signal
from scipy.fft import irfft, rfft
from scipy.signal import welch
from scipy.stats import kurtosis

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# =============================================================================
# GLOBAL SYSTEM CONSTANTS & CONFIGURATION
# =============================================================================
SPEED_OF_SOUND = 343.0  # Dry air at 20°C (m/s)

# Spatial Heatmap Grid Boundary
X_MIN, X_MAX = -2.0, 2.0   # Left / Right (meters)
Y_MIN, Y_MAX =  0.2, 3.5   # Range in front of array (meters)
GRID_X = 51
GRID_Y = 45

# Acoustic Filter Bounds (Hz) - DC Motor & Voice Bandwidth (Rejects DC drift & sub-rumble)
FREQ_MIN = 80.0
FREQ_MAX = 5500.0

# Frame & Animation Timing
UPDATE_INTERVAL_MS = 50     # 20 FPS GUI refresh
HEATMAP_SMOOTH_ALPHA = 0.30  # Surface temporal smoothing


def apply_pair_phase_compensation(
    audio_4ch: np.ndarray,
    fs: float,
    comp_delay_sec: float
) -> np.ndarray:
    """
    Applies exact frequency-domain fractional delay phase compensation to Pair B (channels 2 & 3):
    H(f) = exp(-j * 2 * pi * f * comp_delay_sec)
    """
    if abs(comp_delay_sec) < 1e-12 or audio_4ch.shape[1] < 4:
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


# =============================================================================
# DYNAMIC TARGET-LOCK STATE-SPACE TRACKER
# =============================================================================
class DynamicTargetTracker:
    """Adaptive Target-Lock Tracker with fast re-acquisition and smooth slew."""

    def __init__(self, alpha: float = 0.40, gate_deg: float = 30.0):
        self.alpha = alpha
        self.gate_deg = gate_deg
        self.tracked_angle = 0.0
        self.tracked_range = 1.25
        self.confidence = 0.0
        self.locked = False
        self.pending_angle = 0.0
        self.pending_count = 0
        self.last_valid_time = time.time()

    def reset(self):
        self.tracked_angle = 0.0
        self.tracked_range = 1.25
        self.confidence = 0.0
        self.locked = False
        self.pending_count = 0

    def update(
        self,
        meas_angle: float,
        meas_prom: float,
        is_active_sound: bool,
        nominal_range: float = 1.25
    ) -> Tuple[float, float, float, bool]:
        now = time.time()

        if not is_active_sound or meas_prom < 0.35:
            dt_silence = now - self.last_valid_time
            if dt_silence > 0.40:
                self.confidence *= 0.90
                if self.confidence < 15.0:
                    self.locked = False
            return self.tracked_angle, self.tracked_range, self.confidence, False

        self.last_valid_time = now
        meas_conf_pct = min(99.0, max(30.0, meas_prom * 60.0))

        if not self.locked:
            self.tracked_angle = meas_angle
            self.tracked_range = nominal_range
            self.confidence = meas_conf_pct
            self.locked = True
            self.pending_count = 0
        else:
            diff = meas_angle - self.tracked_angle
            if abs(diff) <= self.gate_deg:
                self.tracked_angle += self.alpha * diff
                self.tracked_range = 0.85 * self.tracked_range + 0.15 * nominal_range
                self.confidence = (1.0 - self.alpha) * self.confidence + self.alpha * meas_conf_pct
                self.pending_count = 0
            else:
                # Fast re-acquisition when sound moves to opposite flank or off-axis
                if meas_prom > 0.50:
                    self.tracked_angle = 0.20 * self.tracked_angle + 0.80 * meas_angle
                    self.confidence = meas_conf_pct
                    self.pending_count = 0
                else:
                    self.pending_angle = meas_angle
                    self.pending_count += 1
                    if self.pending_count >= 2:
                        self.tracked_angle = meas_angle
                        self.confidence = meas_conf_pct
                        self.pending_count = 0
                    self.confidence = max(25.0, self.confidence * 0.95)

        return self.tracked_angle, self.tracked_range, self.confidence, True


# =============================================================================
# SCALABLE ARRAY GEOMETRY (4-MIC ULA & 2-MIC PRESETS)
# =============================================================================
class ArrayGeometry:
    """Manages physical microphone coordinates and precomputes spatial delay manifolds."""

    def __init__(self, name: str, mic_coords: np.ndarray, description: str = ""):
        self.name = name
        self.mic_coords = np.asarray(mic_coords, dtype=np.float64)
        self.num_mics = len(mic_coords)
        self.description = description

        self.pairs: List[Tuple[int, int]] = [
            (i, j) for i in range(self.num_mics) for j in range(i + 1, self.num_mics)
        ]
        self.num_pairs = len(self.pairs)

        # Baseline spacing between reference pair (0, 1)
        self.baseline_m = float(np.linalg.norm(self.mic_coords[0] - self.mic_coords[1]))

        # 2D Cartesian Grid
        self.xs = np.linspace(X_MIN, X_MAX, GRID_X)
        self.ys = np.linspace(Y_MIN, Y_MAX, GRID_Y)
        self.xx, self.yy = np.meshgrid(self.xs, self.ys)

        self.pair_delays = self._precompute_delays()

    def set_baseline(self, new_baseline_m: float):
        """Live adjustment of microphone pitch."""
        pitch = max(0.010, min(0.15, new_baseline_m))
        self.baseline_m = pitch
        if self.num_mics == 4:
            # Symmetrical 4-mic linear array: [-1.5d, -0.5d, +0.5d, +1.5d]
            self.mic_coords = np.array([
                [-1.5 * pitch, 0.0],
                [-0.5 * pitch, 0.0],
                [+0.5 * pitch, 0.0],
                [+1.5 * pitch, 0.0]
            ])
            self.pair_delays = self._precompute_delays()
        elif self.num_mics == 2:
            half = pitch / 2.0
            self.mic_coords = np.array([[-half, 0.0], [half, 0.0]])
            self.pair_delays = self._precompute_delays()

    def _precompute_delays(self) -> np.ndarray:
        distances = []
        for mx, my in self.mic_coords:
            r = np.sqrt((self.xx - mx) ** 2 + (self.yy - my) ** 2)
            distances.append(r)

        delays = []
        for i, j in self.pairs:
            tau = (distances[i] - distances[j]) / SPEED_OF_SOUND
            delays.append(tau)

        return np.asarray(delays, dtype=np.float64)


PRESETS = {
    "ESP32_4MIC_LINEAR": ArrayGeometry(
        "ESP32 Quad INMP441 Linear Array (20.32mm Pitch, 60.96mm Aperture)",
        np.array([
            [-0.03048, 0.0],  # Mic 1: Pair A Left (GPIO 19, Row 20)
            [-0.01016, 0.0],  # Mic 2: Pair A Right (GPIO 19, Row 28)
            [+0.01016, 0.0],  # Mic 3: Pair B Left (GPIO 23, Row 36)
            [+0.03048, 0.0]   # Mic 4: Pair B Right (GPIO 23, Row 44)
        ]),
        "Physical 4-microphone ULA along breadboard: 6 delay pairs with high angular sharpness."
    ),
    "ESP32_2MIC": ArrayGeometry(
        "ESP32 Dual INMP441 Pair A (20.32mm Baseline)",
        np.array([[-0.01016, 0.0], [0.01016, 0.0]]),
        "Physical INMP441 breadboard pair: bearing angle [-90°, +90°]."
    ),
    "LAPTOP_DUAL": ArrayGeometry(
        "Laptop Dual Mic Array (Intel SST)",
        np.array([[-0.060, 0.0], [0.060, 0.0]]),
        "Laptop top-bezel stereo mics (12 cm baseline)."
    ),
    "ESP32_4MIC_SQUARE": ArrayGeometry(
        "4-Mic Planar Square (10x10 cm)",
        np.array([[-0.050, -0.050], [0.050, -0.050], [0.050, 0.050], [-0.050, 0.050]]),
        "4-element planar array: full 2D focal hotspot."
    )
}


# =============================================================================
# MULTI-PAIR SUPER-RESOLUTION GCC-PHAT & SRP ENGINE
# =============================================================================
class SRPPhatEngine:
    """Steered Response Power (SRP) Engine with Multi-Pair Sinc GCC-PHAT."""

    @staticmethod
    def gcc_phat_pair(
        sig_i: np.ndarray,
        sig_j: np.ndarray,
        fs: float,
        d_ij_m: float,
        gamma: float = 0.70,
        interp_factor: int = 16
    ) -> Tuple[float, float, np.ndarray, np.ndarray]:
        """
        Computes super-resolution sinc GCC-PHAT between microphone pair (i, j).
        Convention: sig_i is leftward (smaller x), sig_j is rightward (larger x).
        """
        n = len(sig_i)
        w = signal.windows.tukey(n, alpha=0.25)

        si = (sig_i - np.mean(sig_i)) * w
        sj = (sig_j - np.mean(sig_j)) * w

        std_i = float(np.std(si))
        std_j = float(np.std(sj))
        if std_i > 1e-6: si = si / std_i
        if std_j > 1e-6: sj = sj / std_j

        Si = rfft(si)
        Sj = rfft(sj)

        cross = Si * np.conj(Sj)
        mag = np.abs(cross)

        freqs = np.fft.rfftfreq(n, d=1.0 / fs)
        band_mask = (freqs >= FREQ_MIN) & (freqs <= min(FREQ_MAX, fs / 2.0 - 50.0))

        inband_energy = float(np.sum(mag[band_mask] ** 2))
        total_energy = float(np.sum(mag ** 2)) + 1e-20
        if inband_energy / total_energy < 0.025:
            dummy_lags = np.linspace(-n / (2 * fs), n / (2 * fs), 3)
            return 0.0, 0.0, np.zeros(3), dummy_lags

        cross[~band_mask] = 0.0
        peak_mag = max(1e-12, float(np.max(mag ** gamma))) if len(mag) > 0 else 1.0
        denom = np.maximum(mag ** gamma, 1e-4 * peak_mag)
        cross_phat = np.where(denom > 1e-15, cross / denom, 0.0)

        n_interp = n * interp_factor
        n_freq_interp = n_interp // 2 + 1
        cross_padded = np.zeros(n_freq_interp, dtype=complex)
        cross_padded[:len(cross_phat)] = cross_phat

        corr = np.fft.fftshift(irfft(cross_padded, n=n_interp))
        dt_fine = 1.0 / (fs * interp_factor)
        lags = np.arange(-n_interp // 2, n_interp // 2) * dt_fine

        tau_max = (d_ij_m / SPEED_OF_SOUND) * 1.08
        phys_mask = np.abs(lags) <= tau_max

        c_phys = corr[phys_mask]
        l_phys = lags[phys_mask]

        if len(c_phys) < 3:
            return 0.0, 0.0, c_phys, l_phys

        k = int(np.argmax(c_phys))
        peak_val = float(c_phys[k])
        mean_val = float(np.mean(c_phys))
        std_val = float(np.std(c_phys)) + 1e-12
        prominence = max(0.0, (peak_val - mean_val) / std_val)

        if k == 0 or k == len(c_phys) - 1:
            return 0.0, 0.0, c_phys, l_phys

        if 0 < k < len(c_phys) - 1:
            y0, y1, y2 = float(c_phys[k - 1]), float(c_phys[k]), float(c_phys[k + 1])
            denom_p = 2.0 * (y0 - 2.0 * y1 + y2)
            if abs(denom_p) > 1e-12:
                delta = (y0 - y2) / denom_p
                delta = max(-0.5, min(0.5, delta))
                best_tau = float(l_phys[k] + delta * dt_fine)
            else:
                best_tau = float(l_phys[k])
        else:
            best_tau = float(l_phys[k])

        best_tau = max(-tau_max, min(tau_max, best_tau))
        return best_tau, prominence, c_phys, l_phys

    @classmethod
    def compute_srp_frame(
        cls,
        signals: np.ndarray,
        fs: float,
        geometry: ArrayGeometry
    ) -> Tuple[float, float, np.ndarray, np.ndarray, np.ndarray, Dict[str, object]]:
        """
        Steered Response Power (SRP) bearing estimation AND True 2D Acoustic Camera Backprojection.
        Returns:
            best_bearing_deg: Estimated angle in degrees [-90°, +90°]
            composite_prominence: SNR prominence of the aggregate peak
            angles_deg: Angle grid (181 points)
            srp_norm: Normalized 1D SRP power vs angle
            raw_2d_heatmap: True 2D backprojected acoustic intensity field (GRID_Y, GRID_X)
            side_metrics: Directional power breakdown (pct_left, pct_center, pct_right, dom_side)
        """
        num_mics = min(signals.shape[1], geometry.num_mics)
        angles_deg = np.linspace(-90.0, 90.0, 181)
        angles_rad = np.deg2rad(angles_deg)
        sin_angles = np.sin(angles_rad)

        srp_profile = np.zeros_like(angles_deg, dtype=np.float64)
        raw_2d_heatmap = np.zeros((GRID_Y, GRID_X), dtype=np.float64)
        pair_prominences = []

        for p_idx, (i, j) in enumerate(geometry.pairs):
            if i >= num_mics or j >= num_mics:
                continue

            sig_i = signals[:, i]
            sig_j = signals[:, j]
            d_x = geometry.mic_coords[j, 0] - geometry.mic_coords[i, 0]
            d_ij = abs(d_x) if abs(d_x) > 1e-4 else geometry.baseline_m

            best_tau, prom, c_phys, l_phys = cls.gcc_phat_pair(sig_i, sig_j, fs, d_ij)
            pair_prominences.append(prom)

            if len(c_phys) >= 3 and prom >= 0.20:
                # 1D angular SRP: tau = +(d_x / c) * sin(theta) (Positive lag for sound on right)
                taus_1d = +(d_x / SPEED_OF_SOUND) * sin_angles
                pair_resp_1d = np.interp(taus_1d, l_phys, c_phys, left=0.0, right=0.0)
                pair_weight = math.sqrt(d_ij / geometry.baseline_m) * min(prom, 3.0)
                srp_profile += pair_weight * np.maximum(pair_resp_1d, 0.0)

                # 2D True Spatial SRP Backprojection across physical room grid
                tau_grid = +geometry.pair_delays[p_idx].ravel()
                pair_resp_2d = np.interp(tau_grid, l_phys, c_phys, left=0.0, right=0.0).reshape((GRID_Y, GRID_X))
                raw_2d_heatmap += pair_weight * np.maximum(pair_resp_2d, 0.0)

        if np.max(srp_profile) > 1e-9:
            srp_norm = srp_profile / np.max(srp_profile)
            k_peak = int(np.argmax(srp_norm))
            best_bearing_deg = float(angles_deg[k_peak])
            mean_srp = float(np.mean(srp_norm))
            std_srp = float(np.std(srp_norm)) + 1e-12
            composite_prom = max(0.0, (1.0 - mean_srp) / std_srp)
        else:
            srp_norm = np.zeros_like(angles_deg)
            best_bearing_deg = 0.0
            composite_prom = 0.0

        # Sector Power & Side Intensity Breakdown with Noise-Floor Contrast
        l_mask = angles_deg < -15.0
        c_mask = (angles_deg >= -15.0) & (angles_deg <= 15.0)
        r_mask = angles_deg > 15.0

        baseline = float(np.percentile(srp_norm, 20)) if len(srp_norm) > 0 else 0.0
        contrast = np.maximum(0.0, srp_norm - baseline) ** 2.0

        p_l = float(np.sum(contrast[l_mask])) if np.any(l_mask) else 0.0
        p_c = float(np.sum(contrast[c_mask])) if np.any(c_mask) else 0.0
        p_r = float(np.sum(contrast[r_mask])) if np.any(r_mask) else 0.0
        p_tot = p_l + p_c + p_r + 1e-12

        pct_left = (p_l / p_tot) * 100.0
        pct_center = (p_c / p_tot) * 100.0
        pct_right = (p_r / p_tot) * 100.0

        if best_bearing_deg > 15.0:
            dom_side = "RIGHT"
        elif best_bearing_deg < -15.0:
            dom_side = "LEFT"
        else:
            dom_side = "CENTER"

        side_metrics = {
            "pct_left": pct_left,
            "pct_center": pct_center,
            "pct_right": pct_right,
            "dom_side": dom_side
        }

        return best_bearing_deg, composite_prom, angles_deg, srp_norm, raw_2d_heatmap, side_metrics

    @classmethod
    def compute_srp_bearing(
        cls,
        signals: np.ndarray,
        fs: float,
        geometry: ArrayGeometry
    ) -> Tuple[float, float, np.ndarray, np.ndarray]:
        """Backward-compatible wrapper for 1D bearing estimation."""
        best_bearing, comp_prom, angles, srp_norm, _, _ = cls.compute_srp_frame(signals, fs, geometry)
        return best_bearing, comp_prom, angles, srp_norm

    @classmethod
    def compute_true_2d_acoustic_heatmap(
        cls,
        raw_2d_heatmap: np.ndarray,
        target_range_m: float,
        avg_rms: float,
        is_locked: bool,
        geometry: ArrayGeometry
    ) -> np.ndarray:
        """
        Transforms raw SRP cross-correlation backprojection into dynamic radiometric acoustic camera thermal imagery:
        - Near-field spherical divergence weighting
        - Non-linear thermal contrast enhancement
        - Live acoustic energy modulation (quiet room = cold dark floor; speech = vibrant thermal bloom)
        """
        r_grid = np.sqrt(geometry.xx ** 2 + geometry.yy ** 2)
        dist_weight = 1.0 / np.sqrt(np.maximum(r_grid, 0.40))
        dist_weight /= np.max(dist_weight)

        # Removed artificial radial focal ring (nominal_range = 1.25m prior).
        # Linear array naturally constrains azimuth bearing theta;
        # radial dimension reflects true backprojected energy with spherical divergence weighting.
        spatial_filter = dist_weight

        filtered_heat = raw_2d_heatmap * spatial_filter
        max_heat = np.max(filtered_heat)
        if max_heat > 1e-9:
            norm_heat = filtered_heat / max_heat
            # High-contrast thermal radiation curve (FLIR style)
            thermal_heat = norm_heat ** 2.2
        else:
            thermal_heat = np.zeros_like(raw_2d_heatmap)

        # Modulate thermal glow by live signal RMS energy
        rms_gain = np.clip((avg_rms - 0.003) / 0.12, 0.05, 1.0) if avg_rms > 0.003 else 0.02
        acoustic_heat = thermal_heat * rms_gain
        return np.clip(acoustic_heat, 0.0, 1.0)

    @classmethod
    def synthesize_directional_map(
        cls,
        bearing_deg: float,
        target_range_m: float,
        confidence_pct: float,
        geometry: ArrayGeometry
    ) -> np.ndarray:
        """Synthetic fallback spotlight."""
        xx, yy = geometry.xx, geometry.yy
        angles_grid = np.rad2deg(np.arctan2(xx, yy))
        ranges_grid = np.sqrt(xx ** 2 + yy ** 2)

        beam_width = 8.0 if geometry.num_mics >= 4 else 10.0
        ang_diff = angles_grid - bearing_deg
        beam = np.exp(-0.5 * (ang_diff / beam_width) ** 2)

        r_weight = 1.0 / np.sqrt(np.maximum(ranges_grid, 0.4))
        r_weight /= np.max(r_weight)

        range_diff = ranges_grid - target_range_m
        spot = np.exp(-0.5 * (range_diff / 0.70) ** 2)

        intensity = 0.65 * beam * r_weight + 0.35 * beam * spot
        effective_conf = confidence_pct / 100.0
        intensity *= effective_conf
        return np.clip(intensity, 0.0, 1.0)


# =============================================================================
# ACOUSTIC EVENT CLASSIFIER & ISO 10816 VIBRATION SEVERITY
# =============================================================================
class AcousticClassifier:
    """Diagnoses acoustic & vibration signatures for rotating machinery."""

    @staticmethod
    def classify(audio: np.ndarray, fs: float, vib_mag_g: float) -> Dict[str, object]:
        x = audio.mean(axis=1).astype(np.float64)
        x -= np.mean(x)
        rms = float(np.sqrt(np.mean(x ** 2)))
        peak = float(np.max(np.abs(x))) if len(x) else 0.0

        # Dynamic Vibration Severity Index (Empirical zones inspired by ISO 10816 Class 1)
        # Note: Strict ISO 10816 requires velocity RMS (mm/s). Here, dynamic acceleration magnitude
        # (dynamic_g = |vib_mag - 1.0g| in g) provides an indicative edge severity proxy.
        dynamic_g = max(0.0, vib_mag_g - 1.0) # Subtract 1g static gravity
        if dynamic_g > 0.45:
            severity = "ALERT: EXCESSIVE VIBRATION (ISO Zone D Proxy)"
        elif dynamic_g > 0.20:
            severity = "WARNING: ELEVATED VIBRATION (ISO Zone C Proxy)"
        else:
            severity = "NOMINAL: HEALTHY VIBRATION (ISO Zone A/B Proxy)"

        if rms < 1e-4:
            return {
                "label": "AMBIENT / QUIET",
                "confidence": 0,
                "severity": severity,
                "detail": "Acoustic signal within background noise floor."
            }

        crest = peak / max(rms, 1e-12)
        kurt = float(kurtosis(x, fisher=False, bias=False))

        f, pxx = welch(x, fs=fs, nperseg=min(1024, len(x)), noverlap=None)
        total_p = np.sum(pxx) + 1e-20

        voice_band = float(np.sum(pxx[(f >= 250) & (f <= 3500)]) / total_p)
        high_band = float(np.sum(pxx[(f >= 4000) & (f <= 10000)]) / total_p)
        centroid = float(np.sum(f * pxx) / total_p)
        geo_mean = np.exp(np.mean(np.log(np.maximum(pxx, 1e-20))))
        flatness = float(geo_mean / np.maximum(np.mean(pxx), 1e-20))

        valid_idx = (f >= 40) & (f <= min(fs / 2 - 1, 10000))
        if np.any(valid_idx):
            fv, pv = f[valid_idx], pxx[valid_idx]
            dom_idx = int(np.argmax(pv))
            dom_freq = float(fv[dom_idx])
            dom_ratio = float(pv[dom_idx] / np.sum(pv))
        else:
            dom_freq, dom_ratio = 0.0, 0.0

        if crest > 5.5 and kurt > 5.0:
            return {
                "label": "IMPACT / TRANSIENT BEARING TAP",
                "confidence": int(min(98, 55 + 5 * crest)),
                "severity": severity,
                "detail": f"Impulsive contact (Crest: {crest:.1f}, Kurtosis: {kurt:.1f})."
            }

        if dom_ratio > 0.15 and 40 <= dom_freq <= 3500:
            return {
                "label": "ROTATING MACHINERY / TONAL HARMONICS",
                "confidence": int(min(96, 50 + 200 * dom_ratio)),
                "severity": severity,
                "detail": f"Shaft/Blade tonal harmonic peak at {dom_freq:.0f} Hz ({dom_ratio*100:.1f}% peak ratio)."
            }

        if voice_band > 0.40 and 250 <= centroid <= 3200 and flatness < 0.60:
            return {
                "label": "SPEECH / VOICE / ACOUSTIC MEDIA",
                "confidence": int(min(95, 50 + 40 * voice_band)),
                "severity": severity,
                "detail": f"Voice-band acoustic activity (Centroid: {centroid:.0f} Hz)."
            }

        if flatness > 0.48 and high_band > 0.20 and centroid > 2200:
            return {
                "label": "BROADBAND HISS / BEARING FRICTION",
                "confidence": int(min(92, 45 + 40 * high_band)),
                "severity": severity,
                "detail": f"High-frequency acoustic friction (Centroid: {centroid:.0f} Hz)."
            }

        return {
            "label": "UNCLASSIFIED SOUND SOURCE",
            "confidence": 35,
            "severity": severity,
            "detail": f"Mixed energy profile. RMS: {rms:.4f}, Centroid: {centroid:.0f} Hz."
        }


# =============================================================================
# MULTIMODAL DC MOTOR DIAGNOSTICS & SENSOR FUSION ENGINE
# =============================================================================
class MultimodalMotorEngine:
    """
    Advanced Multimodal Sensor Fusion Engine for DC Motors:
    Fuses:
      1. FC-03 Optical Tachometer (Exact Shaft RPM, Jitter & Pulses)
      2. ADXL345 3-Axis Accelerometer (Chassis Dynamic Vibration, ISO 10816 Severity)
      3. 4-Mic INMP441 Acoustic Array (Harmonic Orders 1x/2x/3x, Kurtosis, Crest Factor, 2D Camera)
    Capabilities:
      - Learn & Lock 'Golden Baseline' of healthy motor with Key 'B'
      - Real-time Delta Deviation Detection (catches tiny differences)
      - Composite Motor Health Score (0 - 100%)
      - Specific Fault Classification (Unbalance, Misalignment, Brush Arcing, Bearing Impact)
    """

    def __init__(self, pulses_per_rev: int = 1):
        self.pulses_per_rev = pulses_per_rev
        self.baseline_locked = False
        self.b_rpm = 0.0
        self.b_vib_g = 0.0
        self.b_rms = 0.0
        self.b_kurt = 3.0
        self.b_spec = None
        self.b_time = 0.0
        self.rpm_history: List[float] = []

        # Per-axis gravity tracking & vibration time series buffer
        self.grav_x = 0.0
        self.grav_y = 0.0
        self.grav_z = -0.82
        self.vib_history: List[float] = []
        self.vib_signed_history = collections.deque(maxlen=512)

    def learn_baseline(self, rpm: float, vib_g: float, audio: np.ndarray, fs: float):
        """Captures golden baseline signature of a known-healthy motor."""
        x = audio.mean(axis=1).astype(np.float64) if audio.ndim > 1 else audio.astype(np.float64)
        x -= np.mean(x)
        rms = float(np.sqrt(np.mean(x**2)))
        kurt = float(np.mean(x**4) / (np.mean(x**2)**2 + 1e-12))
        spec = np.abs(np.fft.rfft(x))

        self.b_rpm = max(10.0, rpm)
        self.b_vib_g = max(0.01, vib_g)
        self.b_rms = max(1e-4, rms)
        self.b_kurt = kurt
        self.b_spec = spec / (np.max(spec) + 1e-12)
        self.baseline_locked = True
        self.b_time = time.time()
        print(f"[+] Motor Baseline Learned & Locked: {self.b_rpm:.1f} RPM, {self.b_vib_g:.2f}g vib, RMS={self.b_rms:.4f}")
        return {
            "status": "LOCKED",
            "rpm": self.b_rpm,
            "vib_g": self.b_vib_g,
            "kurt": self.b_kurt,
            "rms": self.b_rms
        }

    def reset_baseline(self):
        self.baseline_locked = False
        self.b_spec = None
        print("[+] Motor Baseline Reset.")

    def analyze(
        self,
        audio: np.ndarray,
        fs: float,
        ax_g: float,
        ay_g: float,
        az_g: float,
        rpm: float,
        vib_history: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None
    ) -> Dict[str, object]:
        """Cross-fuses all 3 sensors to diagnose motor condition & compute Health Index."""
        x = audio.mean(axis=1).astype(np.float64) if audio.ndim > 1 else audio.astype(np.float64)
        x -= np.mean(x)
        rms = float(np.sqrt(np.mean(x**2)))
        peak = float(np.max(np.abs(x))) if len(x) else 0.0
        crest = peak / max(rms, 1e-12)
        kurt = float(np.mean(x**4) / (np.mean(x**2)**2 + 1e-12)) if len(x) else 3.0

        # Track RPM jitter
        self.rpm_history.append(rpm)
        if len(self.rpm_history) > 20:
            self.rpm_history.pop(0)
        rpm_jitter = float(np.std(self.rpm_history)) if len(self.rpm_history) > 3 else 0.0

        # 1. Decoupled Dynamic Vibration Magnitude (Per-Axis Gravity DC Removal)
        if vib_history is not None and len(vib_history[1]) >= 16:
            t_vib, ax_arr, ay_arr, az_arr = vib_history
            # Slow EMA tracks static orientation/gravity vector from batch average
            self.grav_x = 0.90 * self.grav_x + 0.10 * float(np.mean(ax_arr))
            self.grav_y = 0.90 * self.grav_y + 0.10 * float(np.mean(ay_arr))
            self.grav_z = 0.90 * self.grav_z + 0.10 * float(np.mean(az_arr))

            # Signed dynamic acceleration components (free of static 1g gravity offset)
            dyn_ax_arr = ax_arr - self.grav_x
            dyn_ay_arr = ay_arr - self.grav_y
            dyn_az_arr = az_arr - self.grav_z

            # Signed dynamic vibration on primary radial/transverse axis for order tracking
            signed_vib_series = dyn_ax_arr
            dyn_mag_arr = np.sqrt(dyn_ax_arr**2 + dyn_ay_arr**2 + dyn_az_arr**2)
            dyn_g = float(dyn_mag_arr[-1])

            # Effective vibration sampling rate from hardware timestamps
            dt_span = float(t_vib[-1] - t_vib[0])
            if dt_span > 1e-4 and len(t_vib) > 1:
                fs_vib = float(len(t_vib) - 1) / dt_span
            else:
                fs_vib = 86.13
        else:
            self.grav_x = 0.95 * self.grav_x + 0.05 * ax_g
            self.grav_y = 0.95 * self.grav_y + 0.05 * ay_g
            self.grav_z = 0.95 * self.grav_z + 0.05 * az_g

            dyn_ax = ax_g - self.grav_x
            dyn_ay = ay_g - self.grav_y
            dyn_az = az_g - self.grav_z

            dyn_g = math.sqrt(dyn_ax**2 + dyn_ay**2 + dyn_az**2)
            self.vib_history.append(dyn_g)
            self.vib_signed_history.append(dyn_ax)
            if len(self.vib_history) > 256:
                self.vib_history.pop(0)

            signed_vib_series = np.array(self.vib_signed_history, dtype=np.float64)
            fs_vib = 86.13

        # Vibration severity grading (Project Dynamic Vibration Severity Index inspired by ISO vibration zones)
        if dyn_g < 0.15:
            iso_severity = "ZONE A: GOOD / NOMINAL (Severity Index)"
            iso_color = "#00ffcc"
            vib_penalty = dyn_g / 0.15 * 5.0
        elif dyn_g < 0.35:
            iso_severity = "ZONE B: PERMISSIBLE (Severity Index)"
            iso_color = "#33ccff"
            vib_penalty = 5.0 + (dyn_g - 0.15) / 0.20 * 15.0
        elif dyn_g < 0.70:
            iso_severity = "ZONE C: WARNING (Severity Index)"
            iso_color = "#ffaa00"
            vib_penalty = 20.0 + (dyn_g - 0.35) / 0.35 * 30.0
        else:
            iso_severity = "ZONE D: UNACCEPTABLE (Severity Index)"
            iso_color = "#ff0055"
            vib_penalty = 50.0 + min(35.0, (dyn_g - 0.70) * 40.0)

        # 2. Rotational Order Analysis from SIGNED Dynamic Vibration (NOT Rectified Euclidean Magnitude!)
        f0 = max(1.0, rpm / 60.0)
        if len(signed_vib_series) >= 32 and rpm > 60.0:
            vib_arr = np.array(signed_vib_series, dtype=np.float64)
            vib_arr -= np.mean(vib_arr)
            n_fft = max(256, len(vib_arr) * 4)  # Zero-padding for ~0.08 Hz bin spacing
            vib_spec = np.abs(np.fft.rfft(vib_arr, n=n_fft))
            vib_freqs = np.fft.rfftfreq(n_fft, d=1.0 / fs_vib)

            def get_order_energy(target_f, bandwidth=1.5):
                mask = (vib_freqs >= max(0.5, target_f - bandwidth)) & (vib_freqs <= target_f + bandwidth)
                return float(np.sum(vib_spec[mask]**2)) if np.any(mask) else 0.0

            tot_vib_energy = float(np.sum(vib_spec**2)) + 1e-15
            e_1x = get_order_energy(f0) / tot_vib_energy
            e_2x = get_order_energy(2.0 * f0) / tot_vib_energy
            e_3x = get_order_energy(3.0 * f0) / tot_vib_energy
        else:
            e_1x = 0.0
            e_2x = 0.0
            e_3x = 0.0

        # Acoustic spectrum for high-frequency acoustic baseline comparison
        spec = np.abs(np.fft.rfft(x))

        # 3. Fault Classification Diagnostics
        fault_label = "NOMINAL / SMOOTH OPERATION"
        if dyn_g > 0.40 and e_1x > 0.12:
            fault_label = "MASS UNBALANCE (Strong 1x RPM Vibration)"
        elif dyn_g > 0.30 and e_2x > 0.10:
            fault_label = "SHAFT MISALIGNMENT / BENT (Strong 2x RPM)"
        elif crest > 4.5 and kurt > 4.5:
            fault_label = "BEARING IMPACT / BRUSH ARCING (High Kurtosis)"
        elif dyn_g > 0.35:
            fault_label = "ELEVATED MECHANICAL VIBRATION (ISO Zone C Alert)"
        elif rms > 0.035 and kurt < 2.5:
            fault_label = "COMMUTATOR NOISE / MECHANICAL DRAG"

        # 4. Golden Baseline Comparison (Detecting Tiny Differences!)
        delta_info = {
            "d_rpm": 0.0,
            "d_vib_g": 0.0,
            "d_kurt": 0.0,
            "d_rms_pct": 0.0,
            "spec_dist_pct": 0.0
        }
        delta_penalty = 0.0
        if self.baseline_locked:
            d_rpm = rpm - self.b_rpm
            d_vib = dyn_g - self.b_vib_g
            d_kurt = kurt - self.b_kurt
            d_rms_pct = ((rms - self.b_rms) / self.b_rms) * 100.0

            curr_norm_spec = spec / (np.max(spec) + 1e-12)
            n_min = min(len(curr_norm_spec), len(self.b_spec))
            spec_dist = float(np.mean(np.abs(curr_norm_spec[:n_min] - self.b_spec[:n_min]))) * 100.0

            delta_info = {
                "d_rpm": d_rpm,
                "d_vib_g": d_vib,
                "d_kurt": d_kurt,
                "d_rms_pct": d_rms_pct,
                "spec_dist_pct": spec_dist
            }
            # Sensitive penalty for any deviation from learned baseline
            delta_penalty = (
                abs(d_vib) * 35.0 +
                max(0.0, d_kurt) * 3.5 +
                min(20.0, abs(d_rms_pct) * 0.15) +
                min(15.0, spec_dist * 0.35)
            )

        # 5. Composite Motor Health Score (0 - 100%)
        total_penalty = vib_penalty + max(0.0, (kurt - 3.0) * 3.0) + delta_penalty
        health_score = max(5.0, min(100.0, 100.0 - total_penalty))

        return {
            "health_score": health_score,
            "rpm": rpm,
            "rpm_jitter": rpm_jitter,
            "f0_hz": f0,
            "dyn_vib_g": dyn_g,
            "iso_severity": iso_severity,
            "iso_color": iso_color,
            "crest_factor": crest,
            "kurtosis": kurt,
            "primary_fault": fault_label,
            "e_1x_pct": e_1x * 100.0,
            "e_2x_pct": e_2x * 100.0,
            "baseline_locked": self.baseline_locked,
            "delta": delta_info
        }


# =============================================================================
# MULTI-SOURCE AUDIO & SENSOR INGESTION ENGINES
# =============================================================================
class AudioSourceBase:
    def start(self): raise NotImplementedError
    def stop(self): raise NotImplementedError
    def get_frame(self) -> np.ndarray: raise NotImplementedError
    def get_fs(self) -> float: raise NotImplementedError
    def get_channels(self) -> int: return 4
    def get_multimodal_telemetry(self) -> Dict[str, object]: return {}
    def get_vibration_history(self, max_samples: int = 512) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        return np.array([]), np.array([]), np.array([]), np.array([])


class ESP32MultimodalSource(AudioSourceBase):
    """
    Production 4-Channel Real-Time PCM Ingestion from ESP32 on COM8.
    Streams 11,025 Hz 16-bit 4-channel audio + ADXL345 3-axis vibration + FC-03 tachometer
    at 921,600 baud using zero-copy binary DMA packets.
    """

    def __init__(self, port: str = "COM8", baudrates: Tuple[int, ...] = (921600, 115200)):
        self.port = port
        self.baudrates = baudrates
        self.fs = 11025.0
        self.channels = 4
        self.frame_len = 512

        self.buffer = np.zeros((self.frame_len, self.channels), dtype=np.float32)
        self.packets_valid = 0
        self.checksum_errors = 0
        self.dropped_packets = 0
        self.last_seq: Optional[int] = None
        self.last_packet_time = 0.0

        # Thread-safe ring buffer for decoupling accelerometer acquisition from GUI thread
        self.vib_ring_lock = threading.Lock()
        self.vib_ring_time = collections.deque(maxlen=2048)
        self.vib_ring_ax = collections.deque(maxlen=2048)
        self.vib_ring_ay = collections.deque(maxlen=2048)
        self.vib_ring_az = collections.deque(maxlen=2048)

        self.telemetry = {
            "ax_g": 0.0, "ay_g": -0.02, "az_g": -0.82,
            "rpm_pulses": 0, "estimated_rpm": 0.0,
            "firmware_mode": "CONNECTING...",
            "rms_counts": [0.0, 0.0, 0.0, 0.0],
            "peak_counts": [0, 0, 0, 0],
            "is_hardware_alive": False,
            "packets_valid": 0,
            "checksum_errors": 0,
            "dropped_packets": 0,
            "channels_active": 0,
            "last_packet_time": 0.0
        }
        self.lock = threading.Lock()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.ser: Optional[serial.Serial] = None
        self.last_pulse_count = 0
        self.last_pulse_time = time.time()

    def get_vibration_history(self, max_samples: int = 512) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self.vib_ring_lock:
            if len(self.vib_ring_ax) == 0:
                return np.array([]), np.array([]), np.array([]), np.array([])
            ax = np.array(self.vib_ring_ax, dtype=np.float32)
            ay = np.array(self.vib_ring_ay, dtype=np.float32)
            az = np.array(self.vib_ring_az, dtype=np.float32)
            t = np.array(self.vib_ring_time, dtype=np.float64)
        if len(ax) > max_samples:
            ax = ax[-max_samples:]
            ay = ay[-max_samples:]
            az = az[-max_samples:]
            t = t[-max_samples:]
        return t, ax, ay, az

    def _find_port(self) -> str:
        if not SERIAL_AVAILABLE:
            return self.port
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            hwid = (p.hwid or "").lower()
            desc = (p.description or "").lower()
            if "10c4:ea60" in hwid or "cp210" in desc:
                return p.device
            if "1a86:7523" in hwid or "ch340" in desc:
                return p.device
        for p in ports:
            if "com8" in p.device.lower():
                return p.device
        return self.port

    def start(self):
        if not SERIAL_AVAILABLE:
            return
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass

    def _worker(self):
        HEADER_FORMAT = "<BBBBHHHhhhI"
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

        while self.running:
            target_port = self._find_port()
            connected = False

            for baud in self.baudrates:
                try:
                    s = serial.Serial()
                    s.port = target_port
                    s.baudrate = baud
                    s.dtr = False
                    s.rts = False
                    s.timeout = 0.25
                    s.open()
                    time.sleep(0.1)
                    s.reset_input_buffer()
                    # Trigger continuous streaming mode on ESP32
                    s.write(b"S\n")
                    s.flush()
                    connected = True
                    self.ser = s

                    with self.lock:
                        self.telemetry["firmware_mode"] = f"CONNECTED ({target_port} @ {baud})"
                    break
                except Exception:
                    continue

            if not connected:
                with self.lock:
                    self.telemetry["firmware_mode"] = "STANDBY (ESP32 USB NOT DETECTED)"
                    self.telemetry["is_hardware_alive"] = False
                time.sleep(1.0)
                continue

            buf = bytearray()
            last_rx_time = time.time()

            while self.running and self.ser and self.ser.is_open:
                try:
                    n = self.ser.in_waiting
                    if n > 0:
                        chunk = self.ser.read(n)
                        buf.extend(chunk)
                        last_rx_time = time.time()

                        while len(buf) >= HEADER_SIZE:
                            sync_idx = -1
                            for i in range(len(buf) - 1):
                                if buf[i] == 0xAA and buf[i+1] == 0x55:
                                    sync_idx = i
                                    break

                            if sync_idx == -1:
                                buf.clear()
                                break

                            if sync_idx > 0:
                                del buf[:sync_idx]

                            if len(buf) < HEADER_SIZE:
                                break

                            sync1, sync2, pkt_type, channels, seq, fs, block_frames, ax_mg, ay_mg, az_mg, pulses = struct.unpack_from(
                                HEADER_FORMAT, buf, 0
                            )

                            payload_len = block_frames * channels * 2
                            total_pkt_len = HEADER_SIZE + payload_len + 2
                            if len(buf) < total_pkt_len:
                                break

                            # 16-bit XOR Checksum Validation
                            expected_chk = struct.unpack_from("<H", buf, HEADER_SIZE + payload_len)[0]
                            hdr_bytes = bytes(buf[0 : HEADER_SIZE])
                            payload_bytes = bytes(buf[HEADER_SIZE : HEADER_SIZE + payload_len])

                            chk = 0x55AA
                            for i, b in enumerate(hdr_bytes):
                                chk ^= ((b << (i % 8)) & 0xFFFF)
                            for i, b in enumerate(payload_bytes):
                                chk ^= ((b << (i % 8)) & 0xFFFF)

                            if (chk & 0xFFFF) != (expected_chk & 0xFFFF):
                                self.checksum_errors += 1
                                # Discard current sync byte to search for next valid frame
                                del buf[:2]
                                continue

                            # Packet Valid! Update sequence and tracking
                            self.packets_valid += 1
                            if self.last_seq is not None:
                                expected_seq = (self.last_seq + 1) & 0xFFFF
                                if seq != expected_seq:
                                    drop = (seq - expected_seq) & 0xFFFF
                                    if drop < 500:
                                        self.dropped_packets += drop
                            self.last_seq = seq
                            self.last_packet_time = time.time()

                            # Buffer accelerometer sample into decoupled thread-safe ring buffer
                            t_pkt = self.last_packet_time
                            with self.vib_ring_lock:
                                self.vib_ring_time.append(t_pkt)
                                self.vib_ring_ax.append(float(ax_mg) / 1000.0)
                                self.vib_ring_ay.append(float(ay_mg) / 1000.0)
                                self.vib_ring_az.append(float(az_mg) / 1000.0)

                            raw_payload = payload_bytes
                            samples = np.frombuffer(raw_payload, dtype=np.int16).reshape((block_frames, channels)).astype(np.float32) / 32768.0

                            rms_ch = [float(np.sqrt(np.mean(samples[:, i].astype(np.float64)**2))) for i in range(channels)]
                            active_channels = sum(1 for r in rms_ch if r > 0.0003)

                            with self.lock:
                                self.buffer = np.roll(self.buffer, -block_frames, axis=0)
                                self.buffer[-block_frames:, :] = samples
                                self.fs = float(fs)

                                # Update tachometer speed
                                now = time.time()
                                dt = now - self.last_pulse_time
                                if dt >= 0.5:
                                    dp = pulses - self.last_pulse_count
                                    if dp >= 0 and dt > 0:
                                        self.telemetry["estimated_rpm"] = (dp / dt) * 60.0
                                    self.last_pulse_count = pulses
                                    self.last_pulse_time = now

                                self.telemetry.update({
                                    "ax_g": ax_mg / 1000.0,
                                    "ay_g": ay_mg / 1000.0,
                                    "az_g": az_mg / 1000.0,
                                    "rpm_pulses": pulses,
                                    "firmware_mode": f"4-CH REAL-TIME PCM DMA STREAM ({target_port} @ {self.ser.baudrate})",
                                    "is_hardware_alive": True,
                                    "packets_valid": self.packets_valid,
                                    "checksum_errors": self.checksum_errors,
                                    "dropped_packets": self.dropped_packets,
                                    "channels_active": active_channels,
                                    "last_packet_time": self.last_packet_time,
                                    "rms_counts": [float(r * 32768.0) for r in rms_ch],
                                    "peak_counts": [int(np.max(np.abs(samples[:, i])) * 32768.0) for i in range(channels)]
                                })

                            del buf[:total_pkt_len]

                    else:
                        if time.time() - last_rx_time > 3.0:
                            self.ser.write(b"S\n")
                            last_rx_time = time.time()
                        time.sleep(0.005)

                except Exception:
                    break

            try:
                if self.ser: self.ser.close()
            except Exception: pass
            time.sleep(0.5)

    def get_frame(self) -> np.ndarray:
        with self.lock:
            if not self.telemetry["is_hardware_alive"]:
                # Simulation fallback when hardware unpowered
                t = np.arange(self.frame_len) / self.fs + (time.time() % 10.0)
                angle_deg = 35.0 * np.sin(2 * np.pi * 0.25 * time.time())
                angle_rad = np.deg2rad(angle_deg)
                d_pitch = 0.02032
                mic_x = np.array([-1.5, -0.5, 0.5, 1.5]) * d_pitch
                sim_buf = np.zeros((self.frame_len, 4), dtype=np.float32)
                base_sig = 0.6 * np.sin(2 * np.pi * 1250.0 * t) + 0.04 * np.random.randn(self.frame_len)
                for i in range(4):
                    delay_s = (mic_x[i] * np.sin(angle_rad)) / SPEED_OF_SOUND
                    shift = int(round(delay_s * self.fs))
                    sim_buf[:, i] = np.roll(base_sig, shift)
                return sim_buf
            return self.buffer.copy()

    def get_fs(self) -> float:
        return self.fs

    def get_channels(self) -> int:
        return self.channels

    def get_multimodal_telemetry(self) -> Dict[str, object]:
        with self.lock:
            return self.telemetry.copy()


class Synthetic4ChSource(AudioSourceBase):
    """Simulates a 4-channel orbiting acoustic emitter with 3-axis vibration and shaft tachometer."""

    def __init__(self, fs: float = 16000.0):
        self.fs = fs
        self.channels = 4
        self.frame_len = 512
        self.t_phase = 0.0
        self.pulses = 120

    def start(self): pass
    def stop(self): pass

    def get_frame(self) -> np.ndarray:
        t = np.arange(self.frame_len) / self.fs + self.t_phase
        self.t_phase += self.frame_len / self.fs

        # Move source sinusoidally across [-40 deg, +40 deg]
        angle_deg = 35.0 * np.sin(2.0 * np.pi * 0.28 * self.t_phase)
        angle_rad = np.deg2rad(angle_deg)
        d_pitch = 0.02032
        mic_x = np.array([-1.5, -0.5, +0.5, +1.5]) * d_pitch

        # Multi-harmonic rotating machinery audio signature
        base = (
            0.60 * np.sin(2 * np.pi * 1200.0 * t)
            + 0.25 * np.sin(2 * np.pi * 2400.0 * t)
            + 0.10 * np.sin(2 * np.pi * 3600.0 * t)
            + 0.03 * np.random.randn(self.frame_len)
        ).astype(np.float32)

        out = np.zeros((self.frame_len, 4), dtype=np.float32)
        for i in range(4):
            delay_s = (mic_x[i] * np.sin(angle_rad)) / SPEED_OF_SOUND
            delay_samp = int(round(delay_s * self.fs))
            out[:, i] = np.roll(base, delay_samp)

        self.pulses += int(np.random.randint(1, 4))
        return out

    def get_fs(self) -> float: return self.fs
    def get_channels(self) -> int: return self.channels
    def get_multimodal_telemetry(self) -> Dict[str, object]:
        vib_x = 0.12 * math.sin(2.0 * math.pi * 18.0 * self.t_phase)
        vib_y = 0.15 * math.cos(2.0 * math.pi * 18.0 * self.t_phase)
        vib_z = -0.82 + 0.08 * math.sin(2.0 * math.pi * 36.0 * self.t_phase)
        rpm = 1080.0 + 20.0 * math.sin(self.t_phase)

        rms_counts = [
            float(np.sqrt(np.mean(self.get_frame()[:, i]**2)) * 12000.0) for i in range(4)
        ]

        return {
            "ax_g": vib_x, "ay_g": vib_y, "az_g": vib_z,
            "rpm_pulses": self.pulses,
            "estimated_rpm": rpm,
            "firmware_mode": "SYNTHETIC 4-CH ARRAY SIMULATION",
            "rms_counts": rms_counts,
            "peak_counts": [int(v * 1.45) for v in rms_counts],
            "is_hardware_alive": True
        }

    def get_vibration_history(self, max_samples: int = 512) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        t_now = time.time()
        fs_sim = 400.0
        n_samples = min(max_samples, 256)
        t_arr = np.linspace(t_now - n_samples / fs_sim, t_now, n_samples)
        vib_x = (0.12 * np.sin(2.0 * np.pi * 18.0 * t_arr)).astype(np.float32)
        vib_y = (0.15 * np.cos(2.0 * np.pi * 18.0 * t_arr)).astype(np.float32)
        vib_z = (-0.82 + 0.08 * np.sin(2.0 * np.pi * 36.0 * t_arr)).astype(np.float32)
        return t_arr, vib_x, vib_y, vib_z


class LaptopAudioSource(AudioSourceBase):
    """Ingests audio from laptop internal microphone array (2-channel or 4-channel)."""

    def __init__(self):
        self.device_index, self.channels = self._find_device()
        d = sd.query_devices(self.device_index)
        self.fs = float(d.get("default_samplerate", 44100.0))
        self.frame_len = int(0.040 * self.fs)
        self.buffer = np.zeros((self.frame_len, 4), dtype=np.float32)
        self.lock = threading.Lock()
        self.stream: Optional[sd.InputStream] = None

    def _find_device(self) -> Tuple[int, int]:
        devices = sd.query_devices()
        # Look for 4-channel array first (e.g. Intel Smart Sound directsound)
        for i, d in enumerate(devices):
            name = d.get("name", "").lower()
            ch = int(d.get("max_input_channels", 0))
            if ch >= 4 and any(k in name for k in ["array", "intel", "smart sound"]):
                return i, 4
        # Fallback to stereo array
        for i, d in enumerate(devices):
            name = d.get("name", "").lower()
            ch = int(d.get("max_input_channels", 0))
            if ch >= 2 and any(k in name for k in ["array", "intel", "realtek"]):
                return i, 2
        return int(sd.default.device[0]), 2

    def _callback(self, indata, frames, time_info, status):
        with self.lock:
            n = min(frames, self.frame_len)
            if self.channels >= 4:
                self.buffer[:n, :4] = indata[:n, :4]
            else:
                self.buffer[:n, 0] = indata[:n, 0]
                self.buffer[:n, 1] = indata[:n, 1]
                self.buffer[:n, 2:] = 0.0

    def start(self):
        try:
            self.stream = sd.InputStream(
                device=self.device_index,
                samplerate=self.fs,
                channels=self.channels,
                dtype="float32",
                blocksize=self.frame_len,
                callback=self._callback
            )
            self.stream.start()
        except Exception as exc:
            print(f"[-] Sounddevice Start Warning: {exc}")

    def stop(self):
        if self.stream:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception: pass
            self.stream = None

    def get_frame(self) -> np.ndarray:
        with self.lock:
            return self.buffer.copy()

    def get_fs(self) -> float: return self.fs
    def get_channels(self) -> int: return self.channels
    def get_multimodal_telemetry(self) -> Dict[str, object]:
        return {
            "ax_g": 0.0, "ay_g": 0.0, "az_g": -1.0,
            "rpm_pulses": 0, "estimated_rpm": 0.0,
            "firmware_mode": f"LAPTOP INTERNAL MIC ({self.channels}-CH)",
            "rms_counts": [float(np.sqrt(np.mean(self.buffer[:, i]**2))*10000) for i in range(4)],
            "peak_counts": [1000, 1000, 1000, 1000],
            "is_hardware_alive": True
        }


# =============================================================================
# MASTER RADAR PRO 4-CHANNEL HUD APPLICATION
# =============================================================================
class AcousticRadarApp:
    """Master Interactive HUD with 4-Microphone Live Inputs and Multimodal Telemetry."""

    def __init__(self):
        self.active_source_type = "ESP32"
        self.active_geometry_key = "ESP32_4MIC_LINEAR"
        self.geometry = PRESETS[self.active_geometry_key]

        self.sources: Dict[str, AudioSourceBase] = {
            "ESP32": ESP32MultimodalSource(port="COM8"),
            "SYNTHETIC": Synthetic4ChSource(),
            "LAPTOP": LaptopAudioSource()
        }
        self.current_source = self.sources[self.active_source_type]
        self.current_source.start()

        self.invert_channels = False
        self.tracker = DynamicTargetTracker(alpha=0.40, gate_deg=30.0)

        # Adaptive Noise Gate calibrated for 16-bit INMP441 normalized audio
        self.noise_floor = 0.020
        self.noise_gate_rms = 0.035

        # Display State
        self.smoothed_heatmap = np.zeros((GRID_Y, GRID_X), dtype=np.float64)
        self.paused = False
        self.last_classifier_result = {
            "label": "CALIBRATING", "confidence": 0, "severity": "INITIALIZING", "detail": "Listening for acoustic sources..."
        }

        # I2S Dual-Pair Phase Calibration Delay (Seconds)
        self.calib_delay_sec = 0.0
        proj_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        calib_candidates = [
            os.path.join(proj_dir, "stage2_acoustic_localization", "evidence", "i2s_phase_calibration.json"),
            os.path.join(proj_dir, "stage4_realtime_fusion_radar", "evidence", "i2s_phase_calibration.json"),
            os.path.join(proj_dir, "evidence", "stage2", "i2s_phase_calibration.json"),
            "i2s_phase_calibration.json"
        ]
        for c_path in calib_candidates:
            if os.path.exists(c_path):
                try:
                    with open(c_path, 'r') as f:
                        c_data = json.load(f)
                        if "inter_pair_comp_delay_sec" in c_data:
                            self.calib_delay_sec = float(c_data["inter_pair_comp_delay_sec"])
                        elif "inter_pair_mean_skew_us" in c_data:
                            self.calib_delay_sec = float(c_data["inter_pair_mean_skew_us"]) * 1e-6
                        print(f"[*] Acoustic Radar: Loaded Phase Calibration from {os.path.basename(c_path)}: Delay = {self.calib_delay_sec*1e6:+.2f} μs")
                        break
                except Exception as e:
                    print(f"[!] Warning reading phase calibration from {c_path}: {e}")

        # Multimodal Motor Diagnostics Engine & Sensor Fusion
        self.motor_engine = MultimodalMotorEngine()
        self.learn_baseline_requested = False
        self.pulses_options = [1, 2, 4, 10, 20]
        self.pulses_per_rev_idx = 0
        self.pulses_per_rev = 1

        self._setup_ui()

    def _get_evidence_mode_tag(self) -> str:
        """Returns clear evidence mode tag distinguishing physical hardware from simulation."""
        if self.active_source_type == "ESP32":
            telemetry = self.current_source.get_multimodal_telemetry()
            is_alive = telemetry.get("is_hardware_alive", False)
            pkt_valid = telemetry.get("packets_valid", 0)
            chk_err = telemetry.get("checksum_errors", 0)
            ch_active = telemetry.get("channels_active", 0)
            last_pkt = telemetry.get("last_packet_time", 0.0)

            if is_alive and (time.time() - last_pkt < 1.5) and pkt_valid > 5:
                if ch_active >= 4:
                    return f"MODE: LIVE 4-MIC ULA HARDWARE (COM8, CHK VALIDATED) [{self.geometry.name}]"
                else:
                    return f"MODE: LIVE HARDWARE (COM8, {ch_active}/4 CH ACTIVE, CHK OK) [{self.geometry.name}]"
            elif is_alive and chk_err > 0:
                return f"MODE: HARDWARE WARNING ({chk_err} CHECKSUM ERRORS) [{self.geometry.name}]"
            else:
                return "MODE: ESP32 STANDBY / DISCONNECTED"
        elif self.active_source_type == "SYNTHETIC":
            return f"MODE: SYNTHETIC 4-CH ARRAY SIMULATION [{self.geometry.name}]"
        elif self.active_source_type == "LAPTOP":
            return f"MODE: LAPTOP INTERNAL MIC DEMO [{self.geometry.name}]"
        return f"MODE: {self.active_source_type}"

    def _setup_ui(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(17, 9.5), facecolor="#0a0e17")

        try:
            if hasattr(self.fig.canvas, "manager") and self.fig.canvas.manager:
                self.fig.canvas.manager.set_window_title(
                    "Spatial Multimodal Acoustic Radar Pro | 4-Channel Industrial Suite (v6)"
                )
        except Exception:
            pass

        # 2 rows x 3 columns layout
        gs = gridspec.GridSpec(
            2, 3,
            width_ratios=[1.25, 0.95, 0.95],
            height_ratios=[1.0, 1.0],
            hspace=0.28, wspace=0.22,
            left=0.04, right=0.98, top=0.93, bottom=0.05
        )

        # ---------------------------------------------------------------------
        # 1. 2D Spatial Heatmap / Acoustic Camera (Left Panel: gs[:, 0])
        # ---------------------------------------------------------------------
        self.ax_map = self.fig.add_subplot(gs[:, 0], facecolor="#101726")
        self.im_heat = self.ax_map.imshow(
            self.smoothed_heatmap,
            origin="lower",
            extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
            aspect="auto",
            vmin=0.0, vmax=1.0,
            cmap="inferno",
            interpolation="bicubic"
        )

        self.cb = self.fig.colorbar(self.im_heat, ax=self.ax_map, fraction=0.046, pad=0.04)
        self.cb.set_label("Live Acoustic Intensity (SRP-PHAT Relative Power)", color="#b8c2d1", fontsize=10)
        self.cb.ax.tick_params(colors="#b8c2d1", labelsize=9)

        # Dynamic Side Intensity Indicators (Left & Right Flank VU Rails)
        self.left_gauge_bg, = self.ax_map.plot(
            [-1.88, -1.88], [0.45, 3.25],
            color="#141f33", linewidth=11, solid_capstyle="round", zorder=7
        )
        self.left_gauge_fill, = self.ax_map.plot(
            [-1.88, -1.88], [0.45, 0.45],
            color="#00ffcc", linewidth=11, solid_capstyle="round", zorder=8
        )
        self.left_gauge_text = self.ax_map.text(
            -1.88, 3.32, "◄ LEFT\n0%",
            color="#00ffcc", fontsize=8.5, fontweight="bold", ha="center", va="bottom", zorder=9
        )

        self.right_gauge_bg, = self.ax_map.plot(
            [1.88, 1.88], [0.45, 3.25],
            color="#141f33", linewidth=11, solid_capstyle="round", zorder=7
        )
        self.right_gauge_fill, = self.ax_map.plot(
            [1.88, 1.88], [0.45, 0.45],
            color="#00ffcc", linewidth=11, solid_capstyle="round", zorder=8
        )
        self.right_gauge_text = self.ax_map.text(
            1.88, 3.32, "RIGHT ►\n0%",
            color="#00ffcc", fontsize=8.5, fontweight="bold", ha="center", va="bottom", zorder=9
        )

        # Incoming Sound Sector HUD Banner
        self.sector_banner_text = self.ax_map.text(
            0.0, 3.35, "• SCANNING ACOUSTIC FIELD •",
            color="#8da2c0", fontsize=9.0, fontweight="bold", ha="center", va="bottom", zorder=9
        )

        # Array Microphones Scatter with individual labels
        self.scatter_mics = self.ax_map.scatter(
            self.geometry.mic_coords[:, 0], self.geometry.mic_coords[:, 1],
            marker="^", color="#00ffcc", s=130, edgecolor="#ffffff", linewidth=1.5,
            label="4-Mic Array (INMP441)", zorder=5
        )

        # Microphone labels (M1, M2, M3, M4)
        self.mic_labels = []
        for i in range(self.geometry.num_mics):
            txt = self.ax_map.text(
                self.geometry.mic_coords[i, 0], self.geometry.mic_coords[i, 1] - 0.12,
                f"M{i+1}", color="#00ffcc", fontsize=9, fontweight="bold",
                ha="center", va="top", zorder=6
            )
            self.mic_labels.append(txt)

        # Estimated Peak Target Crosshair
        self.marker_source, = self.ax_map.plot(
            [0], [1.25],
            marker="o", markersize=16, markerfacecolor="none",
            markeredgecolor="#00ffcc", markeredgewidth=3.0,
            label="Locked Target", zorder=6
        )
        self.target_line, = self.ax_map.plot(
            [0, 0], [0, 1.25],
            color="#00ffcc", linestyle=":", linewidth=2.0, zorder=5
        )

        # Distance Range Rings & Azimuth Rays
        for r_m in [1.0, 2.0, 3.0]:
            theta_arc = np.linspace(-np.pi / 2, np.pi / 2, 80)
            self.ax_map.plot(
                r_m * np.sin(theta_arc), r_m * np.cos(theta_arc),
                color="#24334d", linestyle="--", linewidth=1.0, zorder=2
            )
            self.ax_map.text(
                0.05, r_m + 0.05, f"{r_m:.0f}m",
                color="#455b80", fontsize=9, family="monospace"
            )

        for ang_deg in [-60, -30, 0, 30, 60]:
            ang_rad = np.deg2rad(ang_deg)
            self.ax_map.plot(
                [0, 3.4 * np.sin(ang_rad)], [0, 3.4 * np.cos(ang_rad)],
                color="#1c283d", linestyle=":", linewidth=1.0, zorder=2
            )

        self.ax_map.set_xlim(X_MIN, X_MAX)
        self.ax_map.set_ylim(Y_MIN, Y_MAX)
        self.ax_map.set_xlabel("Left / Right Offset (m)", color="#b8c2d1", fontsize=11)
        self.ax_map.set_ylabel("Range in Front of Array (m)", color="#b8c2d1", fontsize=11)
        self.ax_map.set_title("2D ACOUSTIC CAMERA (DOA θ-FOCUS RADIAL BACKPROJECTION)", color="#ffffff", fontsize=11, pad=10, weight="bold")
        self.ax_map.tick_params(colors="#b8c2d1")
        self.ax_map.legend(loc="upper right", facecolor="#101726", edgecolor="#2a3b5c", fontsize=9)
        self.ax_map.grid(alpha=0.15, color="#34496d")

        # ---------------------------------------------------------------------
        # 2. Polar Radar Sweep (Top Middle: gs[0, 1])
        # ---------------------------------------------------------------------
        self.ax_radar = self.fig.add_subplot(gs[0, 1], projection="polar", facecolor="#101726")
        self.radar_angles = np.deg2rad(np.linspace(-90, 90, 181))
        self.radar_line, = self.ax_radar.plot(self.radar_angles, np.zeros_like(self.radar_angles), color="#00ffcc", linewidth=2.0)
        
        self.meas_needle, = self.ax_radar.plot([0, 0], [0, 0.85], color="#ff7700", linewidth=2.0, linestyle="--", label="Instantaneous")
        self.radar_needle, = self.ax_radar.plot([0, 0], [0, 1.0], color="#00ffcc", linewidth=3.5, label="Filtered Track")
        
        self.ax_radar.set_theta_zero_location("N")
        self.ax_radar.set_theta_direction(-1)
        self.ax_radar.set_thetamin(-90)
        self.ax_radar.set_thetamax(90)
        self.ax_radar.set_title("POLAR BEARING RADAR (180°)", color="#ffffff", fontsize=11, weight="bold", pad=12)
        self.ax_radar.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_radar.grid(color="#24334d", alpha=0.5)
        self.ax_radar.legend(loc="lower right", facecolor="#101726", edgecolor="#2a3b5c", fontsize=7)

        # ---------------------------------------------------------------------
        # 3. SRP-PHAT Delay Profile & Angular Spectrum (Top Right: gs[0, 2])
        # ---------------------------------------------------------------------
        self.ax_srp = self.fig.add_subplot(gs[0, 2], facecolor="#101726")
        self.srp_line, = self.ax_srp.plot([], [], color="#00ffcc", linewidth=2.0, label="SRP-PHAT (6 Pairs)")
        self.srp_peak_marker, = self.ax_srp.plot([], [], color="#ff0055", marker="o", markersize=8, label="Estimated DOA")
        self.ax_srp.set_xlim(-90, 90)
        self.ax_srp.set_ylim(-0.05, 1.15)
        self.ax_srp.set_title("MULTI-PAIR STEERED RESPONSE POWER", color="#ffffff", fontsize=11, weight="bold")
        self.ax_srp.set_xlabel("Bearing Angle (deg)", color="#b8c2d1", fontsize=9)
        self.ax_srp.set_ylabel("Normalized SRP Power", color="#b8c2d1", fontsize=9)
        self.ax_srp.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_srp.grid(alpha=0.15, color="#34496d")
        self.ax_srp.legend(loc="upper right", facecolor="#101726", edgecolor="#2a3b5c", fontsize=7)

        # ---------------------------------------------------------------------
        # 4. ALL 4 MICROPHONE LIVE INPUTS (Bottom Middle: gs[1, 1])
        # ---------------------------------------------------------------------
        self.ax_mics = self.fig.add_subplot(gs[1, 1], facecolor="#0e1420")
        self.mic_colors = ["#00ffcc", "#ffaa00", "#ff0077", "#33ccff"] # M1, M2, M3, M4
        self.mic_names = [
            "M1: Pair A Left  (GPIO 19, Row 20)",
            "M2: Pair A Right (GPIO 19, Row 28)",
            "M3: Pair B Left  (GPIO 23, Row 36)",
            "M4: Pair B Right (GPIO 23, Row 44)"
        ]

        # 4 Stacked oscilloscope lines
        self.wave_lines = []
        for i in range(4):
            line, = self.ax_mics.plot([], [], color=self.mic_colors[i], linewidth=1.2, label=f"Mic {i+1}")
            self.wave_lines.append(line)

        self.ax_mics.set_xlim(0, 512)
        self.ax_mics.set_ylim(-4.2, 4.2)
        self.ax_mics.set_title("LIVE 4-CHANNEL MICROPHONE ARRAY INPUTS", color="#ffffff", fontsize=11, weight="bold")
        self.ax_mics.set_xlabel("Audio Frame Samples (DMA Block)", color="#b8c2d1", fontsize=9)
        self.ax_mics.set_ylabel("Stacked Channels [M1..M4]", color="#b8c2d1", fontsize=9)
        self.ax_mics.set_yticks([-3.0, -1.0, 1.0, 3.0])
        self.ax_mics.set_yticklabels(["M4 (B-R)", "M3 (B-L)", "M2 (A-R)", "M1 (A-L)"], fontsize=8, color="#b8c2d1")
        self.ax_mics.grid(alpha=0.15, color="#34496d")

        # Live VU / RMS badge texts beside each channel
        self.vu_texts = []
        for i in range(4):
            y_pos = 3.0 - i * 2.0
            txt = self.ax_mics.text(
                505, y_pos + 0.35, "RMS: 0000 | PK: 0000",
                color=self.mic_colors[i], fontsize=8, family="monospace", ha="right", va="bottom",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#101726", edgecolor=self.mic_colors[i], alpha=0.8)
            )
            self.vu_texts.append(txt)

        # ---------------------------------------------------------------------
        # 5. Multimodal Telemetry & Diagnostics HUD (Bottom Right: gs[1, 2])
        # ---------------------------------------------------------------------
        self.ax_telemetry = self.fig.add_subplot(gs[1, 2], facecolor="#0e1420")
        self.ax_telemetry.axis("off")
        self.hud_text = self.ax_telemetry.text(
            0.02, 0.98, "Acquiring Multimodal Telemetry...",
            va="top", ha="left", color="#e0e6ed", fontsize=8.0, family="monospace", linespacing=1.12
        )

        # Main Title Banner
        self.title_text = self.fig.text(
            0.04, 0.968,
            f"SPATIAL MULTIMODAL ACOUSTIC RADAR | [{self._get_evidence_mode_tag()}]",
            color="#00ffcc", fontsize=11.5, weight="bold"
        )

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _on_key(self, event):
        if event.key in ("q", "escape"):
            plt.close(self.fig)
        elif event.key == " ":
            self.paused = not self.paused
        elif event.key in ("i", "I"):
            self.invert_channels = not self.invert_channels
            state = "INVERTED (M4..M1)" if self.invert_channels else "NORMAL (M1..M4)"
            print(f"[+] Channel Polarity Swapped: {state}")
        elif event.key in ("b", "B"):
            self.learn_baseline_requested = True
            print("[+] Motor Golden Baseline Capture Triggered! Learning reference signature...")
        elif event.key in ("c", "C"):
            self.motor_engine.reset_baseline()
            print("[+] Motor Golden Baseline Cleared / Reset.")
        elif event.key in ("p", "P"):
            self.pulses_per_rev_idx = (self.pulses_per_rev_idx + 1) % len(self.pulses_options)
            self.pulses_per_rev = self.pulses_options[self.pulses_per_rev_idx]
            print(f"[+] Tachometer Pulses Per Rev set to: {self.pulses_per_rev}")
        elif event.key in ("r", "R"):
            self.tracker.reset()
            print("[+] Target tracker reset to 0 deg.")
        elif event.key in ("1",):
            lap_geom = "ESP32_4MIC_LINEAR" if self.sources["LAPTOP"].get_channels() >= 4 else "LAPTOP_DUAL"
            self._switch_source("LAPTOP", lap_geom)
        elif event.key in ("2",):
            self._switch_source("ESP32", "ESP32_4MIC_LINEAR")
        elif event.key in ("3",):
            self._switch_source("SYNTHETIC", "ESP32_4MIC_LINEAR")
        elif event.key in ("m", "g", "M", "G"):
            keys = list(PRESETS.keys())
            idx = (keys.index(self.active_geometry_key) + 1) % len(keys)
            self._switch_geometry(keys[idx])
        elif event.key == "]":
            self.geometry.set_baseline(self.geometry.baseline_m + 0.005)
            self.scatter_mics.set_offsets(self.geometry.mic_coords)
            self._update_mic_label_positions()
            print(f"[+] Baseline pitch: {self.geometry.baseline_m * 1000:.1f} mm")
        elif event.key == "[":
            self.geometry.set_baseline(self.geometry.baseline_m - 0.005)
            self.scatter_mics.set_offsets(self.geometry.mic_coords)
            self._update_mic_label_positions()
            print(f"[+] Baseline pitch: {self.geometry.baseline_m * 1000:.1f} mm")

    def _update_mic_label_positions(self):
        for i in range(len(self.mic_labels)):
            if i < self.geometry.num_mics:
                self.mic_labels[i].set_position((self.geometry.mic_coords[i, 0], self.geometry.mic_coords[i, 1] - 0.12))
                self.mic_labels[i].set_visible(True)
            else:
                self.mic_labels[i].set_visible(False)

    def _switch_source(self, source_type: str, geometry_key: str):
        if source_type == self.active_source_type:
            return
        print(f"[+] Switching Audio Source to: {source_type}")
        self.current_source.stop()
        self.active_source_type = source_type
        self.current_source = self.sources[source_type]
        self.current_source.start()
        self._switch_geometry(geometry_key)

    def _switch_geometry(self, geometry_key: str):
        self.active_geometry_key = geometry_key
        self.geometry = PRESETS[geometry_key]
        self.smoothed_heatmap = np.zeros((GRID_Y, GRID_X), dtype=np.float64)
        self.scatter_mics.set_offsets(self.geometry.mic_coords)
        self._update_mic_label_positions()
        self.title_text.set_text(
            f"SPATIAL MULTIMODAL ACOUSTIC RADAR | SOURCE: {self.active_source_type} ({self.geometry.name})"
        )
        self.fig.canvas.draw_idle()

    def update(self, _frame):
        if self.paused:
            return [
                self.im_heat, self.marker_source, self.target_line,
                self.radar_line, self.radar_needle, self.meas_needle,
                self.srp_line, self.srp_peak_marker, self.hud_text,
                self.left_gauge_bg, self.left_gauge_fill, self.left_gauge_text,
                self.right_gauge_bg, self.right_gauge_fill, self.right_gauge_text,
                self.sector_banner_text
            ] + self.wave_lines + self.vu_texts

        try:
            raw_frame = self.current_source.get_frame()
            fs = self.current_source.get_fs()
            channels = raw_frame.shape[1] if raw_frame.ndim > 1 else 1

            if self.invert_channels:
                raw_frame = np.fliplr(raw_frame)

            # Apply I2S phase calibration compensation to Pair B (channels 2 & 3)
            if abs(self.calib_delay_sec) > 1e-12 and raw_frame.shape[1] >= 4:
                raw_frame = apply_pair_phase_compensation(raw_frame, fs, self.calib_delay_sec)

            # 1. Update 4-Channel Live Waveforms & VU Badges
            n_samples = min(raw_frame.shape[0], 512)
            x_axis = np.arange(n_samples)
            rms_per_ch = []
            pk_per_ch = []

            for i in range(4):
                if i < channels:
                    sig_ch = raw_frame[:n_samples, i]
                    sig_ac = sig_ch - np.mean(sig_ch)
                    r_val = float(np.sqrt(np.mean(sig_ac ** 2)))
                    p_val = float(np.max(np.abs(sig_ac)))
                    rms_per_ch.append(r_val)
                    pk_per_ch.append(p_val)

                    # Offset vertically for clean stacking: M1 at +3, M2 at +1, M3 at -1, M4 at -3
                    y_offset = 3.0 - i * 2.0
                    scaled_wave = np.clip(sig_ac * 1.8, -0.85, 0.85) + y_offset
                    self.wave_lines[i].set_data(x_axis, scaled_wave)

                    dbfs = 20.0 * math.log10(max(r_val, 1e-5))
                    status_str = "ACTIVE" if r_val > 0.0015 else "QUIET "
                    self.vu_texts[i].set_text(f"M{i+1}: RMS={r_val:.4f} ({dbfs:+.1f}dB) | {status_str}")
                else:
                    self.wave_lines[i].set_data([], [])
                    self.vu_texts[i].set_text(f"M{i+1}: [DISABLED]")

            avg_rms = float(np.mean(rms_per_ch)) if rms_per_ch else 0.0
            # Track ambient floor dynamically during quiet intervals
            if avg_rms < self.noise_floor * 1.6:
                self.noise_floor = 0.95 * self.noise_floor + 0.05 * avg_rms
            else:
                self.noise_floor = 0.995 * self.noise_floor + 0.005 * avg_rms

            # Sound is active when average RMS rises above ambient floor by at least 15% AND above 0.0020
            is_active_sound = bool(avg_rms >= max(0.0020, self.noise_floor * 1.15))

            # 2. Multi-Pair Steered Response Power & 2D Backprojection
            best_bearing, composite_prom, srp_angles, srp_norm, raw_2d_heatmap, side_metrics = (
                SRPPhatEngine.compute_srp_frame(raw_frame, fs, self.geometry)
            )

            # 3. Dynamic Target Tracker
            target_angle, target_range, confidence, is_locked = self.tracker.update(
                best_bearing, composite_prom, is_active_sound, nominal_range=1.25
            )

            target_rad = math.radians(target_angle)
            target_x = target_range * math.sin(target_rad)
            target_y = target_range * math.cos(target_rad)

            # 4. True 2D Acoustic Camera Thermal Heatmap (Real Backprojected Energy)
            acoustic_heat = SRPPhatEngine.compute_true_2d_acoustic_heatmap(
                raw_2d_heatmap, target_range, avg_rms, is_locked, self.geometry
            )

            if is_locked or is_active_sound:
                self.smoothed_heatmap = (
                    HEATMAP_SMOOTH_ALPHA * self.smoothed_heatmap
                    + (1.0 - HEATMAP_SMOOTH_ALPHA) * acoustic_heat
                )
            else:
                self.smoothed_heatmap *= 0.85

            self.im_heat.set_data(self.smoothed_heatmap)

            marker_color = "#00ffcc" if is_locked else "#ffaa00"
            self.marker_source.set_data([target_x], [target_y])
            self.marker_source.set_markeredgecolor(marker_color)
            self.target_line.set_data([0, target_x], [0, target_y])
            self.target_line.set_color(marker_color)

            # 4b. Dynamic Sound Intensity Side Indicators (Left / Center / Right)
            pct_l = float(side_metrics.get("pct_left", 33.3))
            pct_c = float(side_metrics.get("pct_center", 33.3))
            pct_r = float(side_metrics.get("pct_right", 33.3))
            dom_s = str(side_metrics.get("dom_side", "CENTER"))

            # Dynamic logarithmic/square-root VU gain
            rms_rel = max(0.0, avg_rms - self.noise_floor) / max(0.015, self.noise_floor * 3.0)
            rms_scale = min(1.0, math.sqrt(min(1.0, max(0.05, rms_rel))))
            intensity_l = float(min(1.0, (pct_l / 100.0) * rms_scale * 1.35))
            intensity_r = float(min(1.0, (pct_r / 100.0) * rms_scale * 1.35))
            intensity_c = float(min(1.0, (pct_c / 100.0) * rms_scale * 1.35))

            y_bot, y_top = 0.45, 3.25
            y_span = y_top - y_bot

            # Left VU rail
            h_left = y_bot + y_span * intensity_l
            self.left_gauge_fill.set_data([-1.88, -1.88], [y_bot, h_left])
            c_left = "#ff0055" if (pct_l >= 48.0 and is_active_sound) else ("#ffaa00" if pct_l >= 36.0 else "#00ffcc")
            self.left_gauge_fill.set_color(c_left)
            self.left_gauge_text.set_color(c_left)
            self.left_gauge_text.set_text(f"◄ LEFT\n{pct_l:.0f}%")

            # Right VU rail
            h_right = y_bot + y_span * intensity_r
            self.right_gauge_fill.set_data([1.88, 1.88], [y_bot, h_right])
            c_right = "#ff0055" if (pct_r >= 48.0 and is_active_sound) else ("#ffaa00" if pct_r >= 36.0 else "#00ffcc")
            self.right_gauge_fill.set_color(c_right)
            self.right_gauge_text.set_color(c_right)
            self.right_gauge_text.set_text(f"RIGHT ►\n{pct_r:.0f}%")

            # Incoming Sector HUD Banner
            if not is_active_sound:
                self.sector_banner_text.set_text("• AMBIENT NOISE FLOOR (QUIET) •")
                self.sector_banner_text.set_color("#6b7d99")
            elif dom_s == "LEFT":
                self.sector_banner_text.set_text(f"◄◄ INCOMING SOUND: LEFT FLANK ({target_angle:+.1f}°) | {pct_l:.0f}% INTENSITY")
                self.sector_banner_text.set_color("#ff0055" if pct_l >= 48.0 else "#ffaa00")
            elif dom_s == "RIGHT":
                self.sector_banner_text.set_text(f"INCOMING SOUND: RIGHT FLANK ({target_angle:+.1f}°) | {pct_r:.0f}% INTENSITY ►►")
                self.sector_banner_text.set_color("#ff0055" if pct_r >= 48.0 else "#ffaa00")
            else:
                self.sector_banner_text.set_text(f"▲ INCOMING SOUND: FRONT CENTER ({target_angle:+.1f}°) | {pct_c:.0f}% INTENSITY ▲")
                self.sector_banner_text.set_color("#00ffcc")

            # 5. Polar Radar Sweep
            polar_angles_deg = np.linspace(-90, 90, 181)
            target_conf = confidence if is_locked else confidence * 0.40
            ang_diff = polar_angles_deg - target_angle
            radar_profile = np.exp(-0.5 * (ang_diff / 8.0) ** 2) * (target_conf / 100.0)
            self.radar_line.set_data(self.radar_angles, radar_profile)

            meas_rad = math.radians(best_bearing)
            self.meas_needle.set_data([meas_rad, meas_rad], [0, 0.85])
            self.radar_needle.set_data([target_rad, target_rad], [0, 1.0])
            self.radar_needle.set_color(marker_color)

            # 6. SRP-PHAT Angular Profile Plot
            if len(srp_norm) > 0:
                self.srp_line.set_data(srp_angles, srp_norm)
                peak_idx = int(np.argmin(np.abs(srp_angles - best_bearing)))
                self.srp_peak_marker.set_data([best_bearing], [srp_norm[peak_idx]])

            # 7. Multimodal DC Motor Diagnostics & Sensor Fusion
            telemetry = self.current_source.get_multimodal_telemetry()
            ax_g = float(telemetry.get("ax_g", 0.0))
            ay_g = float(telemetry.get("ay_g", 0.0))
            az_g = float(telemetry.get("az_g", -0.82))
            total_g = math.sqrt(ax_g**2 + ay_g**2 + az_g**2)
            dyn_vib_g = max(0.0, total_g - 1.0)
            pulses = int(telemetry.get("rpm_pulses", 0))
            raw_rpm = float(telemetry.get("estimated_rpm", 0.0))
            rpm = raw_rpm / max(1, self.pulses_per_rev)
            fw_mode = str(telemetry.get("firmware_mode", "N/A"))
            hw_alive = bool(telemetry.get("is_hardware_alive", False))

            if self.learn_baseline_requested:
                self.motor_engine.learn_baseline(rpm, dyn_vib_g, raw_frame, fs)
                self.learn_baseline_requested = False

            vib_hist = self.current_source.get_vibration_history(max_samples=512)
            motor_diag = self.motor_engine.analyze(raw_frame, fs, ax_g, ay_g, az_g, rpm, vib_history=vib_hist)

            dir_str = "CENTER" if abs(target_angle) < 6.0 else ("RIGHT" if target_angle > 0 else "LEFT")
            lock_badge = f"* LOCKED ({dir_str})" if is_locked else "SEARCHING / SCANNING..."
            invert_badge = "INVERTED (Key 'I')" if self.invert_channels else "NORMAL (Key 'I')"
            hw_badge = "STREAMING (COM8)" if hw_alive else "STANDBY / SIMULATION"

            def make_vu_bar(pct, width=14):
                filled = int(round((pct / 100.0) * width))
                filled = max(0, min(width, filled))
                return "█" * filled + "░" * (width - filled)

            db_l = 20.0 * math.log10(max(intensity_l, 1e-4))
            db_c = 20.0 * math.log10(max(intensity_c, 1e-4))
            db_r = 20.0 * math.log10(max(intensity_r, 1e-4))

            dom_tag = {
                "LEFT": "◄◄ LEFT FLANK",
                "CENTER": "▲ FRONT CENTER",
                "RIGHT": "RIGHT FLANK ►►"
            }.get(dom_s, "") if is_active_sound else "QUIET IDLE"

            # Motor Health Index Display
            h_score = float(motor_diag["health_score"])
            h_bar = make_vu_bar(h_score, width=16)
            if h_score >= 85:
                h_status = "EXCELLENT (NOMINAL)"
            elif h_score >= 65:
                h_status = "PERMISSIBLE (MILD WEAR)"
            elif h_score >= 40:
                h_status = "WARNING (DEGRADED)"
            else:
                h_status = "CRITICAL FAULT / STOP"

            # Baseline Status Line
            if motor_diag["baseline_locked"]:
                d = motor_diag["delta"]
                base_str = f"LOCKED | ΔVib:{d['d_vib_g']:+.2f}g | ΔRPM:{d['d_rpm']:+4.0f} | ΔKurt:{d['d_kurt']:+.1f} | ΔRMS:{d['d_rms_pct']:+3.0f}%"
            else:
                base_str = "UNLOCKED  [Press 'B' to Learn Reference Baseline]"

            mode_tag = self._get_evidence_mode_tag()
            self.title_text.set_text(f"SPATIAL MULTIMODAL ACOUSTIC RADAR | [{mode_tag}]")

            hud_lines = [
                "==========================================================================================",
                f" [EVIDENCE MODE]       [{mode_tag}]",
                f" [MOTOR HEALTH INDEX]  [{h_bar}] {h_score:4.1f}% | {h_status}",
                f" [DIAGNOSED FAULT]     {motor_diag['primary_fault']}",
                f" [GOLDEN BASELINE]     {base_str}",
                "------------------------------------------------------------------------------------------",
                f" [S1: FC-03 TACH]     Speed: {rpm:6.1f} RPM (f0 = {motor_diag['f0_hz']:4.1f} Hz) | Jitter: ±{motor_diag['rpm_jitter']:3.1f} RPM",
                f"                      Total: {pulses:6d} Pulses | PPR: {self.pulses_per_rev} (Key 'P' to cycle)",
                f" [S2: ADXL345 3-AXIS] Dyn Vib: {dyn_vib_g:4.3f}g (Gravity-Decoupled) | Raw: [X:{ax_g:+5.2f}, Y:{ay_g:+5.2f}, Z:{az_g:+5.2f}]",
                f"                      Severity Index: {motor_diag['iso_severity']}",
                f" [S3: ACOUSTIC EMIT]  Kurtosis: {motor_diag['kurtosis']:4.2f} (Norm=3.0) | Crest Factor: {motor_diag['crest_factor']:4.1f}",
                f" [S4: VIB HARMONICS]  1x(Unbalance)={motor_diag['e_1x_pct']:4.1f}% | 2x(Misalignment)={motor_diag['e_2x_pct']:4.1f}%",
                "------------------------------------------------------------------------------------------",
                f" [DIRECTIONAL RADAR]  Target: {target_angle:+6.1f}° [{dir_str:<6}] | Focus: {dom_tag} | Range: Unconstrained (DOA θ-Ray)",
                f"   LEFT FLANK  : [{make_vu_bar(pct_l)}] {pct_l:4.1f}% ({db_l:+5.1f} dBFS) {'◄-- ACTIVE' if dom_s=='LEFT' and is_active_sound else ''}",
                f"   FRONT CENTER: [{make_vu_bar(pct_c)}] {pct_c:4.1f}% ({db_c:+5.1f} dBFS) {'▲-- ACTIVE' if dom_s=='CENTER' and is_active_sound else ''}",
                f"   RIGHT FLANK : [{make_vu_bar(pct_r)}] {pct_r:4.1f}% ({db_r:+5.1f} dBFS) {'ACTIVE --►' if dom_s=='RIGHT' and is_active_sound else ''}",
                f" [SERIAL INTEGRITY]   Valid Pkts: {telemetry.get('packets_valid', 0)} | Drops: {telemetry.get('dropped_packets', 0)} | ChkErr: {telemetry.get('checksum_errors', 0)}",
                f" [ARRAY APERTURE]     Spacing: {self.geometry.baseline_m * 1000.0:4.1f} mm | Span: {self.geometry.baseline_m * 3000.0:4.1f} mm | [{invert_badge}]",
                f" [HARDWARE LINK]      Status: {hw_badge} | Mode: {fw_mode}",
                "==========================================================================================",
                " [HOTKEYS] 'B': Learn Baseline  |  'C': Clear Baseline  |  'P': Cycle PPR ({self.pulses_per_rev})",
                "           'I': Invert Polarity  |  'R': Reset Tracker  |  '1': Laptop  |  '2': ESP32  |  '3': Sim"
            ]
            self.hud_text.set_text("\n".join(hud_lines))

        except Exception as e:
            if not hasattr(self, "_last_err_time") or time.time() - self._last_err_time > 3.0:
                print(f"[!] Warning in GUI update loop: {e}", file=sys.stderr)
                self._last_err_time = time.time()

        return [
            self.im_heat, self.marker_source, self.target_line,
            self.radar_line, self.radar_needle, self.meas_needle,
            self.srp_line, self.srp_peak_marker, self.hud_text,
            self.left_gauge_bg, self.left_gauge_fill, self.left_gauge_text,
            self.right_gauge_bg, self.right_gauge_fill, self.right_gauge_text,
            self.sector_banner_text
        ] + self.wave_lines + self.vu_texts

    def run(self):
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            interval=UPDATE_INTERVAL_MS,
            blit=False,
            cache_frame_data=False
        )
        plt.show()


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    print("=====================================================================")
    print(" SPATIAL MULTIMODAL ACOUSTIC RADAR ENGINE (PRO V6)")
    print(" 4-Microphone Uniform Linear Array + Multimodal Fault Localization")
    print("=====================================================================")
    print(" Physical Array Mapping on Breadboard:")
    print("   [Mic 1] (Row 20, Pair A Left,  GPIO 19, L/R->GND) -> x = -30.48 mm")
    print("   [Mic 2] (Row 28, Pair A Right, GPIO 19, L/R->3V3) -> x = -10.16 mm")
    print("   [Mic 3] (Row 36, Pair B Left,  GPIO 23, L/R->GND) -> x = +10.16 mm")
    print("   [Mic 4] (Row 44, Pair B Right, GPIO 23, L/R->3V3) -> x = +30.48 mm")
    print(" Total Array Aperture: 60.96 mm | 6 Correlation Delay Pairs")
    print(" ADXL345: SDA=TX2 (GPIO 17), SCL=RX2 (GPIO 16) | FC-03: DO=D4 (GPIO 4)")
    print("=====================================================================")
    app = AcousticRadarApp()
    app.run()
