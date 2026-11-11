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
from typing import Any, Dict, List, Optional, Tuple, Union

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
import sounddevice as sd
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap
from scipy import signal
from scipy.ndimage import gaussian_filter
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

# Acoustic Filter Bounds (Hz) - DC Motor & Voice Bandwidth (Admit rotational fundamentals & harmonics)
FREQ_MIN = 45.0
FREQ_MAX = 5500.0

# Fixed palette: blue/cyan/green/yellow/orange/red. Relative digital level, not dB SPL.
ACOUSTIC_HEAT_CMAP = LinearSegmentedColormap.from_list(
    "acoustic_thermal",
    ["#050914", "#123e91", "#08b9d2", "#18c979", "#e5d83b", "#ff9226", "#ff2424"], N=256
)

# Frame & Animation Timing
UPDATE_INTERVAL_MS = 50     # 20 FPS GUI refresh

# Preliminary Empirical Sensitivity Calibration Factors (Controlled-tone experiment relative to MIC3=1.00)
# MIC1=1.09, MIC2=2.47 (weak mic analog compensation), MIC3=1.00, MIC4=1.41
# Applied exclusively to floating-point amplitude analysis (VU meters, flank energy balance).
# Raw audio passed to GCC-PHAT / SRP-PHAT remains unscaled to prevent artificial noise inflation.
PRELIM_MIC_SENSITIVITY_GAINS = np.array([1.09, 2.47, 1.00, 1.41], dtype=np.float32)
HEATMAP_SMOOTH_ALPHA = 0.30  # Surface temporal smoothing


def apply_pair_phase_compensation(
    audio_4ch: np.ndarray,
    fs: float,
    comp_delay_sec: float
) -> np.ndarray:
    """
    Applies exact frequency-domain fractional delay phase compensation to Pair B (channels 2 & 3):
    H(f) = exp(-j * 2 * pi * f * comp_delay_sec)
    Uses symmetric edge-padding to eliminate boundary step transients.
    """
    if abs(comp_delay_sec) < 1e-12 or audio_4ch.shape[1] < 4:
        return audio_4ch.copy()
        
    n_samples, n_channels = audio_4ch.shape
    n_pad = 128
    n_fft = n_samples + 2 * n_pad
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / fs)
    phase_shift = np.exp(-2j * np.pi * freqs * comp_delay_sec)
    
    aligned = audio_4ch.copy()
    for ch in [2, 3]:
        if ch < n_channels:
            padded = np.pad(audio_4ch[:, ch], (n_pad, n_pad), mode="edge")
            ch_fft = np.fft.rfft(padded, n=n_fft)
            compensated = np.fft.irfft(ch_fft * phase_shift, n=n_fft)
            aligned[:, ch] = compensated[n_pad : n_pad + n_samples]
            
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

        if not is_active_sound or meas_prom < 0.50:
            dt_silence = now - self.last_valid_time
            self.confidence *= 0.85
            if dt_silence > 0.25:
                self.locked = False
            return self.tracked_angle, self.tracked_range, self.confidence, False

        self.last_valid_time = now
        meas_conf_pct = min(99.0, max(30.0, meas_prom * 50.0))

        if not self.locked:
            self.tracked_angle = meas_angle
            self.tracked_range = nominal_range
            self.confidence = meas_conf_pct
            self.locked = True
            self.pending_count = 0
        else:
            diff = meas_angle - self.tracked_angle
            # Normalize angle difference to [-180, 180]
            diff = (diff + 180.0) % 360.0 - 180.0
            if abs(diff) <= self.gate_deg:
                self.tracked_angle += self.alpha * diff
                self.tracked_range = 0.85 * self.tracked_range + 0.15 * nominal_range
                self.confidence = (1.0 - self.alpha) * self.confidence + self.alpha * meas_conf_pct
                self.pending_count = 0
            else:
                # Fast re-acquisition when sound moves across sectors
                if meas_prom > 0.80:
                    self.tracked_angle = 0.25 * self.tracked_angle + 0.75 * meas_angle
                    self.confidence = meas_conf_pct
                    self.pending_count = 0
                else:
                    self.pending_angle = meas_angle
                    self.pending_count += 1
                    if self.pending_count >= 3:
                        self.tracked_angle = meas_angle
                        self.confidence = meas_conf_pct
                        self.pending_count = 0

        return self.tracked_angle, self.tracked_range, self.confidence, self.locked


# =============================================================================
# ACOUSTIC ACTIVITY DETECTOR (SCHMITT-TRIGGER VAD WITH NOISE FLOOR FREEZE)
# =============================================================================
class AcousticActivityDetector:
    """
    Production Dual-Threshold Schmitt-Trigger Voice/Acoustic Activity Detector (VAD).
    Features:
      1. Initial ambient quiet-room baseline calibration (median of quiet frames).
      2. Hangover smoothing to bridge speech/acoustic word pauses without flutter.
      3. Strict Noise Floor Adaptation Freeze: during active acoustic sound, the
         ambient floor is 100% frozen, preventing loud sound from absorbing into baseline.
      4. Downward baseline tracking during confirmed quiet intervals (>0.60s post-sound).
    """

    def __init__(
        self,
        min_active_threshold: float = 0.0035,
        calib_frames: int = 25,
        hangover_sec: float = 0.350
    ):
        self.min_active_threshold = min_active_threshold
        self.calib_frames = calib_frames
        self.hangover_sec = hangover_sec
        self.noise_floor = 0.0030
        self.calib_samples: List[float] = []
        self.is_calibrating = True
        self.last_active_time = 0.0

    def start_calibration(self):
        """Clears existing sample history and restarts ambient noise floor calibration."""
        self.calib_samples.clear()
        self.is_calibrating = True

    def note_coherent_source(self, current_time: float):
        """Recover when SRP proves a coherent source was present during startup."""
        self.is_calibrating = False
        self.calib_samples.clear()
        self.noise_floor = min(max(0.0008, self.noise_floor), 0.0030)
        self.min_active_threshold = max(0.0025, self.noise_floor * 1.60)
        self.last_active_time = current_time

    def process_frame(self, avg_rms: float, current_time: float) -> Tuple[bool, float, float]:
        """
        Evaluates incoming audio frame energy against dynamic Schmitt thresholds.
        Returns:
            is_active: Whether sound activity is detected in this frame
            noise_floor: Current estimated ambient floor RMS
            attack_thresh: Active sound attack threshold
        """
        # Phase 1: Ambient baseline calibration
        if self.is_calibrating:
            self.calib_samples.append(avg_rms)
            if len(self.calib_samples) >= self.calib_frames:
                # Lower quantile is less sensitive to brief target sound during startup.
                self.noise_floor = max(0.0008, float(np.percentile(self.calib_samples, 20)))
                self.min_active_threshold = max(0.0025, self.noise_floor * 1.60)
                self.is_calibrating = False

        # Phase 2: Dual-threshold Schmitt Trigger with Hangover
        attack_thresh = max(self.min_active_threshold, self.noise_floor * 1.60)
        release_thresh = max(0.0020, self.noise_floor * 1.25)

        if avg_rms >= attack_thresh:
            self.last_active_time = current_time
            is_active = True
        elif (current_time - self.last_active_time) < self.hangover_sec:
            is_active = bool(avg_rms >= release_thresh)
        else:
            is_active = False

        # Phase 3: Noise floor adaptation rules (frozen during sound; tracks quiet intervals)
        if not is_active and (current_time - self.last_active_time) > 0.60:
            if avg_rms < self.noise_floor:
                self.noise_floor = 0.92 * self.noise_floor + 0.08 * avg_rms
            elif avg_rms < self.noise_floor * 1.30:
                self.noise_floor = 0.995 * self.noise_floor + 0.005 * avg_rms

        return is_active, self.noise_floor, attack_thresh


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
    def compute_spatial_coherence(signals: np.ndarray, max_delay_samples: int = 6) -> Tuple[int, float]:
        """
        Computes scale-invariant inter-channel normalized cross-correlation across all pairs
        within physical inter-microphone propagation lag window.
        Returns:
            coherent_pairs: Count of microphone pairs exhibiting normalized cross-correlation >= 0.18
            max_rho: Maximum cross-channel correlation coefficient observed across all pairs
        """
        N, M = signals.shape
        if M < 2 or N < 16:
            return 0, 0.0
        norms = np.sqrt(np.sum(signals.astype(np.float64)**2, axis=0))
        alive_channels = [m for m in range(M) if norms[m] >= 1e-12]
        if len(alive_channels) < 2:
            return 0, 0.0
        coherent_pairs = 0
        max_rho = 0.0
        for i_idx, i in enumerate(alive_channels):
            for j in alive_channels[i_idx + 1:]:
                s1 = signals[:, i].astype(np.float64)
                s2 = signals[:, j].astype(np.float64)
                denom = norms[i] * norms[j]
                if denom < 1e-12:
                    continue
                c = np.correlate(s1, s2, mode='full') / denom
                center = N - 1
                lo = max(0, center - max_delay_samples)
                hi = min(len(c), center + max_delay_samples + 1)
                phys_c = c[lo:hi]
                if len(phys_c) > 0:
                    rho = float(np.max(phys_c))
                    if rho > max_rho:
                        max_rho = rho
                    if rho >= 0.18:
                        coherent_pairs += 1
        return coherent_pairs, max_rho

    @staticmethod
    def gcc_phat_pair(
        sig_i: np.ndarray,
        sig_j: np.ndarray,
        fs: float,
        d_ij_m: float,
        gamma: float = 0.70,
        interp_factor: int = 16,
        min_signal_rms: float = 0.0015,
        gain_i: float = 1.0,
        gain_j: float = 1.0
    ) -> Tuple[float, float, np.ndarray, np.ndarray, bool]:
        """
        Computes super-resolution sinc GCC-PHAT between microphone pair (i, j).
        Convention: sig_i is leftward (smaller x), sig_j is rightward (larger x).
        Returns:
            best_tau: Estimated delay in seconds
            prominence: SNR prominence of the correlation peak
            c_phys: Cross-correlation in physical delay window
            l_phys: Physical delay axis
            is_valid: Boolean indicating whether peak passes coherence/PSR thresholds
        """
        n = len(sig_i)
        tau_max = (d_ij_m / SPEED_OF_SOUND) * 1.08
        dummy_lags = np.linspace(-tau_max, tau_max, 3)

        # 1. Absolute Energy Gating: Reject weak background noise before normalization
        # Evaluated on sensitivity-calibrated effective RMS to prevent discarding weak microphones (e.g. MIC2)
        rms_i = float(np.sqrt(np.mean(sig_i ** 2))) * gain_i
        rms_j = float(np.sqrt(np.mean(sig_j ** 2))) * gain_j
        if rms_i < min_signal_rms or rms_j < min_signal_rms:
            return 0.0, 0.0, np.zeros(3), dummy_lags, False

        w = signal.windows.tukey(n, alpha=0.25)
        si = (sig_i - np.mean(sig_i)) * w
        sj = (sig_j - np.mean(sig_j)) * w

        std_i = float(np.std(si)) * gain_i
        std_j = float(np.std(sj)) * gain_j
        if std_i < min_signal_rms or std_j < min_signal_rms:
            return 0.0, 0.0, np.zeros(3), dummy_lags, False

        si = si / std_i
        sj = sj / std_j

        Si = rfft(si)
        Sj = rfft(sj)

        cross = Si * np.conj(Sj)
        mag = np.abs(cross)

        freqs = np.fft.rfftfreq(n, d=1.0 / fs)
        band_mask = (freqs >= FREQ_MIN) & (freqs <= min(FREQ_MAX, fs / 2.0 - 50.0))

        inband_energy = float(np.sum(mag[band_mask] ** 2))
        total_energy = float(np.sum(mag ** 2)) + 1e-20
        if inband_energy / total_energy < 0.04:
            return 0.0, 0.0, np.zeros(3), dummy_lags, False

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

        phys_mask = np.abs(lags) <= tau_max
        c_phys = corr[phys_mask]
        l_phys = lags[phys_mask]

        if len(c_phys) < 5:
            return 0.0, 0.0, c_phys, l_phys, False

        k = int(np.argmax(c_phys))
        peak_val = float(c_phys[k])
        mean_val = float(np.mean(c_phys))
        std_val = float(np.std(c_phys)) + 1e-12
        prominence = max(0.0, (peak_val - mean_val) / std_val)

        # 2. Statistical Peak Significance Test:
        # Validates that cross-correlation peak rises significantly above baseline correlation floor
        # and does not stick to the unphysical lag window boundary.
        is_valid = bool(prominence >= 0.70 and peak_val > 0.012 and 0 < k < len(c_phys) - 1)

        if k == 0 or k == len(c_phys) - 1:
            return 0.0, 0.0, c_phys, l_phys, False

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
        return best_tau, prominence, c_phys, l_phys, is_valid

    @classmethod
    def compute_srp_frame(
        cls,
        signals: np.ndarray,
        fs: float,
        geometry: ArrayGeometry,
        min_active_rms: float = 0.0020,
        calib_gains: Optional[np.ndarray] = None
    ) -> Tuple[float, float, np.ndarray, np.ndarray, np.ndarray, Dict[str, object]]:
        """
        Steered Response Power (SRP) bearing estimation AND True 2D Acoustic Camera Backprojection.
        Returns:
            best_bearing_deg: Estimated angle in degrees [-90°, +90°]
            composite_prominence: SNR prominence of the aggregate peak
            angles_deg: Angle grid (181 points)
            srp_norm: Normalized 1D SRP power vs angle
            raw_2d_heatmap: True 2D backprojected acoustic intensity field (GRID_Y, GRID_X)
            side_metrics: Directional power breakdown (pct_left, pct_center, pct_right, dom_side, is_valid, active_pairs)
        """
        num_mics = min(signals.shape[1], geometry.num_mics)
        angles_deg = np.linspace(-90.0, 90.0, 181)
        angles_rad = np.deg2rad(angles_deg)
        sin_angles = np.sin(angles_rad)

        srp_profile = np.zeros_like(angles_deg, dtype=np.float64)
        raw_2d_heatmap = np.zeros((GRID_Y, GRID_X), dtype=np.float64)
        pair_prominences = []

        # 1. Per-Channel Health & Signal Strength Diagnostics (Raw & Sensitivity-Calibrated)
        ch_rms = [float(np.sqrt(np.mean(signals[:, m]**2))) for m in range(num_mics)]
        ch_peak = [float(np.max(np.abs(signals[:, m]))) for m in range(num_mics)]
        gains = calib_gains if calib_gains is not None else PRELIM_MIC_SENSITIVITY_GAINS[:num_mics]
        calib_ch_rms = [ch_rms[m] * float(gains[m]) if m < len(gains) else ch_rms[m] for m in range(num_mics)]
        avg_rms = float(np.mean(ch_rms)) if ch_rms else 0.0
        avg_calib_rms = float(np.mean(calib_ch_rms)) if calib_ch_rms else avg_rms

        ch_status = []
        for m in range(num_mics):
            effective_rms = calib_ch_rms[m]
            if ch_peak[m] > 0.98:
                ch_status.append("CLIPPED")
            elif ch_rms[m] < 1e-4 and avg_rms > 0.003:
                ch_status.append("DEAD")
            elif effective_rms < min_active_rms:
                ch_status.append("QUIET")
            else:
                ch_status.append("ACTIVE")

        # Silence Gate: If overall physical acoustic energy (calibrated) is below physical threshold,
        # return an explicit QUIET state immediately. DO NOT compute SRP on noise!
        if avg_calib_rms < min_active_rms or sum(st != "DEAD" for st in ch_status) < 2:
            return (
                0.0,
                0.0,
                angles_deg,
                np.zeros_like(angles_deg),
                np.zeros((GRID_Y, GRID_X), dtype=np.float64),
                {
                    "pct_left": 33.3, "pct_center": 33.3, "pct_right": 33.3,
                    "dom_side": "NONE", "is_valid": False, "active_pairs": 0,
                    "avg_rms": avg_rms, "ch_rms": ch_rms, "calib_ch_rms": calib_ch_rms, "ch_status": ch_status,
                    "spatial_rho": 0.0, "status": "BELOW_ENERGY_GATE"
                }
            )

        # Scale-Invariant Spatial Coherence Gate:
        # Computes normalized inter-channel correlation across microphone pairs.
        # Coherent acoustic wavefronts exhibit rho >= 0.18, whereas independent noise
        # (even at high amplitude) has max rho < 0.15. Rejects independent noise 100%.
        coherent_pairs, max_spatial_rho = cls.compute_spatial_coherence(signals, max_delay_samples=6)
        if coherent_pairs < 1 and max_spatial_rho < 0.18:
            return (
                0.0,
                0.0,
                angles_deg,
                np.zeros_like(angles_deg),
                np.zeros((GRID_Y, GRID_X), dtype=np.float64),
                {
                    "pct_left": 33.3, "pct_center": 33.3, "pct_right": 33.3,
                    "dom_side": "NONE", "is_valid": False, "active_pairs": 0,
                    "avg_rms": avg_rms, "ch_rms": ch_rms, "calib_ch_rms": calib_ch_rms, "ch_status": ch_status,
                    "spatial_rho": max_spatial_rho, "status": "LOW_SPATIAL_COHERENCE"
                }
            )

        valid_pairs_count = 0
        pair_angles: List[float] = []
        active_ch_count = sum(st == "ACTIVE" for st in ch_status)

        for p_idx, (i, j) in enumerate(geometry.pairs):
            if i >= num_mics or j >= num_mics:
                continue
            if ch_status[i] in ("DEAD", "CLIPPED") or ch_status[j] in ("DEAD", "CLIPPED"):
                continue

            sig_i = signals[:, i]
            sig_j = signals[:, j]
            d_x = geometry.mic_coords[j, 0] - geometry.mic_coords[i, 0]
            d_ij = abs(d_x) if abs(d_x) > 1e-4 else geometry.baseline_m

            best_tau, prom, c_phys, l_phys, is_valid = cls.gcc_phat_pair(
                sig_i, sig_j, fs, d_ij, min_signal_rms=min_active_rms * 0.40,
                gain_i=float(gains[i]) if i < len(gains) else 1.0,
                gain_j=float(gains[j]) if j < len(gains) else 1.0
            )
            pair_prominences.append(prom)

            if is_valid and len(c_phys) >= 5 and prom >= 0.70:
                valid_pairs_count += 1
                sin_p = np.clip((best_tau * SPEED_OF_SOUND) / d_x, -1.0, 1.0)
                pair_angles.append(float(np.rad2deg(np.arcsin(sin_p))))
                # 1D angular SRP: tau = +(d_x / c) * sin(theta) (Positive lag for sound on right)
                taus_1d = +(d_x / SPEED_OF_SOUND) * sin_angles
                pair_resp_1d = np.interp(taus_1d, l_phys, c_phys, left=0.0, right=0.0)
                pair_weight = math.sqrt(d_ij / geometry.baseline_m) * min(prom, 4.0)
                srp_profile += pair_weight * np.maximum(pair_resp_1d, 0.0)

                # 2D True Spatial SRP Backprojection across physical room grid
                tau_grid = +geometry.pair_delays[p_idx].ravel()
                pair_resp_2d = np.interp(tau_grid, l_phys, c_phys, left=0.0, right=0.0).reshape((GRID_Y, GRID_X))
                raw_2d_heatmap += pair_weight * np.maximum(pair_resp_2d, 0.0)

        # Consistency Gate: Require at least 1 valid pair to form a reliable bearing
        max_srp = float(np.max(srp_profile))
        if valid_pairs_count < 1 or max_srp < 1e-6:
            return (
                0.0,
                0.0,
                angles_deg,
                np.zeros_like(angles_deg),
                np.zeros((GRID_Y, GRID_X), dtype=np.float64),
                {
                    "pct_left": 33.3, "pct_center": 33.3, "pct_right": 33.3,
                    "dom_side": "NONE", "is_valid": False, "active_pairs": valid_pairs_count,
                    "avg_rms": avg_rms, "ch_rms": ch_rms, "calib_ch_rms": calib_ch_rms, "ch_status": ch_status,
                    "spatial_rho": max_spatial_rho, "status": "NO_VALID_GCC_PAIRS"
                }
            )

        srp_norm = srp_profile / max_srp
        k_peak = int(np.argmax(srp_norm))
        best_bearing_deg = float(angles_deg[k_peak])
        mean_srp = float(np.mean(srp_norm))
        std_srp = float(np.std(srp_norm)) + 1e-12
        composite_prom = max(0.0, (1.0 - mean_srp) / std_srp)

        # Contrast check: is the peak well-separated from the angular baseline?
        if composite_prom < 1.0:
            # Low contrast / diffuse acoustic field - return angular profile for display, flagged invalid
            return (
                best_bearing_deg,
                composite_prom,
                angles_deg,
                srp_norm,
                raw_2d_heatmap,
                {
                    "pct_left": 33.3, "pct_center": 33.3, "pct_right": 33.3,
                    "dom_side": "NONE", "is_valid": False, "active_pairs": valid_pairs_count,
                    "avg_rms": avg_rms, "ch_rms": ch_rms, "calib_ch_rms": calib_ch_rms, "ch_status": ch_status,
                    "spatial_rho": max_spatial_rho, "status": "LOW_ANGULAR_CONTRAST"
                }
            )

        # Multi-Pair Angular Consensus Gate:
        # When 3+ pairs are valid, verify at least 2 pairs agree within 25°
        agreeing_pairs = sum(1 for ang in pair_angles if abs(ang - best_bearing_deg) <= 25.0)
        if valid_pairs_count >= 3 and agreeing_pairs < 2:
            return (
                best_bearing_deg,
                composite_prom,
                angles_deg,
                srp_norm,
                raw_2d_heatmap,
                {
                    "pct_left": 33.3, "pct_center": 33.3, "pct_right": 33.3,
                    "dom_side": "NONE", "is_valid": False, "active_pairs": valid_pairs_count,
                    "avg_rms": avg_rms, "ch_rms": ch_rms, "calib_ch_rms": calib_ch_rms, "ch_status": ch_status,
                    "spatial_rho": max_spatial_rho, "status": "PAIR_ANGLE_DISAGREEMENT"
                }
            )

        # Sector Power & Side Intensity Breakdown with Noise-Floor Contrast
        l_mask = angles_deg < -15.0
        c_mask = (angles_deg >= -15.0) & (angles_deg <= 15.0)
        r_mask = angles_deg > 15.0

        baseline = float(np.percentile(srp_norm, 20)) if len(srp_norm) > 0 else 0.0
        contrast = np.maximum(0.0, srp_norm - baseline) ** 2.0

        p_l = float(np.sum(contrast[l_mask])) if np.any(l_mask) else 0.0
        p_c = float(np.sum(contrast[c_mask])) if np.any(c_mask) else 0.0
        p_r = float(np.sum(contrast[r_mask])) if np.any(r_mask) else 0.0

        # Calibrated Physical Flank Amplitude Weighting:
        # Fuses SRP angular contrast with calibrated microphone pair energy balance
        # (Left mics: M1, M2; Right mics: M3, M4)
        if len(calib_ch_rms) >= 4:
            e_left = 0.5 * (calib_ch_rms[0]**2 + calib_ch_rms[1]**2) + 1e-12
            e_right = 0.5 * (calib_ch_rms[2]**2 + calib_ch_rms[3]**2) + 1e-12
            e_tot_calib = e_left + e_right
            w_left = (e_left / e_tot_calib) * 2.0
            w_right = (e_right / e_tot_calib) * 2.0
            p_l *= (0.70 + 0.30 * w_left)
            p_r *= (0.70 + 0.30 * w_right)

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
            "dom_side": dom_side,
            "is_valid": True,
            "active_pairs": valid_pairs_count,
            "avg_rms": avg_rms,
            "ch_rms": ch_rms,
            "calib_ch_rms": calib_ch_rms,
            "ch_status": ch_status,
            "spatial_rho": max_spatial_rho,
            "status": "LOCALIZED"
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
        cls, raw_2d_heatmap: np.ndarray, target_range_m: float, avg_rms: float,
        is_locked: bool, geometry: ArrayGeometry, noise_floor: float = 0.0030
    ) -> np.ndarray:
        """Fixed monotonic thermal intensity from SRP shape and RMS above ambient.

        Relative digital level, not calibrated dB SPL. A 1D ULA measures bearing,
        not source range; radial weighting is only for display.
        """
        raw = np.asarray(raw_2d_heatmap, dtype=np.float64)
        raw = np.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0)
        spatial = np.maximum(raw, 0.0)
        peak = float(np.max(spatial)) if spatial.size else 0.0
        if peak <= 1e-12:
            return np.zeros_like(raw, dtype=np.float64)
        # Normalize spatial shape only; never normalize final intensity per frame.
        spatial /= peak
        spatial = gaussian_filter(spatial, sigma=1.15, mode="nearest")
        spatial_peak = float(np.max(spatial))
        if spatial_peak <= 1e-12:
            return np.zeros_like(raw, dtype=np.float64)
        spatial /= spatial_peak
        r_grid = np.sqrt(geometry.xx ** 2 + geometry.yy ** 2)
        dist_weight = 1.0 / np.sqrt(np.maximum(r_grid, 0.40))
        dist_weight /= max(float(np.max(dist_weight)), 1e-12)
        spatial *= dist_weight
        spatial /= max(float(np.max(spatial)), 1e-12)
        # Fixed monotonic 1.5–16 dB-above-ambient digital scale.
        floor = max(float(noise_floor), 0.0008)
        level_db = 20.0 * math.log10(max(float(avg_rms), 1e-9) / floor)
        level = float(np.clip((level_db - 1.5) / 14.5, 0.0, 1.0))
        return np.clip(np.power(np.clip(spatial, 0.0, 1.0), 1.35) * level, 0.0, 1.0)

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
        self.rpm_history = collections.deque(maxlen=20)

        # Per-axis gravity tracking & vibration time series buffer
        self.grav_x = 0.0
        self.grav_y = 0.0
        self.grav_z = -0.82
        self.vib_history = collections.deque(maxlen=256)
        self.vib_signed_history = collections.deque(maxlen=256)
        self.vib_signed_y_history = collections.deque(maxlen=256)
        self.vib_signed_z_history = collections.deque(maxlen=256)

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

        # Track RPM jitter (bounded by deque maxlen=20)
        self.rpm_history.append(rpm)
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

            # Robust RMS dynamic vibration magnitude across window (eliminates point-sample flicker)
            dyn_mag_sq = dyn_ax_arr**2 + dyn_ay_arr**2 + dyn_az_arr**2
            dyn_g = float(np.sqrt(np.mean(dyn_mag_sq)))

            # 3-Axis Signed Vibration Components for Orientation-Invariant Order Tracking
            dyn_axes = [dyn_ax_arr, dyn_ay_arr, dyn_az_arr]

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

            dyn_pt = math.sqrt(dyn_ax**2 + dyn_ay**2 + dyn_az**2)
            self.vib_history.append(dyn_pt)
            self.vib_signed_history.append(dyn_ax)
            self.vib_signed_y_history.append(dyn_ay)
            self.vib_signed_z_history.append(dyn_az)

            dyn_g = float(np.sqrt(np.mean(np.array(self.vib_history)**2)))
            dyn_axes = [
                np.array(self.vib_signed_history, dtype=np.float64),
                np.array(self.vib_signed_y_history, dtype=np.float64),
                np.array(self.vib_signed_z_history, dtype=np.float64)
            ]
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

        is_motor_running = bool(rpm >= 30.0)

        # 2. Rotational Order Analysis from Multi-Axis Power Spectrum (Orientation Invariant, No Rectification Doubling)
        f0 = (rpm / 60.0) if is_motor_running else 0.0
        n_samples_vib = len(dyn_axes[0])
        if is_motor_running and n_samples_vib >= 32 and rpm > 60.0:
            n_fft = max(256, n_samples_vib * 4)  # Zero-padding for ~0.08 Hz bin spacing
            vib_freqs = np.fft.rfftfreq(n_fft, d=1.0 / fs_vib)
            # Sum power spectra of each signed axis: P(f) = |X(f)|^2 + |Y(f)|^2 + |Z(f)|^2
            vib_power_spec = np.zeros_like(vib_freqs, dtype=np.float64)
            for axis_sig in dyn_axes:
                axis_ac = np.array(axis_sig, dtype=np.float64)
                axis_ac -= np.mean(axis_ac)
                vib_power_spec += np.abs(np.fft.rfft(axis_ac, n=n_fft)) ** 2

            def get_order_energy(target_f, bandwidth=1.5):
                mask = (vib_freqs >= max(0.5, target_f - bandwidth)) & (vib_freqs <= target_f + bandwidth)
                return float(np.sum(vib_power_spec[mask])) if np.any(mask) else 0.0

            tot_vib_energy = float(np.sum(vib_power_spec)) + 1e-15
            e_1x = get_order_energy(f0) / tot_vib_energy
            e_2x = get_order_energy(2.0 * f0) / tot_vib_energy
            e_3x = get_order_energy(3.0 * f0) / tot_vib_energy
        else:
            e_1x = 0.0
            e_2x = 0.0
            e_3x = 0.0

        # True ISO 10816/20816 Vibration Velocity RMS (mm/s) via Parseval Omega-Arithmetic Integration
        # V(f) = (A_w(f) * 9806.65) / (2 * pi * f) for f in [10 Hz, 200 Hz]
        v_rms_mms = 0.0
        if n_samples_vib >= 16:
            n_vfft = max(128, n_samples_vib)
            v_freqs = np.fft.rfftfreq(n_vfft, d=1.0 / fs_vib)
            iso_band = (v_freqs >= 10.0) & (v_freqs <= min(200.0, fs_vib / 2.0))
            if np.any(iso_band):
                v_axis_rms_sq = 0.0
                scale = np.full_like(v_freqs, 2.0)
                scale[0] = 1.0
                if n_vfft % 2 == 0:
                    scale[-1] = 1.0

                safe_freqs = v_freqs.copy()
                safe_freqs[0] = 1.0  # Guard against DC division

                for axis_sig in dyn_axes:
                    ax_data = np.array(axis_sig, dtype=np.float64)
                    if len(ax_data) < 16:
                        continue
                    ax_data -= np.mean(ax_data)
                    w_v = np.hanning(len(ax_data))
                    sum_w_sq = float(np.sum(w_v**2)) + 1e-12

                    X_w = np.fft.rfft(ax_data * w_v, n=n_vfft)
                    # Velocity spectral amplitude (mm/s)
                    V_w = (np.abs(X_w) * 9806.65) / (2.0 * np.pi * safe_freqs)
                    # Parseval discrete energy integration
                    axis_v_rms_sq = float(np.sum(scale[iso_band] * (V_w[iso_band]**2))) / (n_vfft * sum_w_sq)
                    v_axis_rms_sq += axis_v_rms_sq

                v_rms_mms = float(np.sqrt(v_axis_rms_sq))

        # Acoustic spectrum for high-frequency acoustic baseline comparison
        spec = np.abs(np.fft.rfft(x))

        # 3. Fault Classification Diagnostics (Multimodal Edge-AI Decision Rules)
        is_acoustic_elevated = bool(rms > 0.025 or kurt > 4.2 or crest > 4.2)
        is_vib_elevated = bool(dyn_g > 0.25)

        if not is_motor_running:
            fault_label = "MOTOR STOPPED / STANDBY (Awaiting Rotation)"
            iso_severity = "STANDBY: MOTOR OFF"
            iso_color = "#64748b"
        elif is_vib_elevated and dyn_g > 0.40 and e_1x > 0.12:
            fault_label = "MASS UNBALANCE (Strong 1x RPM Vibration)"
        elif is_vib_elevated and dyn_g > 0.30 and e_2x > 0.10:
            fault_label = "SHAFT MISALIGNMENT / BENT (Strong 2x RPM)"
        elif is_vib_elevated and crest > 4.5 and kurt > 4.5:
            fault_label = "BEARING IMPACT / BRUSH ARCING (High Kurtosis)"
        elif is_vib_elevated and dyn_g > 0.35:
            fault_label = "ELEVATED MECHANICAL VIBRATION (ISO Zone C Alert)"
        elif is_acoustic_elevated and not is_vib_elevated:
            # Core Minor Project Principle (PDF Page 13): Acoustic Disturbance != Mechanical Fault
            fault_label = "ACOUSTIC DISTURBANCE DETECTED (Vibration Nominal -> Non-Mechanical)"
        elif rms > 0.035 and kurt < 2.5:
            fault_label = "COMMUTATOR NOISE / MECHANICAL DRAG"
        else:
            fault_label = "NOMINAL / SMOOTH OPERATION"

        # 4. Golden Baseline Comparison (Detecting Tiny Differences!)
        delta_info = {
            "d_rpm": 0.0,
            "d_vib_g": 0.0,
            "d_kurt": 0.0,
            "d_rms_pct": 0.0,
            "spec_dist_pct": 0.0
        }
        delta_penalty = 0.0
        if self.baseline_locked and is_motor_running:
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
        if not is_motor_running:
            health_score = 0.0
        else:
            total_penalty = vib_penalty + max(0.0, (kurt - 3.0) * 3.0) + delta_penalty
            if is_acoustic_elevated and not is_vib_elevated:
                # External environmental acoustic noise must not degrade mechanical motor health rating
                total_penalty = min(total_penalty, 5.0)
            health_score = max(5.0, min(100.0, 100.0 - total_penalty))

        return {
            "is_running": is_motor_running,
            "health_score": health_score,
            "rpm": rpm,
            "rpm_jitter": rpm_jitter,
            "f0_hz": f0,
            "dyn_vib_g": dyn_g,
            "v_rms_mms": v_rms_mms,
            "iso_severity": iso_severity,
            "iso_color": iso_color,
            "crest_factor": crest,
            "kurtosis": kurt,
            "primary_fault": fault_label,
            "e_1x_pct": e_1x * 100.0,
            "e_2x_pct": e_2x * 100.0,
            "e_3x_pct": e_3x * 100.0,
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


def compute_packet_checksum(data: bytes) -> int:
    """
    Computes 16-bit XOR checksum initialized to 0x55AA across contiguous byte buffer.
    Matches ESP32 firmware implementation for serial binary packets.
    """
    chk = 0x55AA
    for i, b in enumerate(data):
        chk ^= ((b << (i % 8)) & 0xFFFF)
    return chk & 0xFFFF


class ESP32MultimodalSource(AudioSourceBase):
    """
    Production 4-Channel Real-Time PCM Ingestion from ESP32 on COM8.
    Streams 10,000 Hz 16-bit 4-channel audio + ADXL345 3-axis vibration (continuous 400 Hz FIFO stream)
    + FC-03 tachometer at 921,600 baud using zero-copy binary DMA packets (packet type 0x05).
    """

    def __init__(self, port: str = "auto", baudrates: Tuple[int, ...] = (921600, 1152000, 115200)):
        self.port = port
        self.baudrates = baudrates
        self.fs = 10000.0
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

        self._rx_buf = bytearray()

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
            "last_packet_time": 0.0,
            "num_vib_last": 0,
            "port": port if port != "auto" else "COM8",
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
            return self.port if self.port and self.port != "auto" else "COM8"
        # If user explicitly specified a port that isn't "auto", try that directly
        if self.port and self.port.lower() != "auto":
            return self.port

        # Try standalone hardware port detector
        try:
            tools_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "tools")
            if tools_dir not in sys.path:
                sys.path.insert(0, tools_dir)
            import find_port
            detected = find_port.detect_port()
            if detected:
                return detected
        except Exception:
            pass

        # Robust in-situ detection with Bluetooth / ACPI rejection
        try:
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                h = ((p.hwid or "") + " " + (p.description or "")).lower()
                if any(b in h for b in ["bthenum", "bluetooth", "acpi", "pnp0501"]):
                    continue
                if any(k in h for k in ["10c4:ea60", "cp210", "1a86:7523", "ch340", "0403:6001", "ftdi", "303a:", "esp32"]):
                    return p.device
            for p in ports:
                h = ((p.hwid or "") + " " + (p.description or "")).lower()
                if any(b in h for b in ["bthenum", "bluetooth", "acpi", "pnp0501"]):
                    continue
                if "usb" in h:
                    return p.device
            for p in ports:
                if p.device.upper() == "COM8":
                    return p.device
        except Exception:
            pass
        return "COM8"

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

    @staticmethod
    def build_packet_v2(
        seq: int,
        fs: int,
        block_frames: int,
        accel_samples: List[Tuple[int, int, int]],
        audio_data: np.ndarray,
        tach_pulses: int = 0
    ) -> bytes:
        """
        Constructs a binary Packet Type 0x05 (PKT_TYPE_4CH_MULTIVIB) frame per PROJECT.md interface contract.
        Header: 16 bytes (sync1, sync2, pkt_type=0x05, channels=4, seq, fs, block_frames, num_vib, reserved, pulses)
        Vibration payload: num_vib * 6 bytes (int16 x_mg, y_mg, z_mg)
        Audio payload: block_frames * 4 * 2 bytes (int16 PCM)
        Checksum: 16-bit XOR initialized to 0x55AA across header, vibration, and audio payloads.
        """
        num_vib = len(accel_samples)
        header = struct.pack(
            "<BBBBHHHBBI",
            0xAA, 0x55, 0x05, 4,
            seq & 0xFFFF,
            int(fs) & 0xFFFF,
            int(block_frames) & 0xFFFF,
            num_vib & 0xFF,
            0x00,
            tach_pulses & 0xFFFFFFFF
        )
        vib_bytes = bytearray()
        for x, y, z in accel_samples:
            vib_bytes.extend(struct.pack("<hhh", int(x), int(y), int(z)))

        if audio_data.dtype in (np.float32, np.float64):
            audio_int16 = (np.clip(audio_data, -1.0, 1.0) * 32767.0).astype(np.int16)
        else:
            audio_int16 = audio_data.astype(np.int16)
        audio_bytes = audio_int16.tobytes()

        data_to_checksum = header + bytes(vib_bytes) + audio_bytes
        chk = compute_packet_checksum(data_to_checksum)
        chk_bytes = struct.pack("<H", chk)

        return data_to_checksum + chk_bytes

    @staticmethod
    def build_packet_v1(
        seq: int,
        fs: int,
        block_frames: int,
        accel_sample: Tuple[int, int, int],
        audio_data: np.ndarray,
        tach_pulses: int = 0
    ) -> bytes:
        """
        Constructs a legacy binary Packet Type 0x04 frame for backward compatibility testing.
        """
        ax_mg, ay_mg, az_mg = accel_sample
        header = struct.pack(
            "<BBBBHHHhhhI",
            0xAA, 0x55, 0x04, 4,
            seq & 0xFFFF,
            int(fs) & 0xFFFF,
            int(block_frames) & 0xFFFF,
            int(ax_mg), int(ay_mg), int(az_mg),
            tach_pulses & 0xFFFFFFFF
        )
        if audio_data.dtype in (np.float32, np.float64):
            audio_int16 = (np.clip(audio_data, -1.0, 1.0) * 32767.0).astype(np.int16)
        else:
            audio_int16 = audio_data.astype(np.int16)
        audio_bytes = audio_int16.tobytes()

        chk = 0x55AA
        for i, b in enumerate(header):
            chk ^= ((b << (i % 8)) & 0xFFFF)
        for i, b in enumerate(audio_bytes):
            chk ^= ((b << (i % 8)) & 0xFFFF)
        chk_bytes = struct.pack("<H", chk)

        return header + audio_bytes + chk_bytes

    def _parse_incoming_bytes(self, chunk: bytes, port_str: str = "COM8", baud_str: Union[int, str] = 1152000) -> int:
        """
        Parses incoming serial byte stream into ring buffers and telemetry.
        Handles both Type 0x05 (multi-sample continuous 400Hz vibration) and Type 0x04 (legacy single-sample).
        Validates 16-bit XOR checksum, tracks dropped packets via sequence counter, and rejects corruption.
        Returns the number of valid packets extracted in this call.
        """
        HEADER_FORMAT_V2 = "<BBBBHHHBBI"  # 16 bytes: sync1, sync2, pkt_type, channels, seq, fs, block_frames, num_vib, reserved, pulses
        HEADER_SIZE_V2 = struct.calcsize(HEADER_FORMAT_V2)

        HEADER_FORMAT_V1 = "<BBBBHHHhhhI"  # 20 bytes: sync1, sync2, pkt_type, channels, seq, fs, block_frames, ax, ay, az, pulses
        HEADER_SIZE_V1 = struct.calcsize(HEADER_FORMAT_V1)

        self._rx_buf.extend(chunk)
        buf = self._rx_buf
        parsed_count = 0

        while len(buf) >= 3:
            sync_idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == 0xAA and buf[i+1] == 0x55:
                    sync_idx = i
                    break

            if sync_idx == -1:
                if len(buf) > 0 and buf[-1] == 0xAA:
                    del buf[:-1]
                else:
                    buf.clear()
                break

            if sync_idx > 0:
                del buf[:sync_idx]

            if len(buf) < 3:
                break

            pkt_type = buf[2]

            if pkt_type == 0x05:
                if len(buf) < HEADER_SIZE_V2:
                    break

                sync1, sync2, ptype, channels, seq, fs_val, block_frames, num_vib, reserved, pulses = struct.unpack_from(
                    HEADER_FORMAT_V2, buf, 0
                )

                # Defensive sanity check: enforce interface contract bounds before calculating total_pkt_len
                if channels != 4 or block_frames != 128 or num_vib > 32 or fs_val < 1000 or fs_val > 96000:
                    # Corrupted or false sync: advance past sync and continue scanning
                    del buf[:2]
                    continue

                vib_len = num_vib * 6
                audio_len = block_frames * channels * 2
                total_pkt_len = HEADER_SIZE_V2 + vib_len + audio_len + 2
                if len(buf) < total_pkt_len:
                    break

                # 16-bit XOR Checksum Validation across header, vibration payload, and audio payload
                expected_chk = struct.unpack_from("<H", buf, total_pkt_len - 2)[0]
                data_bytes = bytes(buf[0 : total_pkt_len - 2])
                chk = compute_packet_checksum(data_bytes)

                if (chk & 0xFFFF) != (expected_chk & 0xFFFF):
                    self.checksum_errors += 1
                    del buf[:2]
                    continue

                # Valid Type 0x05 Packet
                parsed_count += 1
                self.packets_valid += 1
                if self.last_seq is not None:
                    expected_seq = (self.last_seq + 1) & 0xFFFF
                    if seq != expected_seq:
                        drop = (seq - expected_seq) & 0xFFFF
                        if drop < 500:
                            self.dropped_packets += drop
                self.last_seq = seq
                self.last_packet_time = time.time()

                vib_bytes = bytes(buf[HEADER_SIZE_V2 : HEADER_SIZE_V2 + vib_len])
                audio_bytes = bytes(buf[HEADER_SIZE_V2 + vib_len : HEADER_SIZE_V2 + vib_len + audio_len])

                if num_vib > 0:
                    raw_vib = np.frombuffer(vib_bytes, dtype=np.int16).reshape((num_vib, 3)).astype(np.float32) / 1000.0
                    t_pkt = self.last_packet_time
                    dt_vib = 1.0 / 400.0
                    t_start = t_pkt - (num_vib - 1) * dt_vib
                    with self.vib_ring_lock:
                        if len(self.vib_ring_time) > 0:
                            last_t = self.vib_ring_time[-1]
                            # If batch is advancing (t_pkt < last_t) or last sample was recent (< 0.5s in past), enforce monotonicity
                            if (t_pkt - last_t) < 0.5:
                                t_start = max(t_start, last_t + dt_vib)
                        for k in range(num_vib):
                            self.vib_ring_time.append(t_start + k * dt_vib)
                            self.vib_ring_ax.append(float(raw_vib[k, 0]))
                            self.vib_ring_ay.append(float(raw_vib[k, 1]))
                            self.vib_ring_az.append(float(raw_vib[k, 2]))
                    latest_ax = float(raw_vib[-1, 0])
                    latest_ay = float(raw_vib[-1, 1])
                    latest_az = float(raw_vib[-1, 2])
                else:
                    latest_ax, latest_ay, latest_az = 0.0, 0.0, -0.82

                samples = np.frombuffer(audio_bytes, dtype=np.int16).reshape((block_frames, channels)).astype(np.float32) / 32768.0
                rms_ch = [float(np.sqrt(np.mean(samples[:, ch].astype(np.float64)**2))) for ch in range(channels)]
                active_channels = sum(1 for r in rms_ch if r > 0.0003)

                with self.lock:
                    if samples.shape == (block_frames, self.channels):
                        self.buffer = np.roll(self.buffer, -block_frames, axis=0)
                        self.buffer[-block_frames:, :] = samples
                    self.fs = float(fs_val)

                    now = time.time()
                    dt = now - self.last_pulse_time
                    if dt >= 0.5:
                        dp = pulses - self.last_pulse_count
                        if dp >= 0 and dt > 0:
                            self.telemetry["estimated_rpm"] = (dp / dt) * 60.0
                        self.last_pulse_count = pulses
                        self.last_pulse_time = now

                    adxl_hw_ok = bool(reserved & 0x01)
                    adxl_hw_str = "ADXL345: ONLINE (400Hz FIFO)" if adxl_hw_ok else "ADXL345: OFFLINE / ABSENT"

                    self.telemetry.update({
                        "port": str(port_str),
                        "baud": str(baud_str),
                        "ax_g": latest_ax,
                        "ay_g": latest_ay,
                        "az_g": latest_az,
                        "rpm_pulses": pulses,
                        "adxl_hw_ok": adxl_hw_ok,
                        "adxl_hw_str": adxl_hw_str,
                        "firmware_mode": f"4-CH REAL-TIME PCM DMA STREAM ({port_str} @ {baud_str}) [Type 0x05 400Hz FIFO]",
                        "is_hardware_alive": True,
                        "packets_valid": self.packets_valid,
                        "checksum_errors": self.checksum_errors,
                        "dropped_packets": self.dropped_packets,
                        "channels_active": active_channels,
                        "last_packet_time": self.last_packet_time,
                        "num_vib_last": num_vib,
                        "rms_counts": [float(r * 32768.0) for r in rms_ch],
                        "peak_counts": [int(np.max(np.abs(samples[:, ch])) * 32768.0) for ch in range(channels)]
                    })

                del buf[:total_pkt_len]

            elif pkt_type == 0x04:
                if len(buf) < HEADER_SIZE_V1:
                    break

                sync1, sync2, ptype, channels, seq, fs_val, block_frames, ax_mg, ay_mg, az_mg, pulses = struct.unpack_from(
                    HEADER_FORMAT_V1, buf, 0
                )

                # Defensive sanity check on legacy 0x04 packet bounds
                if channels != 4 or block_frames != 128 or fs_val < 1000 or fs_val > 96000:
                    del buf[:2]
                    continue

                audio_len = block_frames * channels * 2
                total_pkt_len = HEADER_SIZE_V1 + audio_len + 2
                if len(buf) < total_pkt_len:
                    break

                expected_chk = struct.unpack_from("<H", buf, HEADER_SIZE_V1 + audio_len)[0]
                hdr_bytes = bytes(buf[0 : HEADER_SIZE_V1])
                payload_bytes = bytes(buf[HEADER_SIZE_V1 : HEADER_SIZE_V1 + audio_len])

                chk = 0x55AA
                for i, b in enumerate(hdr_bytes):
                    chk ^= ((b << (i % 8)) & 0xFFFF)
                for i, b in enumerate(payload_bytes):
                    chk ^= ((b << (i % 8)) & 0xFFFF)

                if (chk & 0xFFFF) != (expected_chk & 0xFFFF):
                    self.checksum_errors += 1
                    del buf[:2]
                    continue

                parsed_count += 1
                self.packets_valid += 1
                if self.last_seq is not None:
                    expected_seq = (self.last_seq + 1) & 0xFFFF
                    if seq != expected_seq:
                        drop = (seq - expected_seq) & 0xFFFF
                        if drop < 500:
                            self.dropped_packets += drop
                self.last_seq = seq
                self.last_packet_time = time.time()

                t_pkt = self.last_packet_time
                with self.vib_ring_lock:
                    self.vib_ring_time.append(t_pkt)
                    self.vib_ring_ax.append(float(ax_mg) / 1000.0)
                    self.vib_ring_ay.append(float(ay_mg) / 1000.0)
                    self.vib_ring_az.append(float(az_mg) / 1000.0)

                latest_ax = float(ax_mg) / 1000.0
                latest_ay = float(ay_mg) / 1000.0
                latest_az = float(az_mg) / 1000.0

                samples = np.frombuffer(payload_bytes, dtype=np.int16).reshape((block_frames, channels)).astype(np.float32) / 32768.0
                rms_ch = [float(np.sqrt(np.mean(samples[:, ch].astype(np.float64)**2))) for ch in range(channels)]
                active_channels = sum(1 for r in rms_ch if r > 0.0003)

                with self.lock:
                    if samples.shape == (block_frames, self.channels):
                        self.buffer = np.roll(self.buffer, -block_frames, axis=0)
                        self.buffer[-block_frames:, :] = samples
                    self.fs = float(fs_val)

                    now = time.time()
                    dt = now - self.last_pulse_time
                    if dt >= 0.5:
                        dp = pulses - self.last_pulse_count
                        if dp >= 0 and dt > 0:
                            self.telemetry["estimated_rpm"] = (dp / dt) * 60.0
                        self.last_pulse_count = pulses
                        self.last_pulse_time = now

                    self.telemetry.update({
                        "ax_g": latest_ax,
                        "ay_g": latest_ay,
                        "az_g": latest_az,
                        "rpm_pulses": pulses,
                        "firmware_mode": f"4-CH REAL-TIME PCM DMA STREAM ({port_str} @ {baud_str}) [Type 0x04 Legacy]",
                        "is_hardware_alive": True,
                        "packets_valid": self.packets_valid,
                        "checksum_errors": self.checksum_errors,
                        "dropped_packets": self.dropped_packets,
                        "channels_active": active_channels,
                        "last_packet_time": self.last_packet_time,
                        "num_vib_last": 1,
                        "rms_counts": [float(r * 32768.0) for r in rms_ch],
                        "peak_counts": [int(np.max(np.abs(samples[:, ch])) * 32768.0) for ch in range(channels)]
                    })

                del buf[:total_pkt_len]

            else:
                # Unknown packet type or noise: advance 2 bytes past sync
                del buf[:2]
                continue

        return parsed_count

    def _worker(self):
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
                        self.telemetry["port"] = target_port
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

            self._rx_buf.clear()
            last_rx_time = time.time()

            while self.running and self.ser and self.ser.is_open:
                try:
                    n = self.ser.in_waiting
                    if n > 0:
                        chunk = self.ser.read(n)
                        last_rx_time = time.time()
                        self._parse_incoming_bytes(chunk, port_str=target_port, baud_str=self.ser.baudrate)
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
                # Hardware disconnected or standby: Return silent zeros.
                # NEVER invent synthetic moving audio in ESP32 hardware mode!
                return np.zeros((self.frame_len, self.channels), dtype=np.float32)
            return self.buffer.copy()

    def get_fs(self) -> float:
        return self.fs

    def get_channels(self) -> int:
        return self.channels

    def get_multimodal_telemetry(self) -> Dict[str, object]:
        with self.lock:
            return self.telemetry.copy()


# Backward compatibility alias
SerialAudioSource = ESP32MultimodalSource


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

        # Multi-harmonic rotating machinery audio signature with exact continuous sub-sample delays
        out = np.zeros((self.frame_len, 4), dtype=np.float32)
        for i in range(4):
            delay_s = (mic_x[i] * np.sin(angle_rad)) / SPEED_OF_SOUND
            t_i = t + delay_s
            sig_i = (
                0.60 * np.sin(2 * np.pi * 1200.0 * t_i)
                + 0.25 * np.sin(2 * np.pi * 2400.0 * t_i)
                + 0.10 * np.sin(2 * np.pi * 3600.0 * t_i)
                + 0.03 * np.random.randn(self.frame_len)
            ).astype(np.float32)
            out[:, i] = sig_i

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

    def __init__(self, port: str = "auto", baud: int = 921600):
        self.active_source_type = "ESP32"
        self.active_geometry_key = "ESP32_4MIC_LINEAR"
        self.geometry = PRESETS[self.active_geometry_key]

        self.sources: Dict[str, AudioSourceBase] = {
            "ESP32": ESP32MultimodalSource(port=port, baudrates=(baud, 921600, 1152000, 115200)),
            "SYNTHETIC": Synthetic4ChSource(),
            "LAPTOP": LaptopAudioSource()
        }
        self.current_source = self.sources[self.active_source_type]
        self.current_source.start()

        self.invert_channels = False
        self.tracker = DynamicTargetTracker(alpha=0.40, gate_deg=30.0)

        # Production Schmitt-Trigger VAD with Initial Ambient Calibration
        self.vad = AcousticActivityDetector(min_active_threshold=0.0035, calib_frames=25, hangover_sec=0.350)
        self.noise_floor = self.vad.noise_floor

        # Display State
        self.smoothed_heatmap = np.zeros((GRID_Y, GRID_X), dtype=np.float64)
        self.paused = False
        self.last_classifier_result = {
            "label": "CALIBRATING", "confidence": 0, "severity": "INITIALIZING", "detail": "Listening for acoustic sources..."
        }

        # I2S Dual-Pair Phase Calibration Delay (Seconds)
        self.calib_delay_sec = 0.0
        self.apply_calibration = False
        self.calib_status_str = "UNCALIBRATED (RAW DMA)"
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
                        is_healthy = bool(c_data.get("is_sync_healthy", False))
                        if "inter_pair_comp_delay_sec" in c_data:
                            self.calib_delay_sec = float(c_data["inter_pair_comp_delay_sec"])
                        elif "inter_pair_mean_skew_us" in c_data:
                            self.calib_delay_sec = float(c_data["inter_pair_mean_skew_us"]) * 1e-6

                        if is_healthy:
                            self.apply_calibration = True
                            self.calib_status_str = f"ACTIVE ({self.calib_delay_sec*1e6:+.2f} μs)"
                            print(f"[*] Acoustic Radar: Loaded HEALTHY Phase Calibration: Delay = {self.calib_delay_sec*1e6:+.2f} μs")
                        else:
                            self.apply_calibration = False
                            self.calib_status_str = f"BYPASSED (SKEWED: {self.calib_delay_sec*1e6:+.2f} μs, Raw DMA)"
                            print(f"[!] Warning: Phase calibration file reports is_sync_healthy=False ({c_data.get('status_diagnosis', 'SKEWED')}).")
                            print(f"    Calibration BYPASSED by default (Raw DMA alignment used). Press 'K' to toggle.")
                        break
                except Exception as e:
                    print(f"[!] Warning reading phase calibration from {c_path}: {e}")

        # Multimodal Motor Diagnostics Engine & Sensor Fusion
        self.motor_engine = MultimodalMotorEngine()
        self.learn_baseline_requested = False
        self.pulses_options = [1, 2, 4, 10, 20]
        self.pulses_per_rev_idx = 0
        self.pulses_per_rev = 1

        # Interactive Motor Demo / Test Profiles (Key 'T' to cycle)
        self.demo_profile_idx = 0
        self.demo_profiles = [
            "LIVE_HARDWARE",
            "DEMO_HEALTHY_1800RPM",
            "DEMO_UNBALANCE_1X",
            "DEMO_MISALIGNMENT_2X",
            "DEMO_DISTURBANCE_PROOF"
        ]
        self.demo_phase = 0.0
        self.demo_pulses = 120
        self._last_seen_pulses = -1
        self._last_pulse_blink_time = 0.0

        self._setup_ui()

    def _get_evidence_mode_tag(self) -> str:
        """Returns concise evidence mode tag that never overflows the top header."""
        if getattr(self, "demo_profile_idx", 0) > 0:
            return f"DEMO MODE: {self.demo_profiles[self.demo_profile_idx]} (Press 'T' for Live)"
        if self.active_source_type == "ESP32":
            telemetry = self.current_source.get_multimodal_telemetry()
            is_alive = telemetry.get("is_hardware_alive", False)
            pkt_valid = telemetry.get("packets_valid", 0)
            chk_err = telemetry.get("checksum_errors", 0)
            ch_active = telemetry.get("channels_active", 0)
            last_pkt = telemetry.get("last_packet_time", 0.0)
            port_label = str(telemetry.get("port", getattr(self.current_source, "port", "ESP32")))
            if port_label == "auto":
                port_label = "ESP32"

            if is_alive and (time.time() - last_pkt < 1.5) and pkt_valid > 5:
                if ch_active >= 4:
                    return f"LIVE HARDWARE: {port_label} @ 921.6k | 4-MIC ULA + ADXL345 + FC-03 (CHK OK)"
                else:
                    return f"LIVE HARDWARE: {port_label} ({ch_active}/4 MICS ACTIVE | CHK OK)"
            elif is_alive and chk_err > 0:
                return f"HARDWARE LINK WARNING ({chk_err} CHECKSUM ERRORS)"
            else:
                return f"ESP32 STANDBY / DISCONNECTED ({port_label}) (Press 'T' for Demo)"
        elif self.active_source_type == "SYNTHETIC":
            return "SYNTHETIC 4-CH ARRAY & MOTOR SIMULATION"
        elif self.active_source_type == "LAPTOP":
            return "LAPTOP INTERNAL MICROPHONE ARRAY DEMO"
        return f"SOURCE: {self.active_source_type}"

    def _setup_ui(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(19.2, 10.2), facecolor="#070b12")

        try:
            if hasattr(self.fig.canvas, "manager") and self.fig.canvas.manager:
                self.fig.canvas.manager.set_window_title(
                    "Spatial Multimodal Edge-AI | Rotating Machinery Fault Detection & Localization (v6)"
                )
        except Exception:
            pass

        # 2 rows x 4 columns balanced layout with generous margins to prevent clipping/overlap
        gs = gridspec.GridSpec(
            2, 4,
            width_ratios=[1.08, 0.92, 0.92, 1.08],
            height_ratios=[1.0, 1.0],
            hspace=0.35, wspace=0.29,
            left=0.048, right=0.985, top=0.885, bottom=0.055
        )

        # ---------------------------------------------------------------------
        # 1. 2D Spatial Heatmap / Acoustic Camera (Col 0: gs[:, 0])
        # ---------------------------------------------------------------------
        self.ax_map = self.fig.add_subplot(gs[:, 0], facecolor="#0d1422")
        self.im_heat = self.ax_map.imshow(
            self.smoothed_heatmap,
            origin="lower",
            extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
            aspect="auto",
            vmin=0.0, vmax=1.0,
            cmap=ACOUSTIC_HEAT_CMAP,
            interpolation="bicubic"
        )

        # Horizontal colorbar at bottom of Col 0 -> ZERO collision with Col 1 labels!
        self.cb = self.fig.colorbar(
            self.im_heat, ax=self.ax_map, orientation="horizontal",
            fraction=0.036, pad=0.09, aspect=28
        )
        self.cb.set_label("Relative Acoustic Intensity Above Ambient (not dB SPL)", color="#94a3b8", fontsize=8.5)
        self.cb.ax.tick_params(colors="#94a3b8", labelsize=8)

        # Dynamic Side Intensity Indicators (Shortened to y=[0.45, 2.50] so top is clutter-free)
        self.left_gauge_bg, = self.ax_map.plot(
            [-1.84, -1.84], [0.45, 2.50],
            color="#141f33", linewidth=10, solid_capstyle="round", zorder=7
        )
        self.left_gauge_fill, = self.ax_map.plot(
            [-1.84, -1.84], [0.45, 0.45],
            color="#00ffcc", linewidth=10, solid_capstyle="round", zorder=8
        )
        self.left_gauge_text = self.ax_map.text(
            -1.84, 2.60, "◄ L\n0%",
            color="#00ffcc", fontsize=8.0, fontweight="bold", ha="center", va="bottom", zorder=9,
            bbox=dict(boxstyle="round,pad=0.18", facecolor="#070b12", edgecolor="none", alpha=0.75)
        )

        self.right_gauge_bg, = self.ax_map.plot(
            [1.84, 1.84], [0.45, 2.50],
            color="#141f33", linewidth=10, solid_capstyle="round", zorder=7
        )
        self.right_gauge_fill, = self.ax_map.plot(
            [1.84, 1.84], [0.45, 0.45],
            color="#00ffcc", linewidth=10, solid_capstyle="round", zorder=8
        )
        self.right_gauge_text = self.ax_map.text(
            1.84, 2.60, "R ►\n0%",
            color="#00ffcc", fontsize=8.0, fontweight="bold", ha="center", va="bottom", zorder=9,
            bbox=dict(boxstyle="round,pad=0.18", facecolor="#070b12", edgecolor="none", alpha=0.75)
        )

        # Incoming Sound Sector HUD Pill Banner (Isolated cleanly at top of camera at y=3.32)
        self.sector_banner_text = self.ax_map.text(
            0.0, 3.33, "SCANNING ACOUSTIC FIELD",
            color="#8da2c0", fontsize=8.5, fontweight="bold", ha="center", va="center", zorder=10,
            bbox=dict(boxstyle="round,pad=0.32", facecolor="#070b12", edgecolor="#2a3b5c", alpha=0.92, lw=1.2)
        )

        # Array Microphones Scatter with individual labels
        self.scatter_mics = self.ax_map.scatter(
            self.geometry.mic_coords[:, 0], self.geometry.mic_coords[:, 1],
            marker="^", color="#00ffcc", s=110, edgecolor="#ffffff", linewidth=1.2,
            label="4-Mic ULA (INMP441)", zorder=5
        )

        self.mic_labels = []
        for i in range(self.geometry.num_mics):
            txt = self.ax_map.text(
                self.geometry.mic_coords[i, 0], self.geometry.mic_coords[i, 1] - 0.12,
                f"M{i+1}", color="#00ffcc", fontsize=8, fontweight="bold",
                ha="center", va="top", zorder=6
            )
            self.mic_labels.append(txt)

        # Estimated Peak Target Crosshair
        self.marker_source, = self.ax_map.plot(
            [0], [1.25],
            marker="o", markersize=15, markerfacecolor="none",
            markeredgecolor="#00ffcc", markeredgewidth=2.8,
            label="DOA Target Ray", zorder=6
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
                color="#24334d", linestyle="--", linewidth=0.9, zorder=2
            )
            self.ax_map.text(
                0.06, r_m + 0.04, f"{r_m:.0f}m",
                color="#455b80", fontsize=8, family="monospace"
            )

        for ang_deg in [-60, -30, 0, 30, 60]:
            ang_rad = np.deg2rad(ang_deg)
            self.ax_map.plot(
                [0, 3.4 * np.sin(ang_rad)], [0, 3.4 * np.cos(ang_rad)],
                color="#1c283d", linestyle=":", linewidth=0.9, zorder=2
            )

        self.ax_map.set_xlim(X_MIN, X_MAX)
        self.ax_map.set_ylim(Y_MIN, Y_MAX)
        self.ax_map.set_xlabel("Lateral Offset X (m)", color="#b8c2d1", fontsize=9.5)
        self.ax_map.set_ylabel("Forward Standoff Range Y (m) [DOA Ray]", color="#b8c2d1", fontsize=9.5)
        self.ax_map.set_title("1. 2D ACOUSTIC CAMERA (WHERE IS IT?)", color="#00ffcc", fontsize=10, pad=8, weight="bold")
        self.ax_map.tick_params(colors="#b8c2d1", labelsize=8.5)
        # Legend placed at lower center away from the top banner!
        self.ax_map.legend(loc="lower center", ncol=2, facecolor="#070b12", edgecolor="#2a3b5c", fontsize=7.8, framealpha=0.90)
        self.ax_map.grid(alpha=0.15, color="#34496d")

        # ---------------------------------------------------------------------
        # 2. Polar Radar Sweep (Col 1 Top: gs[0, 1])
        # ---------------------------------------------------------------------
        self.ax_radar = self.fig.add_subplot(gs[0, 1], projection="polar", facecolor="#0d1422")
        self.radar_angles = np.deg2rad(np.linspace(-90, 90, 181))
        self.radar_line, = self.ax_radar.plot(self.radar_angles, np.zeros_like(self.radar_angles), color="#00ffcc", linewidth=2.0)

        self.meas_needle, = self.ax_radar.plot([0, 0], [0, 0.85], color="#ff7700", linewidth=2.0, linestyle="--", label="Instant DOA")
        self.radar_needle, = self.ax_radar.plot([0, 0], [0, 1.0], color="#00ffcc", linewidth=3.2, label="Kalman Track")

        self.ax_radar.set_theta_zero_location("N")
        self.ax_radar.set_theta_direction(-1)
        self.ax_radar.set_thetamin(-90)
        self.ax_radar.set_thetamax(90)
        self.ax_radar.set_title("2. POLAR BEARING RADAR (180°)", color="#ffffff", fontsize=10, weight="bold", pad=10)
        self.ax_radar.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_radar.grid(color="#24334d", alpha=0.5)
        self.ax_radar.legend(loc="lower right", facecolor="#070b12", edgecolor="#2a3b5c", fontsize=7.2)

        # ---------------------------------------------------------------------
        # 3. ALL 4 MICROPHONE LIVE INPUTS (Col 1 Bottom: gs[1, 1])
        # ---------------------------------------------------------------------
        self.ax_mics = self.fig.add_subplot(gs[1, 1], facecolor="#0b111d")
        self.mic_colors = ["#00ffcc", "#ffaa00", "#ff0077", "#33ccff"]
        self.mic_names = [
            "M1: Pair A Left  (GPIO 19)",
            "M2: Pair A Right (GPIO 19)",
            "M3: Pair B Left  (GPIO 23)",
            "M4: Pair B Right (GPIO 23)"
        ]

        self.wave_lines = []
        for i in range(4):
            line, = self.ax_mics.plot([], [], color=self.mic_colors[i], linewidth=1.15, label=f"Mic {i+1}")
            self.wave_lines.append(line)

        self.ax_mics.set_xlim(0, 512)
        self.ax_mics.set_ylim(-4.3, 4.3)
        self.ax_mics.set_title("3. 4-MIC INMP441 ARRAY WAVEFORMS", color="#ffffff", fontsize=10, weight="bold", pad=8)
        self.ax_mics.set_xlabel("Audio Frame Samples (DMA Block)", color="#b8c2d1", fontsize=8.5)
        self.ax_mics.set_ylabel("Channels [M1..M4]", color="#b8c2d1", fontsize=8.5)
        self.ax_mics.set_yticks([-3.0, -1.0, 1.0, 3.0])
        self.ax_mics.set_yticklabels(["M4 (B-R)", "M3 (B-L)", "M2 (A-R)", "M1 (A-L)"], fontsize=8, color="#b8c2d1")
        self.ax_mics.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_mics.grid(alpha=0.15, color="#34496d")

        self.vu_texts = []
        for i in range(4):
            y_pos = 3.0 - i * 2.0
            txt = self.ax_mics.text(
                504, y_pos + 0.36, "RMS: 0.0000",
                color=self.mic_colors[i], fontsize=7.3, family="monospace", ha="right", va="bottom",
                bbox=dict(boxstyle="round,pad=0.18", facecolor="#070b12", edgecolor=self.mic_colors[i], alpha=0.88)
            )
            self.vu_texts.append(txt)

        # ---------------------------------------------------------------------
        # 4. SRP-PHAT Delay Profile & Angular Spectrum (Col 2 Top: gs[0, 2])
        # ---------------------------------------------------------------------
        self.ax_srp = self.fig.add_subplot(gs[0, 2], facecolor="#0d1422")
        self.srp_line, = self.ax_srp.plot([], [], color="#00ffcc", linewidth=2.0, label="SRP-PHAT (6 Pairs)")
        self.srp_peak_marker, = self.ax_srp.plot([], [], color="#ff0055", marker="o", markersize=8, label="Estimated DOA")
        self.ax_srp.set_xlim(-90, 90)
        self.ax_srp.set_ylim(-0.05, 1.15)
        self.ax_srp.set_title("4. MULTI-PAIR SRP-PHAT (6 PAIRS)", color="#ffffff", fontsize=10, weight="bold", pad=8)
        self.ax_srp.set_xlabel("Bearing Angle θ (deg)", color="#b8c2d1", fontsize=8.5)
        self.ax_srp.set_ylabel("Normalized SRP Power", color="#b8c2d1", fontsize=8.5)
        self.ax_srp.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_srp.grid(alpha=0.15, color="#34496d")
        self.ax_srp.legend(loc="upper right", facecolor="#070b12", edgecolor="#2a3b5c", fontsize=7.2)

        # ---------------------------------------------------------------------
        # 5. LIVE ADXL345 3-AXIS VIBRATION SCOPE (Col 2 Bottom: gs[1, 2])
        # ---------------------------------------------------------------------
        self.ax_vib = self.fig.add_subplot(gs[1, 2], facecolor="#0b111d")
        self.vib_line_x, = self.ax_vib.plot([], [], color="#ff9226", linewidth=1.05, label="a_x (X)")
        self.vib_line_y, = self.ax_vib.plot([], [], color="#38bdf8", linewidth=1.05, label="a_y (Y)")
        self.vib_line_z, = self.ax_vib.plot([], [], color="#e879f9", linewidth=1.05, label="a_z (Z)")
        self.vib_line_dyn, = self.ax_vib.plot([], [], color="#00ffcc", linewidth=1.9, label="a_dyn (RMS)")

        self.ax_vib.axhline(0.15, color="#00ffcc", linestyle="--", alpha=0.45, linewidth=0.9)
        self.ax_vib.axhline(0.35, color="#38bdf8", linestyle="--", alpha=0.45, linewidth=0.9)
        self.ax_vib.axhline(0.70, color="#f59e0b", linestyle="--", alpha=0.45, linewidth=0.9)

        self.ax_vib.text(0.98, 0.56, "Zone A (<0.15g)", transform=self.ax_vib.transAxes, color="#00ffcc", fontsize=7.0, ha="right", alpha=0.85)
        self.ax_vib.text(0.98, 0.64, "Zone B (<0.35g)", transform=self.ax_vib.transAxes, color="#38bdf8", fontsize=7.0, ha="right", alpha=0.85)
        self.ax_vib.text(0.98, 0.78, "Zone C (0.70g)", transform=self.ax_vib.transAxes, color="#f59e0b", fontsize=7.0, ha="right", alpha=0.85)

        self.ax_vib.set_xlim(0, 256)
        self.ax_vib.set_ylim(-1.25, 1.25)
        self.ax_vib.set_title("5. ADXL345 3-AXIS VIBRATION (400 Hz)", color="#ffffff", fontsize=10, weight="bold", pad=8)
        self.ax_vib.set_xlabel("Vibration FIFO Samples (400 Hz ODR)", color="#b8c2d1", fontsize=8.5)
        self.ax_vib.set_ylabel("Acceleration (g)", color="#b8c2d1", fontsize=8.5)
        self.ax_vib.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_vib.grid(alpha=0.15, color="#34496d")
        self.ax_vib.legend(loc="upper left", facecolor="#070b12", edgecolor="#2a3b5c", fontsize=7.0, ncol=2)

        self.vib_badge_text = self.ax_vib.text(
            0.98, 0.04, "a_dyn: 0.000g | ISO Vel: 0.00 mm/s",
            transform=self.ax_vib.transAxes,
            color="#00ffcc", fontsize=7.5, family="monospace", ha="right", va="bottom",
            bbox=dict(boxstyle="round,pad=0.22", facecolor="#070b12", edgecolor="#00ffcc", alpha=0.90)
        )

        # ---------------------------------------------------------------------
        # 6. ROTATIONAL ORDER SPECTRUM & TACHOMETER (Col 3 Top: gs[0, 3])
        # ---------------------------------------------------------------------
        self.ax_order = self.fig.add_subplot(gs[0, 3], facecolor="#0b111d")

        # Digital Speedometer Readout (Sized to fit strictly inside Col 3)
        self.speedometer_text = self.ax_order.text(
            0.50, 0.90, "SPEED: 0 RPM (f0 = 0.0 Hz)",
            transform=self.ax_order.transAxes,
            ha="center", va="center", color="#00ffcc", fontsize=9.2, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.28", facecolor="#101b2d", edgecolor="#00ffcc", linewidth=1.3)
        )

        # FC-03 Pulse Blinker LED
        self.tach_indicator_text = self.ax_order.text(
            0.50, 0.76, "○ FC-03 TACH: READY (0 Pulses)",
            transform=self.ax_order.transAxes,
            ha="center", va="center", color="#8da2c0", fontsize=8.0, fontweight="bold", family="monospace"
        )

        bar_x = [0, 1, 2]
        self.order_bars = self.ax_order.bar(
            bar_x, [0.0, 0.0, 0.0],
            width=0.52,
            color=["#00ffcc", "#38bdf8", "#e879f9"],
            edgecolor="#ffffff", linewidth=1.0, zorder=3
        )
        self.ax_order.set_xticks(bar_x)
        self.ax_order.set_xticklabels(["1X (Unbalance)", "2X (Misalign)", "3X (Harmonic)"], fontsize=8.2, color="#b8c2d1", fontweight="bold")
        self.ax_order.set_ylim(0, 118)
        self.ax_order.set_ylabel("Rotational Energy (%)", color="#b8c2d1", fontsize=8.5)
        self.ax_order.set_title("6. SHAFT RPM & ROTATIONAL ORDERS", color="#ffffff", fontsize=10, weight="bold", pad=8)
        self.ax_order.tick_params(colors="#b8c2d1", labelsize=8)
        self.ax_order.grid(alpha=0.15, color="#34496d", axis="y")

        self.bar_labels = [
            self.ax_order.text(i, 3.0, "0.0%", ha="center", va="bottom", fontsize=8.2, fontweight="bold", color="#ffffff")
            for i in range(3)
        ]

        # ---------------------------------------------------------------------
        # 7. MULTIMODAL EDGE-AI FUSION & MINOR PROJECT VISION HUD (Col 3 Bottom: gs[1, 3])
        # ---------------------------------------------------------------------
        self.ax_telemetry = self.fig.add_subplot(gs[1, 3], facecolor="#0b111d")
        self.ax_telemetry.set_title("7. MULTIMODAL FUSION (WHAT & WHERE)", color="#00ffcc", fontsize=10, weight="bold", pad=8)
        self.ax_telemetry.set_xticks([])
        self.ax_telemetry.set_yticks([])
        for spine in self.ax_telemetry.spines.values():
            spine.set_edgecolor("#1e293b")
            spine.set_linewidth(1.2)

        # Card 1: WHAT IS WRONG? (Mechanical Condition)
        self.what_box_text = self.ax_telemetry.text(
            0.50, 0.91,
            "WHAT IS WRONG? (MECHANICAL STATE)\nINITIALIZING...",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color="#00ffcc", fontsize=8.0, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.30", facecolor="#101b2d", edgecolor="#00ffcc", linewidth=1.3)
        )

        # Card 2: WHERE IS IT? (Spatial Localization & Fusion)
        self.where_box_text = self.ax_telemetry.text(
            0.50, 0.73,
            "WHERE IS THE SOUND? (SPATIAL DOA)\nSCANNING...",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color="#38bdf8", fontsize=8.0, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.28", facecolor="#101b2d", edgecolor="#38bdf8", linewidth=1.2)
        )

        # Structured Telemetry & Hotkey Block (Strictly width-bounded <46 chars so zero clipping!)
        self.hud_text = self.ax_telemetry.text(
            0.03, 0.60, "Acquiring Multimodal Telemetry...",
            transform=self.ax_telemetry.transAxes,
            va="top", ha="left", color="#e2e8f0", fontsize=7.3, family="monospace", linespacing=1.18
        )

        # ---------------------------------------------------------------------
        # Top 2-Tier Minor Project Vision Header
        # ---------------------------------------------------------------------
        self.fig.text(
            0.50, 0.972,
            "SPATIAL MULTIMODAL EDGE-AI FOR FAULT DETECTION & LOCALIZATION OF ROTATING MACHINERY",
            ha="center", va="center", color="#00ffcc", fontsize=12.0, weight="bold"
        )
        self.title_text = self.fig.text(
            0.50, 0.938,
            f"MINOR PROJECT VISION: [VIBRATION + 4-MIC ARRAY + RPM]  ►  ANSWERS 'WHAT IS WRONG?' & 'WHERE IS IT?'  |  [{self._get_evidence_mode_tag()}]",
            ha="center", va="center", color="#f8fafc", fontsize=8.8, weight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.28", facecolor="#101b2d", edgecolor="#2a3b5c", linewidth=1.2)
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
        elif event.key in ("t", "T"):
            self.demo_profile_idx = (self.demo_profile_idx + 1) % len(self.demo_profiles)
            p_name = self.demo_profiles[self.demo_profile_idx]
            print(f"[+] Motor Test Profile Cycled: {p_name} ({self.demo_profile_idx}/{len(self.demo_profiles)-1})")
        elif event.key in ("b", "B"):
            self.learn_baseline_requested = True
            print("[+] Motor Golden Baseline Capture Triggered! Learning reference signature...")
        elif event.key in ("c", "C"):
            self.motor_engine.reset_baseline()
            print("[+] Motor Golden Baseline Cleared / Reset.")
        elif event.key in ("n", "N"):
            self.calibrate_ambient_noise()
        elif event.key in ("k", "K"):
            self.apply_calibration = not self.apply_calibration
            tag = f"ACTIVE ({self.calib_delay_sec*1e6:+.2f} μs)" if self.apply_calibration else "BYPASSED (RAW DMA)"
            self.calib_status_str = tag
            print(f"[+] Phase Calibration Toggled: {tag}")
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

    def calibrate_ambient_noise(self):
        """Triggers re-calibration of the ambient room noise floor."""
        self.vad.start_calibration()
        print("[*] Re-calibrating ambient quiet-room noise floor (listening 25 frames)...")

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
        all_artists = [
            self.im_heat, self.marker_source, self.target_line,
            self.radar_line, self.radar_needle, self.meas_needle,
            self.srp_line, self.srp_peak_marker, self.hud_text,
            self.left_gauge_bg, self.left_gauge_fill, self.left_gauge_text,
            self.right_gauge_bg, self.right_gauge_fill, self.right_gauge_text,
            self.sector_banner_text,
            self.vib_line_x, self.vib_line_y, self.vib_line_z, self.vib_line_dyn,
            self.vib_badge_text,
            self.speedometer_text, self.tach_indicator_text,
            self.what_box_text, self.where_box_text, self.title_text
        ] + list(self.order_bars) + self.bar_labels + self.wave_lines + self.vu_texts

        if self.paused:
            return all_artists

        try:
            raw_frame = self.current_source.get_frame()
            fs = self.current_source.get_fs()
            channels = raw_frame.shape[1] if raw_frame.ndim > 1 else 1

            if self.invert_channels:
                raw_frame = np.fliplr(raw_frame)

            # Apply I2S phase calibration compensation to Pair B ONLY if enabled
            if self.apply_calibration and abs(self.calib_delay_sec) > 1e-12 and raw_frame.shape[1] >= 4:
                raw_frame = apply_pair_phase_compensation(raw_frame, fs, self.calib_delay_sec)

            # 1. Update 4-Channel Live Waveforms & Sensitivity-Calibrated VU Badges
            # Create a separate floating-point calibrated amplitude array:
            # Raw audio (raw_frame) is strictly preserved unscaled for GCC-PHAT & SRP-PHAT phase correlation.
            gains_vec = PRELIM_MIC_SENSITIVITY_GAINS[:channels].reshape(1, -1)
            calib_audio_4ch = raw_frame * gains_vec

            n_samples = min(raw_frame.shape[0], 512)
            x_axis = np.arange(n_samples)
            rms_per_ch = []
            rms_calib_per_ch = []
            pk_per_ch = []

            for i in range(4):
                if i < channels:
                    sig_ch = raw_frame[:n_samples, i]
                    sig_ac = sig_ch - np.mean(sig_ch)
                    r_val = float(np.sqrt(np.mean(sig_ac ** 2)))
                    p_val = float(np.max(np.abs(sig_ac)))
                    rms_per_ch.append(r_val)
                    pk_per_ch.append(p_val)

                    gain = float(PRELIM_MIC_SENSITIVITY_GAINS[i]) if i < len(PRELIM_MIC_SENSITIVITY_GAINS) else 1.0
                    r_calib = r_val * gain
                    rms_calib_per_ch.append(r_calib)

                    # Offset vertically for clean stacking: M1 at +3, M2 at +1, M3 at -1, M4 at -3
                    y_offset = 3.0 - i * 2.0
                    scaled_wave = np.clip(sig_ac * 1.8, -0.85, 0.85) + y_offset
                    self.wave_lines[i].set_data(x_axis, scaled_wave)

                    dbfs_calib = 20.0 * math.log10(max(r_calib, 1e-5))
                    status_str = "ACTIVE" if (r_val > 0.0018 or r_calib > 0.0030) else "QUIET "
                    # Display calibrated RMS while retaining raw RMS for diagnostic inspection
                    self.vu_texts[i].set_text(f"M{i+1}: {r_calib:.4f} ({dbfs_calib:+.1f}dB) | {status_str}")
                else:
                    self.wave_lines[i].set_data([], [])
                    self.vu_texts[i].set_text(f"M{i+1}: [DISABLED]")

            avg_rms = float(np.mean(rms_per_ch)) if rms_per_ch else 0.0
            avg_calib_rms = float(np.mean(rms_calib_per_ch)) if rms_calib_per_ch else avg_rms
            now = time.time()

            # RMS VAD gives ambient status, but a contaminated startup baseline must not
            # veto SRP's independent spatial-coherence and GCC-PHAT source validation.
            is_active_sound, self.noise_floor, attack_thresh = self.vad.process_frame(avg_rms, now)

            # Adaptive Physical SRP Energy Gate:
            # Rejects quiescent room floor while allowing quiet motor acoustic emissions
            min_srp_rms = max(0.0008, min(0.0022, self.noise_floor * 1.25))
            best_bearing, composite_prom, srp_angles, srp_norm, raw_2d_heatmap, side_metrics = (
                SRPPhatEngine.compute_srp_frame(raw_frame, fs, self.geometry, min_active_rms=min_srp_rms, calib_gains=PRELIM_MIC_SENSITIVITY_GAINS)
            )

            is_audio_energy_present = bool(avg_calib_rms >= min_srp_rms)
            is_source_valid = bool(side_metrics.get("is_valid", False))
            if is_source_valid:
                self.vad.note_coherent_source(now)
                self.noise_floor = self.vad.noise_floor
                is_active_sound = True
            else:
                # Energy is present, but do not invent a bearing or hotspot.
                is_active_sound = bool(is_active_sound or is_audio_energy_present)

            # 3. Dynamic Target Tracker
            target_angle, target_range, confidence, is_locked = self.tracker.update(
                best_bearing, composite_prom, is_source_valid, nominal_range=1.25
            )

            target_rad = math.radians(target_angle)
            target_x = target_range * math.sin(target_rad)
            target_y = target_range * math.cos(target_rad)

            # 4. True 2D Acoustic Camera Thermal Heatmap (Real Backprojected Energy)
            if is_source_valid:
                acoustic_heat = SRPPhatEngine.compute_true_2d_acoustic_heatmap(
                    raw_2d_heatmap, target_range, avg_rms, is_locked, self.geometry, self.noise_floor
                )
                self.smoothed_heatmap = (
                    HEATMAP_SMOOTH_ALPHA * self.smoothed_heatmap
                    + (1.0 - HEATMAP_SMOOTH_ALPHA) * acoustic_heat
                )
            else:
                self.smoothed_heatmap *= 0.65

            self.im_heat.set_data(self.smoothed_heatmap)

            # Needles & Target Line
            if is_source_valid and is_locked:
                marker_color = "#00ffcc"
                self.marker_source.set_data([target_x], [target_y])
                self.marker_source.set_markeredgecolor(marker_color)
                self.target_line.set_data([0, target_x], [0, target_y])
                self.target_line.set_color(marker_color)
                meas_rad = math.radians(best_bearing)
                self.meas_needle.set_data([meas_rad, meas_rad], [0, 0.85])
                self.radar_needle.set_data([target_rad, target_rad], [0, 1.0])
                self.radar_needle.set_color(marker_color)
            elif is_source_valid:
                marker_color = "#ffaa00"
                self.marker_source.set_data([target_x], [target_y])
                self.marker_source.set_markeredgecolor(marker_color)
                self.target_line.set_data([0, target_x], [0, target_y])
                self.target_line.set_color(marker_color)
                meas_rad = math.radians(best_bearing)
                self.meas_needle.set_data([meas_rad, meas_rad], [0, 0.85])
                self.radar_needle.set_data([target_rad, target_rad], [0, 0.65])
                self.radar_needle.set_color(marker_color)
            else:
                # Standby Monitoring: Keep needles and target line visible in high-contrast standby slate
                marker_color = "#2a4365"
                self.marker_source.set_data([target_x], [target_y])
                self.marker_source.set_markeredgecolor(marker_color)
                self.target_line.set_data([0, target_x], [0, target_y])
                self.target_line.set_color("#1c2e4a")
                meas_rad = math.radians(best_bearing)
                self.meas_needle.set_data([meas_rad, meas_rad], [0, 0.50])
                self.radar_needle.set_data([target_rad, target_rad], [0, 0.50])
                self.radar_needle.set_color(marker_color)

            # 4b. Dynamic Sound Intensity Side Indicators (Left / Center / Right)
            pct_l = float(side_metrics.get("pct_left", 33.3))
            pct_c = float(side_metrics.get("pct_center", 33.3))
            pct_r = float(side_metrics.get("pct_right", 33.3))
            dom_s = str(side_metrics.get("dom_side", "NONE"))

            if is_source_valid:
                # Use calibrated RMS for physical acoustic intensity rails
                rms_rel = max(0.0, avg_calib_rms - self.noise_floor) / max(0.015, self.noise_floor * 3.0)
                rms_scale = min(1.0, math.sqrt(min(1.0, max(0.05, rms_rel))))
                intensity_l = float(min(1.0, (pct_l / 100.0) * rms_scale * 1.35))
                intensity_r = float(min(1.0, (pct_r / 100.0) * rms_scale * 1.35))
                intensity_c = float(min(1.0, (pct_c / 100.0) * rms_scale * 1.35))
            else:
                intensity_l = 0.02
                intensity_r = 0.02
                intensity_c = 0.02

            y_bot, y_top = 0.45, 2.50
            y_span = y_top - y_bot

            # Left VU rail
            h_left = y_bot + y_span * intensity_l
            self.left_gauge_fill.set_data([-1.84, -1.84], [y_bot, h_left])
            c_left = "#ff0055" if (pct_l >= 48.0 and is_source_valid) else ("#ffaa00" if (pct_l >= 36.0 and is_source_valid) else "#00ffcc")
            self.left_gauge_fill.set_color(c_left)
            self.left_gauge_text.set_color(c_left)
            self.left_gauge_text.set_text(f"◄ L\n{pct_l:.0f}%" if is_source_valid else "◄ L\nIDLE")

            # Right VU rail
            h_right = y_bot + y_span * intensity_r
            self.right_gauge_fill.set_data([1.84, 1.84], [y_bot, h_right])
            c_right = "#ff0055" if (pct_r >= 48.0 and is_source_valid) else ("#ffaa00" if (pct_r >= 36.0 and is_source_valid) else "#00ffcc")
            self.right_gauge_fill.set_color(c_right)
            self.right_gauge_text.set_color(c_right)
            self.right_gauge_text.set_text(f"R ►\n{pct_r:.0f}%" if is_source_valid else "R ►\nIDLE")

            # Incoming Sector HUD Banner
            if not is_source_valid and is_active_sound:
                reject_reason = str(side_metrics.get("status", "GATE"))
                self.sector_banner_text.set_text(f"SOUND DETECTED — UNRESOLVED ({reject_reason})")
                self.sector_banner_text.set_color("#ffaa00")
            elif not is_source_valid:
                self.sector_banner_text.set_text("AMBIENT MONITORING — READY FOR SOUND")
                self.sector_banner_text.set_color("#8da2c0")
            elif dom_s == "LEFT":
                self.sector_banner_text.set_text(f"◄◄ LEFT FLANK ({target_angle:+.1f}° | {pct_l:.0f}% POWER)")
                self.sector_banner_text.set_color("#ff0055" if pct_l >= 48.0 else "#ffaa00")
            elif dom_s == "RIGHT":
                self.sector_banner_text.set_text(f"RIGHT FLANK ({target_angle:+.1f}° | {pct_r:.0f}% POWER) ►►")
                self.sector_banner_text.set_color("#ff0055" if pct_r >= 48.0 else "#ffaa00")
            else:
                self.sector_banner_text.set_text(f"▲ FRONT CENTER ({target_angle:+.1f}° | {pct_c:.0f}% POWER) ▲")
                self.sector_banner_text.set_color("#00ffcc")

            # 5. Polar Radar Sweep
            polar_angles_deg = np.linspace(-90, 90, 181)
            if is_source_valid:
                target_conf = confidence if is_locked else confidence * 0.40
                ang_diff = polar_angles_deg - target_angle
                radar_profile = np.exp(-0.5 * (ang_diff / 8.0) ** 2) * (target_conf / 100.0)
                self.radar_line.set_data(self.radar_angles, radar_profile)
                self.radar_line.set_color("#00ffcc" if is_locked else "#ffaa00")
            else:
                ang_diff = polar_angles_deg - target_angle
                radar_profile = np.exp(-0.5 * (ang_diff / 16.0) ** 2) * 0.18
                self.radar_line.set_data(self.radar_angles, radar_profile)
                self.radar_line.set_color("#2a4365")

            # 6. SRP-PHAT Angular Profile Plot
            if len(srp_norm) > 0 and np.max(srp_norm) > 0.001:
                self.srp_line.set_data(srp_angles, srp_norm)
                peak_idx = int(np.argmin(np.abs(srp_angles - best_bearing)))
                self.srp_peak_marker.set_data([best_bearing], [srp_norm[peak_idx]])
                if is_source_valid:
                    self.srp_line.set_color("#00ffcc")
                    self.srp_peak_marker.set_color("#ff0055")
                else:
                    self.srp_line.set_color("#3b82f6")
                    self.srp_peak_marker.set_color("#475569")
            else:
                self.srp_line.set_data(srp_angles, np.zeros_like(srp_angles))
                self.srp_peak_marker.set_data([], [])

            # 7. Multimodal DC Motor Diagnostics & Sensor Fusion
            telemetry = self.current_source.get_multimodal_telemetry()
            ax_g = float(telemetry.get("ax_g", 0.0))
            ay_g = float(telemetry.get("ay_g", 0.0))
            az_g = float(telemetry.get("az_g", -0.82))
            pulses = int(telemetry.get("rpm_pulses", 0))
            raw_rpm = float(telemetry.get("estimated_rpm", 0.0))
            rpm = raw_rpm / max(1, self.pulses_per_rev)
            fw_mode = str(telemetry.get("firmware_mode", "N/A"))
            hw_alive = bool(telemetry.get("is_hardware_alive", False))

            vib_hist = self.current_source.get_vibration_history(max_samples=512)

            # Interactive Demo / Benchmark Injection (Key 'T' to cycle)
            if self.demo_profile_idx > 0:
                self.demo_phase += 0.06
                t_now = time.time()
                demo_name = self.demo_profiles[self.demo_profile_idx]
                n_sim_vib = 128
                t_sim_arr = np.linspace(t_now - n_sim_vib / 400.0, t_now, n_sim_vib)

                if demo_name == "DEMO_HEALTHY_1800RPM":
                    rpm = 1800.0 + 8.0 * math.sin(self.demo_phase * 2.0)
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f_shaft = rpm / 60.0
                    ax_arr = 0.045 * np.sin(2.0 * np.pi * f_shaft * t_sim_arr) + 0.008 * np.random.randn(n_sim_vib)
                    ay_arr = 0.055 * np.cos(2.0 * np.pi * f_shaft * t_sim_arr) + 0.008 * np.random.randn(n_sim_vib)
                    az_arr = -0.82 + 0.025 * np.sin(4.0 * np.pi * f_shaft * t_sim_arr) + 0.008 * np.random.randn(n_sim_vib)
                    vib_hist = (t_sim_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                elif demo_name == "DEMO_UNBALANCE_1X":
                    rpm = 1765.0 + 15.0 * math.sin(self.demo_phase * 2.5)
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f_shaft = rpm / 60.0
                    ax_arr = 0.380 * np.sin(2.0 * np.pi * f_shaft * t_sim_arr) + 0.025 * np.random.randn(n_sim_vib)
                    ay_arr = 0.420 * np.cos(2.0 * np.pi * f_shaft * t_sim_arr) + 0.025 * np.random.randn(n_sim_vib)
                    az_arr = -0.82 + 0.110 * np.sin(4.0 * np.pi * f_shaft * t_sim_arr) + 0.015 * np.random.randn(n_sim_vib)
                    vib_hist = (t_sim_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                elif demo_name == "DEMO_MISALIGNMENT_2X":
                    rpm = 1750.0 + 12.0 * math.sin(self.demo_phase * 3.0)
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f_shaft = rpm / 60.0
                    ax_arr = 0.140 * np.sin(2.0 * np.pi * f_shaft * t_sim_arr) + 0.020 * np.random.randn(n_sim_vib)
                    ay_arr = 0.160 * np.cos(2.0 * np.pi * f_shaft * t_sim_arr) + 0.020 * np.random.randn(n_sim_vib)
                    az_arr = -0.82 + 0.380 * np.sin(4.0 * np.pi * f_shaft * t_sim_arr) + 0.025 * np.random.randn(n_sim_vib)
                    vib_hist = (t_sim_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                elif demo_name == "DEMO_DISTURBANCE_PROOF":
                    rpm = 1800.0
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f_shaft = rpm / 60.0
                    ax_arr = 0.040 * np.sin(2.0 * np.pi * f_shaft * t_sim_arr) + 0.008 * np.random.randn(n_sim_vib)
                    ay_arr = 0.050 * np.cos(2.0 * np.pi * f_shaft * t_sim_arr) + 0.008 * np.random.randn(n_sim_vib)
                    az_arr = -0.82 + 0.020 * np.sin(4.0 * np.pi * f_shaft * t_sim_arr) + 0.008 * np.random.randn(n_sim_vib)
                    vib_hist = (t_sim_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                    # Injected acoustic noise proves PDF Page 13 disturbance rejection
                    raw_frame = (raw_frame + 0.15 * np.random.randn(*raw_frame.shape)).astype(np.float32)

            motor_diag = self.motor_engine.analyze(raw_frame, fs, ax_g, ay_g, az_g, rpm, vib_history=vib_hist)
            dyn_vib_g = float(motor_diag.get("dyn_vib_g", 0.0))

            if self.learn_baseline_requested:
                self.motor_engine.learn_baseline(rpm, dyn_vib_g, raw_frame, fs)
                self.learn_baseline_requested = False

            # Update Live ADXL345 3-Axis Dynamic Vibration Scope (gs[1, 2])
            if vib_hist is not None and len(vib_hist[1]) >= 8:
                _, ax_pts, ay_pts, az_pts = vib_hist
                n_vpts = min(len(ax_pts), 256)
                x_v = np.arange(n_vpts)
                dyn_x = ax_pts[-n_vpts:] - self.motor_engine.grav_x
                dyn_y = ay_pts[-n_vpts:] - self.motor_engine.grav_y
                dyn_z = az_pts[-n_vpts:] - self.motor_engine.grav_z
                dyn_mag = np.sqrt(dyn_x**2 + dyn_y**2 + dyn_z**2)

                self.vib_line_x.set_data(x_v, np.clip(dyn_x, -1.2, 1.2))
                self.vib_line_y.set_data(x_v, np.clip(dyn_y, -1.2, 1.2))
                self.vib_line_z.set_data(x_v, np.clip(dyn_z, -1.2, 1.2))
                self.vib_line_dyn.set_data(x_v, np.clip(dyn_mag, -1.2, 1.2))
                self.ax_vib.set_xlim(0, max(32, n_vpts))
            else:
                self.vib_line_x.set_data([], [])
                self.vib_line_y.set_data([], [])
                self.vib_line_z.set_data([], [])
                self.vib_line_dyn.set_data([], [])

            is_running = bool(motor_diag.get("is_running", False))
            iso_zone_short = motor_diag["iso_severity"].split(":")[0]
            adxl_hw = telemetry.get("adxl_hw_str", "ADXL345: STANDBY")
            num_v_smp = telemetry.get("num_vib_last", len(vib_hist[1]) if (vib_hist and len(vib_hist[1])) else 0)
            status_note = "MOTOR OFF" if not is_running else iso_zone_short

            self.vib_badge_text.set_text(
                f"{adxl_hw} ({num_v_smp} smp/pkt)\n"
                f"a_dyn={dyn_vib_g:.3f}g | ISO Vel={motor_diag.get('v_rms_mms', 0.0):.2f} mm/s | {status_note}"
            )
            self.vib_badge_text.set_color(motor_diag["iso_color"])

            # Update Rotational Order Spectrum & Tachometer Speedometer (gs[0, 3])
            if pulses > self._last_seen_pulses:
                self._last_pulse_blink_time = now
                self._last_seen_pulses = pulses

            is_blinking = (now - self._last_pulse_blink_time) < 0.25

            if is_blinking or (is_running and (int(now * 8) % 2 == 0)):
                self.tach_indicator_text.set_text(f"● FC-03 TACH: PULSE ACTIVE ({pulses} Pulses)")
                self.tach_indicator_text.set_color("#00ffcc")
            elif pulses > 0:
                self.tach_indicator_text.set_text(f"○ FC-03 TACH: READY ({pulses} Pulses)")
                self.tach_indicator_text.set_color("#8da2c0")
            else:
                self.tach_indicator_text.set_text("○ FC-03 TACH: READY (0 Pulses — Shaft Idle)")
                self.tach_indicator_text.set_color("#64748b")

            if is_running:
                self.speedometer_text.set_text(f"● RUNNING: {rpm:4.0f} RPM (f0 = {motor_diag['f0_hz']:.1f} Hz)")
                self.speedometer_text.set_color("#00ffcc")
            else:
                self.speedometer_text.set_text("○ MOTOR STOPPED / STANDBY (0 RPM)")
                self.speedometer_text.set_color("#94a3b8")

            e1 = float(motor_diag.get("e_1x_pct", 0.0))
            e2 = float(motor_diag.get("e_2x_pct", 0.0))
            e3 = float(motor_diag.get("e_3x_pct", 0.0))

            self.order_bars[0].set_height(e1)
            self.order_bars[1].set_height(e2)
            self.order_bars[2].set_height(e3)

            c1 = "#ef4444" if (e1 > 18.0 and dyn_vib_g >= 0.14) else ("#f59e0b" if (e1 > 10.0 and dyn_vib_g >= 0.10) else "#00ffcc")
            c2 = "#e879f9" if e2 > 12.0 else ("#a855f7" if e2 > 6.0 else "#38bdf8")
            c3 = "#f43f5e" if e3 > 10.0 else "#64748b"

            self.order_bars[0].set_color(c1)
            self.order_bars[1].set_color(c2)
            self.order_bars[2].set_color(c3)

            self.bar_labels[0].set_position((0, max(2.0, e1 + 2.0)))
            self.bar_labels[0].set_text(f"{e1:.1f}%")
            self.bar_labels[1].set_position((1, max(2.0, e2 + 2.0)))
            self.bar_labels[1].set_text(f"{e2:.1f}%")
            self.bar_labels[2].set_position((2, max(2.0, e3 + 2.0)))
            self.bar_labels[2].set_text(f"{e3:.1f}%")

            # 8. Multimodal Motor Health & Diagnostic HUD Text (gs[1, 3])
            dir_str = "CENTER" if abs(target_angle) < 6.0 else ("RIGHT" if target_angle > 0 else "LEFT")
            lock_badge = f"* LOCKED ({dir_str})" if is_locked else "SEARCHING / SCANNING..."
            invert_badge = "INVERTED (Key 'I')" if self.invert_channels else "NORMAL (Key 'I')"
            port_label = str(telemetry.get("port", getattr(self.current_source, "port", "ESP32")))
            if port_label == "auto":
                port_label = "ESP32"
            hw_badge = f"STREAMING ({port_label})" if hw_alive else "STANDBY / SIMULATION"

            def make_vu_bar(pct, width=12):
                filled = int(round((pct / 100.0) * width))
                filled = max(0, min(width, filled))
                return "█" * filled + "░" * (width - filled)

            db_l = 20.0 * math.log10(max(intensity_l, 1e-4))
            db_c = 20.0 * math.log10(max(intensity_c, 1e-4))
            db_r = 20.0 * math.log10(max(intensity_r, 1e-4))

            dom_tag = {
                "LEFT": "LEFT FLANK",
                "CENTER": "FRONT CENTER",
                "RIGHT": "RIGHT FLANK"
            }.get(dom_s, "") if is_active_sound else "QUIET IDLE"

            # Motor Health Index Display
            if not is_running:
                h_bar = "░" * 12
                h_score_str = " --.-%"
                h_status = "STANDBY (STOPPED)"
            else:
                h_score = float(motor_diag["health_score"])
                h_bar = make_vu_bar(h_score, width=12)
                h_score_str = f"{h_score:5.1f}%"
                if h_score >= 85:
                    h_status = "EXCELLENT"
                elif h_score >= 65:
                    h_status = "PERMISSIBLE"
                elif h_score >= 40:
                    h_status = "WARNING"
                else:
                    h_status = "CRITICAL"

            # Baseline Status Line
            if motor_diag["baseline_locked"]:
                d = motor_diag["delta"]
                base_str = f"LOCKED (ΔVib:{d['d_vib_g']:+.2f}g, ΔRPM:{d['d_rpm']:+.0f})"
            else:
                base_str = "UNLOCKED [Press 'B' to Learn]"

            mode_tag = self._get_evidence_mode_tag()
            self.title_text.set_text(
                f"MINOR PROJECT VISION: [VIBRATION + 4-MIC ARRAY + RPM]  ►  ANSWERS 'WHAT IS WRONG?' & 'WHERE IS IT?'  |  [{mode_tag}]"
            )

            f0_str = f"{motor_diag['f0_hz']:.1f}Hz" if is_running else "IDLE"
            if self.demo_profile_idx > 0:
                adxl_short = "DEMO (400Hz SIM)"
            else:
                adxl_short = "ONLINE (FIFO)" if telemetry.get("adxl_hw_ok", False) else "CHECK I2C"
            iso_short = motor_diag["iso_severity"].split(":")[0]
            calib_short = "ON" if self.apply_calibration else "RAW"
            vad_short = "ACTIVE" if is_active_sound else "QUIET"

            # Update Card 1: WHAT IS WRONG?
            fault_str = str(motor_diag["primary_fault"])
            if "DISTURBANCE" in fault_str:
                what_title = "ACOUSTIC DISTURBANCE (NON-MECHANICAL)"
                what_sub = f"Vib Nominal ({dyn_vib_g:.3f}g {iso_short}) -> Motor OK!"
                what_col = "#ffaa00"
            elif not is_running:
                what_title = "MOTOR STOPPED / STANDBY"
                what_sub = f"Vib: {dyn_vib_g:.3f}g | Awaiting Shaft Rotation"
                what_col = "#94a3b8"
            elif "NOMINAL" in fault_str:
                what_title = "NOMINAL / HEALTHY MOTOR"
                what_sub = f"Health: {h_score_str} | Vib: {dyn_vib_g:.3f}g ({iso_short})"
                what_col = "#00ffcc"
            else:
                what_title = fault_str[:36]
                what_sub = f"Health: {h_score_str} ({h_status}) | {iso_short}"
                what_col = "#ff0055"

            if hasattr(self, "what_box_text"):
                self.what_box_text.set_text(f"WHAT IS WRONG?  ►  {what_title}\n{what_sub}")
                self.what_box_text.set_color(what_col)
                self.what_box_text.get_bbox_patch().set_edgecolor(what_col)

            # Update Card 2: WHERE IS THE SOUND?
            if is_source_valid:
                where_title = f"DOA: {target_angle:+5.1f}° [{dir_str} FLANK]"
                where_sub = f"Left:{pct_l:3.0f}% | Center:{pct_c:3.0f}% | Right:{pct_r:3.0f}%"
                where_col = "#00ffcc" if is_locked else "#ffaa00"
            elif is_active_sound:
                where_title = "SOUND PRESENT (DIFFUSE / UNRESOLVED)"
                where_sub = f"Gate: {side_metrics.get('status', 'SPATIAL GATE')[:28]}"
                where_col = "#ffaa00"
            else:
                where_title = "QUIET AMBIENT FIELD (NO TARGET)"
                where_sub = f"Noise Floor: {self.noise_floor:.4f} | 6-Pair SRP Idle"
                where_col = "#64748b"

            if hasattr(self, "where_box_text"):
                self.where_box_text.set_text(f"WHERE IS IT?  ►  {where_title}\n{where_sub}")
                self.where_box_text.set_color(where_col)
                self.where_box_text.get_bbox_patch().set_edgecolor(where_col)

            port_label = str(telemetry.get("port", "COM8"))
            hud_lines = [
                f" [HEALTH]   [{h_bar}] {h_score_str} ({h_status})",
                f" [BASELINE] {base_str[:34]}",
                f" [S1: TACH] {rpm:5.0f} RPM ({f0_str}) | Pulses:{pulses} (PPR:{self.pulses_per_rev})",
                f" [S2: VIB]  {adxl_short} | a_dyn:{dyn_vib_g:.3f}g | {motor_diag.get('v_rms_mms', 0.0):.2f}mm/s",
                f" [S3: ACOU] Kurtosis:{motor_diag['kurtosis']:5.2f} | Crest:{motor_diag['crest_factor']:4.1f} | VAD:{vad_short}",
                f" [S4: ORDR] 1X:{e1:4.1f}% | 2X:{e2:4.1f}% | 3X:{e3:4.1f}%",
                f" [SERIAL]   {port_label} @ 921.6k | OK:{telemetry.get('packets_valid', 0)} Drop:{telemetry.get('dropped_packets', 0)} Cal:{calib_short}",
                " --------------------------------------------",
                " KEYS: [B] Lock Base [C] Clear [P] PPR [T] Demo",
                "       [1] Laptop [2] ESP32 [K] Cal [I] Flip"
            ]
            self.hud_text.set_text("\n".join(hud_lines))

        except Exception as e:
            if not hasattr(self, "_last_err_time") or time.time() - self._last_err_time > 3.0:
                print(f"[!] Warning in GUI update loop: {e}", file=sys.stderr)
                self._last_err_time = time.time()

        return all_artists

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
    import argparse
    parser = argparse.ArgumentParser(description="Spatial Multimodal Acoustic Radar Engine (Pro V6)")
    parser.add_argument("--port", type=str, default="auto", help="ESP32 serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baudrate (default: 921600)")
    args, _ = parser.parse_known_args()

    print("=====================================================================")
    print(" SPATIAL MULTIMODAL ACOUSTIC RADAR ENGINE (PRO V6)")
    print(" 4-Microphone Uniform Linear Array + Multimodal Fault Localization")
    print(f" Target Serial Port: {args.port} @ {args.baud} Baud")
    print("=====================================================================")
    print(" Physical Array Mapping on Breadboard:")
    print("   [Mic 1] (Row 20, Pair A Left,  GPIO 19, L/R->GND) -> x = -30.48 mm")
    print("   [Mic 2] (Row 28, Pair A Right, GPIO 19, L/R->3V3) -> x = -10.16 mm")
    print("   [Mic 3] (Row 36, Pair B Left,  GPIO 23, L/R->GND) -> x = +10.16 mm")
    print("   [Mic 4] (Row 44, Pair B Right, GPIO 23, L/R->3V3) -> x = +30.48 mm")
    print(" Total Array Aperture: 60.96 mm | 6 Correlation Delay Pairs")
    print(" ADXL345: SDA=TX2 (GPIO 17), SCL=RX2 (GPIO 16) | FC-03: DO=D4 (GPIO 4)")
    print("=====================================================================")
    app = AcousticRadarApp(port=args.port, baud=args.baud)
    app.run()
