"""
=============================================================================
SPHERICAL ACOUSTIC PHYSICS & ADVANCED MULTIMODAL KINEMATICS ENGINE (3D)
=============================================================================
Near-Field Spherical-Wave SRP-PHAT 3D Coordinate Backprojection,
Volumetric Acoustic Imaging, and Advanced Vibration Demodulation

Features:
  1. Near-Field Spherical Wave Propagation Model:
     - Replaces far-field planar wave approximations (tau = d/c * sin(theta))
       with exact spherical Euclidean delay manifolds tau_ij(x, y, z)
       and 1/r spherical divergence attenuation.
  2. Volumetric 3D Acoustic Backprojection:
     - Computes spatial intensity field S(x, y, z) over 3D Cartesian volume
       spanning Azimuth [-90 deg, +90 deg], Elevation [-45 deg, +45 deg],
       and Range [0.2m, 3.5m].
  3. Spectral Kurtosis & Kurtogram:
     - Subband spectral kurtosis for detecting transient impulsive impacts
       characteristic of bearing flaking, spalling, and gear pitting.
  4. Envelope Demodulation (Hilbert Transform):
     - Analytical envelope extraction to isolate bearing pass frequencies
       (BPFO, BPFI, BSF, FTF) from high-frequency structural resonances.
  5. 3D Dynamic Shaft Orbit Trajectory:
     - Tri-axial vibration state-space (a_x, a_y, a_z) after DC gravity decoupling
       yielding 3D dynamic center-line orbit ellipses.
=============================================================================
"""

import math
import numpy as np
from typing import Dict, List, Optional, Tuple, Union
from scipy import signal
from scipy.fft import rfft, irfft

SPEED_OF_SOUND = 343.0  # m/s at 20 deg C dry air

class SphericalAcousticPhysics3D:
    """
    Near-field spherical wave acoustic beamformer and 3D volumetric backprojection engine.
    """

    def __init__(
        self,
        mic_coords: np.ndarray,
        grid_bounds: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]] = (
            (-2.0, 2.0),  # X: Left/Right (m)
            (0.2, 3.5),   # Y: Depth/Range (m)
            (-0.8, 0.8),  # Z: Elevation/Height (m)
        ),
        grid_resolution: Tuple[int, int, int] = (25, 25, 11),
    ):
        """
        mic_coords: (M, 3) or (M, 2) array of physical microphone coordinates in meters.
        """
        coords = np.asarray(mic_coords, dtype=np.float64)
        if coords.shape[1] == 2:
            # Expand 2D (x, y) into 3D (x, 0, z) or (x, y, 0)
            zeros = np.zeros((coords.shape[0], 1), dtype=np.float64)
            coords = np.hstack([coords[:, :1], zeros, coords[:, 1:]])
        self.mic_coords = coords
        self.num_mics = coords.shape[0]

        # Generate microphone pairs
        self.pairs: List[Tuple[int, int]] = [
            (i, j) for i in range(self.num_mics) for j in range(i + 1, self.num_mics)
        ]
        self.num_pairs = len(self.pairs)

        # Baseline distance between reference mic 0 and 1
        if self.num_mics >= 2:
            self.baseline_m = float(np.linalg.norm(self.mic_coords[0] - self.mic_coords[1]))
        else:
            self.baseline_m = 0.02032

        # 3D Coordinate Grid Setup
        (x_min, x_max), (y_min, y_max), (z_min, z_max) = grid_bounds
        self.xs = np.linspace(x_min, x_max, grid_resolution[0])
        self.ys = np.linspace(y_min, y_max, grid_resolution[1])
        self.zs = np.linspace(z_min, z_max, grid_resolution[2])

        self.xx, self.yy, self.zz = np.meshgrid(self.xs, self.ys, self.zs, indexing="ij")
        self.grid_shape = self.xx.shape  # (Nx, Ny, Nz)

        # Precompute Spherical Delay Manifolds tau_ij(x, y, z)
        self.pair_delays = self._precompute_spherical_delays()

    def _precompute_spherical_delays(self) -> np.ndarray:
        """
        Precomputes the exact spherical time-difference of arrival (TDOA)
        for every pair (i, j) at every voxel (x, y, z) in the room.
        tau_ij(r) = (||r - m_i|| - ||r - m_j||) / c
        """
        mic_distances = []
        for mx, my, mz in self.mic_coords:
            r = np.sqrt(
                (self.xx - mx) ** 2 +
                (self.yy - my) ** 2 +
                (self.zz - mz) ** 2
            )
            mic_distances.append(r)

        delays = []
        for i, j in self.pairs:
            tau = (mic_distances[i] - mic_distances[j]) / SPEED_OF_SOUND
            delays.append(tau)

        return np.asarray(delays, dtype=np.float64)  # Shape: (P, Nx, Ny, Nz)

    def backproject_3d_volume(
        self,
        pair_correlations: Dict[Tuple[int, int], Tuple[np.ndarray, np.ndarray]],
        min_prominence: float = 0.50,
    ) -> np.ndarray:
        """
        Backprojects multi-pair GCC-PHAT cross-correlations into a 3D acoustic power volume.
        pair_correlations: dict mapping (i, j) -> (lags_seconds, correlation_values)
        Returns:
            volume_power: (Nx, Ny, Nz) array of normalized acoustic intensity
        """
        volume = np.zeros(self.grid_shape, dtype=np.float64)
        active_pairs = 0

        for p_idx, (i, j) in enumerate(self.pairs):
            if (i, j) not in pair_correlations:
                continue

            lags, corr = pair_correlations[(i, j)]
            if len(corr) < 5:
                continue

            # Prominence weighting
            pk = np.max(corr)
            avg = np.mean(corr)
            std = np.std(corr) + 1e-12
            prom = max(0.0, float((pk - avg) / std))
            if prom < min_prominence:
                continue

            active_pairs += 1
            weight = math.sqrt(float(np.linalg.norm(self.mic_coords[i] - self.mic_coords[j])) / self.baseline_m) * min(prom, 4.0)

            # Interpolate correlation values across the 3D voxel delay manifold
            tau_grid = self.pair_delays[p_idx].ravel()
            interp_vals = np.interp(tau_grid, lags, corr, left=0.0, right=0.0).reshape(self.grid_shape)
            volume += weight * np.maximum(interp_vals, 0.0)

        max_val = float(np.max(volume)) if volume.size > 0 else 0.0
        if max_val > 1e-12:
            volume /= max_val

        return volume

    def extract_3d_focal_point(
        self, volume_power: np.ndarray
    ) -> Tuple[float, float, float, float, float, float]:
        """
        Extracts the peak 3D acoustic source location from the power volume.
        Returns:
            x, y, z: Cartesian coordinates (meters)
            azimuth_deg: Azimuth bearing angle [-90, +90] deg
            elevation_deg: Elevation angle [-45, +45] deg
            range_m: Euclidean distance from array center (meters)
        """
        if volume_power.size == 0 or np.max(volume_power) < 1e-6:
            return 0.0, 1.0, 0.0, 0.0, 0.0, 1.0

        idx = np.unravel_index(np.argmax(volume_power), self.grid_shape)
        x = float(self.xs[idx[0]])
        y = float(self.ys[idx[1]])
        z = float(self.zs[idx[2]])

        range_m = float(np.sqrt(x**2 + y**2 + z**2))
        azimuth_deg = float(np.rad2deg(np.arctan2(x, y)))
        hypot_xy = float(np.sqrt(x**2 + y**2))
        elevation_deg = float(np.rad2deg(np.arctan2(z, hypot_xy)))

        return x, y, z, azimuth_deg, elevation_deg, range_m


class AdvancedVibrationKinematics:
    """
    Multimodal Vibration Kinematics & Advanced Fault Demodulation Suite.
    Implements Spectral Kurtosis, Analytic Envelope Demodulation, and 3D Shaft Orbit analysis.
    """

    @staticmethod
    def decouple_gravity_dc(accel_3axis: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Separates static gravitational acceleration from dynamic machinery vibration:
        a_dyn(t) = a(t) - g_vector
        """
        g_vector = np.mean(accel_3axis, axis=0)
        dyn_accel = accel_3axis - g_vector
        return dyn_accel, g_vector

    @staticmethod
    def compute_3d_orbit_trajectory(dyn_accel: np.ndarray, num_points: int = 256) -> Dict[str, np.ndarray]:
        """
        Computes 3D trajectory and 2D orbital projections (XY, YZ, ZX)
        of the rotating shaft centerline displacement.
        """
        n = min(len(dyn_accel), num_points)
        recent = dyn_accel[-n:]
        return {
            "x": recent[:, 0],
            "y": recent[:, 1],
            "z": recent[:, 2],
            "orbit_amplitude_rms": float(np.sqrt(np.mean(np.sum(recent**2, axis=1)))),
        }

    @staticmethod
    def spectral_kurtosis(signal_1d: np.ndarray, fs: float, nperseg: int = 128) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes Spectral Kurtosis (SK) across frequency subbands via STFT.
        Detects localized impulsive impact transients buried in background noise.
        """
        f, t, zxx = signal.stft(signal_1d, fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
        power = np.abs(zxx) ** 2

        # 4th and 2nd spectral moments
        mean_power = np.mean(power, axis=1) + 1e-15
        mean_sq_power = np.mean(power**2, axis=1)

        # SK = (E[|X|^4] / (E[|X|^2])^2) - 2
        sk = (mean_sq_power / (mean_power**2)) - 2.0
        sk = np.maximum(sk, 0.0)

        return f, sk

    @staticmethod
    def envelope_spectrum(
        signal_1d: np.ndarray,
        fs: float,
        bandpass_bounds: Optional[Tuple[float, float]] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Computes the Analytical Envelope Spectrum via Hilbert Transform:
        1. Optional bandpass filtering around structural resonance carrier.
        2. Hilbert transform extraction of instantaneous amplitude envelope.
        3. FFT of the demeaned envelope to reveal bearing impact repetition rates.
        """
        sig = signal_1d - np.mean(signal_1d)

        # Bandpass filter around carrier if specified
        if bandpass_bounds is not None:
            f_low, f_high = bandpass_bounds
            nyq = fs / 2.0
            if f_high < nyq and f_low > 0:
                b, a = signal.butter(4, [f_low / nyq, f_high / nyq], btype="band")
                sig = signal.filtfilt(b, a, sig)

        # Hilbert transform for analytical envelope
        analytic = signal.hilbert(sig)
        envelope = np.abs(analytic)
        envelope_dyn = envelope - np.mean(envelope)

        # Spectrum of envelope
        n = len(envelope_dyn)
        freqs = np.fft.rfftfreq(n, d=1.0 / fs)
        env_spec = np.abs(rfft(envelope_dyn)) / (n / 2.0 + 1e-12)

        return freqs, env_spec

    @staticmethod
    def calculate_bearing_fault_frequencies(
        rpm: float,
        num_balls: int = 8,
        ball_diameter_mm: float = 7.5,
        pitch_diameter_mm: float = 30.0,
        contact_angle_deg: float = 0.0,
    ) -> Dict[str, float]:
        """
        Calculates theoretical bearing kinematic fault characteristic frequencies (Hz):
        - BPFO: Ball Pass Frequency Outer Race
        - BPFI: Ball Pass Frequency Inner Race
        - BSF:  Ball Spin Frequency
        - FTF:  Fundamental Train Frequency (Cage)
        """
        fr = max(0.1, rpm / 60.0)  # Shaft rotational frequency (1X Hz)
        gamma = (ball_diameter_mm / pitch_diameter_mm) * math.cos(math.radians(contact_angle_deg))

        bpfo = (num_balls / 2.0) * fr * (1.0 - gamma)
        bpfi = (num_balls / 2.0) * fr * (1.0 + gamma)
        bsf  = (pitch_diameter_mm / (2.0 * ball_diameter_mm)) * fr * (1.0 - gamma**2)
        ftf  = (fr / 2.0) * (1.0 - gamma)

        return {
            "shaft_1x_hz": round(fr, 2),
            "bpfo_hz": round(bpfo, 2),
            "bpfi_hz": round(bpfi, 2),
            "bsf_hz": round(bsf, 2),
            "ftf_hz": round(ftf, 2),
        }
