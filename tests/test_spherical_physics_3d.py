"""
Unit & Kinematic Verification Suite for Spherical Acoustic Physics & 3D Kinematics
"""

import math
import numpy as np
import pytest

from stage4_realtime_fusion_radar.src.spherical_acoustic_physics_3d import (
    SphericalAcousticPhysics3D,
    AdvancedVibrationKinematics,
    SPEED_OF_SOUND,
)

def test_spherical_delay_geometry():
    # 4-Mic linear array along X-axis
    mics = np.array([
        [-0.03048, 0.0, 0.0],
        [-0.01016, 0.0, 0.0],
        [+0.01016, 0.0, 0.0],
        [+0.03048, 0.0, 0.0],
    ])
    engine = SphericalAcousticPhysics3D(
        mic_coords=mics,
        grid_bounds=((-1.0, 1.0), (0.5, 2.0), (-0.5, 0.5)),
        grid_resolution=(11, 11, 5)
    )

    # Voxel exactly on boresight (x=0, y=1.0, z=0)
    # Distance to mic 1 (-0.03048) and mic 4 (+0.03048) must be identical
    d_m1 = math.sqrt((-0.03048)**2 + 1.0**2)
    d_m4 = math.sqrt((+0.03048)**2 + 1.0**2)
    assert abs(d_m1 - d_m4) < 1e-12

    # Verify precomputed delay shape
    assert engine.pair_delays.shape == (6, 11, 11, 5)

def test_bearing_fault_kinematics():
    # 1800 RPM = 30 Hz shaft speed
    rpm = 1800.0
    faults = AdvancedVibrationKinematics.calculate_bearing_fault_frequencies(
        rpm=rpm,
        num_balls=8,
        ball_diameter_mm=7.5,
        pitch_diameter_mm=30.0,
        contact_angle_deg=0.0
    )

    assert faults["shaft_1x_hz"] == 30.0
    # gamma = 7.5 / 30 = 0.25
    # BPFO = 4 * 30 * (1 - 0.25) = 90.0 Hz
    # BPFI = 4 * 30 * (1 + 0.25) = 150.0 Hz
    assert faults["bpfo_hz"] == 90.0
    assert faults["bpfi_hz"] == 150.0
    assert faults["ftf_hz"] == 11.25

def test_dynamic_vibration_gravity_decoupling():
    # Create synthetic 3-axis accel with 1g gravity on Z-axis and sinusoidal vibration
    t = np.linspace(0, 1.0, 400)
    vx = 0.05 * np.sin(2 * np.pi * 25 * t)
    vy = 0.03 * np.cos(2 * np.pi * 50 * t)
    vz = 0.08 * np.sin(2 * np.pi * 25 * t)

    raw_accel = np.column_stack([
        vx,
        vy,
        vz - 0.981  # -1g gravity
    ])

    dyn_accel, g_vec = AdvancedVibrationKinematics.decouple_gravity_dc(raw_accel)

    # Static gravity vector should be ~ -0.981 on Z
    assert abs(g_vec[2] - (-0.981)) < 0.02
    assert abs(g_vec[0]) < 0.01

    # Dynamic vibration should have near-zero mean
    assert abs(np.mean(dyn_accel[:, 2])) < 1e-6

    # Orbit trajectory
    orbit = AdvancedVibrationKinematics.compute_3d_orbit_trajectory(dyn_accel, num_points=len(dyn_accel))
    assert orbit["orbit_amplitude_rms"] > 0.04
    assert len(orbit["x"]) == len(dyn_accel)

def test_envelope_spectrum_detection():
    # Synthetic amplitude-modulated signal: 200 Hz carrier modulated by 15 Hz fault impact
    fs = 1000.0
    t = np.linspace(0, 1.0, int(fs))
    mod = 1.0 + 0.7 * np.sin(2 * np.pi * 15 * t)
    carrier = np.sin(2 * np.pi * 200 * t)
    sig = mod * carrier

    freqs, env_spec = AdvancedVibrationKinematics.envelope_spectrum(sig, fs=fs)

    # Peak in envelope spectrum should be at ~15 Hz
    peak_idx = int(np.argmax(env_spec[1:])) + 1
    detected_freq = freqs[peak_idx]
    assert abs(detected_freq - 15.0) <= 2.0

def test_3d_volumetric_backprojection_localization():
    mics = np.array([
        [-0.03048, 0.0, 0.0],
        [-0.01016, 0.0, 0.0],
        [+0.01016, 0.0, 0.0],
        [+0.03048, 0.0, 0.0],
    ])
    engine = SphericalAcousticPhysics3D(
        mic_coords=mics,
        grid_bounds=((-1.5, 1.5), (0.4, 2.5), (-0.4, 0.4)),
        grid_resolution=(15, 15, 7)
    )

    # Synthetic source located at x = +0.5, y = 1.0, z = 0.0
    src = np.array([0.5, 1.0, 0.0])
    pair_corrs = {}

    for i, j in engine.pairs:
        d_i = np.linalg.norm(src - mics[i])
        d_j = np.linalg.norm(src - mics[j])
        true_tau = (d_i - d_j) / SPEED_OF_SOUND

        # Create sinc-like correlation curve centered at true_tau
        lags = np.linspace(-0.001, 0.001, 101)
        corr = np.sinc((lags - true_tau) * 5000.0)
        pair_corrs[(i, j)] = (lags, corr)

    vol = engine.backproject_3d_volume(pair_corrs)
    assert vol.shape == (15, 15, 7)
    assert np.max(vol) == 1.0

    x, y, z, az, el, r = engine.extract_3d_focal_point(vol)
    # Azimuth should be positive (source on the right)
    assert az > 15.0
    assert 0.3 < r < 2.0
