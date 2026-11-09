#!/usr/bin/env python3
"""Guarded live-radar fix. Run from the Spatial-accoustic-project root."""
from pathlib import Path
import hashlib
import shutil

EXPECTED = {
    Path("stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py"): "c99c47603e8ab6f32c4c5a69fefb10063e40f669",
    Path("tests/test_noise_gating_regression.py"): "1b37dba9ebc452c1bb90e829eacfb7fa55721fc3",
}

def blob_sha(text):
    b = text.encode("utf-8")
    return hashlib.sha1(f"blob {len(b)}\0".encode() + b).hexdigest()

def replace_one(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"{label}: wanted exactly one match, found {n}")
    return text.replace(old, new, 1)

def main():
    root = Path.cwd()
    loaded = {}
    for rel, expected in EXPECTED.items():
        path = root / rel
        if not path.is_file():
            raise RuntimeError(f"Run this from repository root. Missing {rel}")
        content = path.read_text(encoding="utf-8")
        actual = blob_sha(content)
        if actual != expected:
            raise RuntimeError(
                f"{rel} is not the reviewed revision.\nExpected blob: {expected}\nFound blob:    {actual}\n"
                "No files were changed. Commit/stash local changes or update this patcher."
            )
        loaded[rel] = content

    src = next(p for p in EXPECTED if p.name == "spatial_acoustic_radar_pro.py")
    test = next(p for p in EXPECTED if p.name == "test_noise_gating_regression.py")
    s, t = loaded[src], loaded[test]

    s = replace_one(s,
        "from matplotlib.animation import FuncAnimation\nfrom scipy import signal\n",
        "from matplotlib.animation import FuncAnimation\n"
        "from matplotlib.colors import LinearSegmentedColormap\n"
        "from scipy import signal\nfrom scipy.ndimage import gaussian_filter\n",
        "imports")
    s = replace_one(s,
        "FREQ_MIN = 80.0\nFREQ_MAX = 5500.0\n\n# Frame & Animation Timing\n",
        "FREQ_MIN = 80.0\nFREQ_MAX = 5500.0\n\n"
        "# Fixed palette: blue/cyan/green/yellow/orange/red. Relative digital level, not dB SPL.\n"
        "ACOUSTIC_HEAT_CMAP = LinearSegmentedColormap.from_list(\n"
        '    "acoustic_thermal",\n'
        '    ["#050914", "#123e91", "#08b9d2", "#18c979", "#e5d83b", "#ff9226", "#ff2424"], N=256\n'
        ")\n\n# Frame & Animation Timing\n",
        "thermal palette")

    s = replace_one(s,
        '    def start_calibration(self):\n'
        '        """Clears existing sample history and restarts ambient noise floor calibration."""\n'
        '        self.calib_samples.clear()\n'
        '        self.is_calibrating = True\n\n'
        '    def process_frame(self, avg_rms: float, current_time: float) -> Tuple[bool, float, float]:\n',
        '    def start_calibration(self):\n'
        '        """Clears existing sample history and restarts ambient noise floor calibration."""\n'
        '        self.calib_samples.clear()\n'
        '        self.is_calibrating = True\n\n'
        '    def note_coherent_source(self, current_time: float):\n'
        '        """Recover when SRP proves a coherent source was present during startup."""\n'
        '        self.is_calibrating = False\n'
        '        self.calib_samples.clear()\n'
        '        self.noise_floor = min(max(0.0008, self.noise_floor), 0.0030)\n'
        '        self.min_active_threshold = max(0.0025, self.noise_floor * 1.60)\n'
        '        self.last_active_time = current_time\n\n'
        '    def process_frame(self, avg_rms: float, current_time: float) -> Tuple[bool, float, float]:\n',
        "VAD recovery method")
    s = replace_one(s,
        '            if len(self.calib_samples) >= self.calib_frames:\n'
        '                self.noise_floor = max(0.0010, float(np.median(self.calib_samples)))\n'
        '                self.min_active_threshold = max(0.0030, self.noise_floor * 1.50)\n'
        '                self.is_calibrating = False\n',
        '            if len(self.calib_samples) >= self.calib_frames:\n'
        '                # Lower quantile is less sensitive to brief target sound during startup.\n'
        '                self.noise_floor = max(0.0008, float(np.percentile(self.calib_samples, 20)))\n'
        '                self.min_active_threshold = max(0.0025, self.noise_floor * 1.60)\n'
        '                self.is_calibrating = False\n',
        "robust startup baseline")

    # Add machine-readable status to each invalid SRP result.
    s = replace_one(s, '"spatial_rho": 0.0\n',
                    '"spatial_rho": 0.0, "status": "BELOW_ENERGY_GATE"\n', "energy status")
    s = replace_one(s,
        '"spatial_rho": max_spatial_rho\n                }\n            )\n\n        valid_pairs_count = 0',
        '"spatial_rho": max_spatial_rho, "status": "LOW_SPATIAL_COHERENCE"\n                }\n            )\n\n        valid_pairs_count = 0', "coherence status")
    s = replace_one(s,
        '"spatial_rho": max_spatial_rho\n                }\n            )\n\n        srp_norm = srp_profile / max_srp',
        '"spatial_rho": max_spatial_rho, "status": "NO_VALID_GCC_PAIRS"\n                }\n            )\n\n        srp_norm = srp_profile / max_srp', "GCC status")
    s = replace_one(s,
        '"spatial_rho": max_spatial_rho\n                }\n            )\n\n        # Multi-Pair Angular Consensus Gate:',
        '"spatial_rho": max_spatial_rho, "status": "LOW_ANGULAR_CONTRAST"\n                }\n            )\n\n        # Multi-Pair Angular Consensus Gate:', "contrast status")
    s = replace_one(s,
        '"spatial_rho": max_spatial_rho\n                }\n            )\n\n        # Sector Power & Side Intensity Breakdown with Noise-Floor Contrast',
        '"spatial_rho": max_spatial_rho, "status": "PAIR_ANGLE_DISAGREEMENT"\n                }\n            )\n\n        # Sector Power & Side Intensity Breakdown with Noise-Floor Contrast', "consensus status")
    s = replace_one(s,
        '"ch_rms": ch_rms,\n            "ch_status": ch_status\n        }\n\n        return best_bearing_deg, composite_prom, angles_deg, srp_norm, raw_2d_heatmap, side_metrics',
        '"ch_rms": ch_rms,\n            "ch_status": ch_status,\n            "spatial_rho": max_spatial_rho,\n            "status": "LOCALIZED"\n        }\n\n        return best_bearing_deg, composite_prom, angles_deg, srp_norm, raw_2d_heatmap, side_metrics',
        "localized status")

    # Replace the whole heatmap method to avoid fragile line-by-line edits.
    a = s.index("    @classmethod\n    def compute_true_2d_acoustic_heatmap(")
    b = s.index("    @classmethod\n    def synthesize_directional_map(", a)
    heat_lines = [
        "    @classmethod",
        "    def compute_true_2d_acoustic_heatmap(",
        "        cls, raw_2d_heatmap: np.ndarray, target_range_m: float, avg_rms: float,",
        "        is_locked: bool, geometry: ArrayGeometry, noise_floor: float = 0.0030",
        "    ) -> np.ndarray:",
        '        """Fixed monotonic thermal intensity from SRP shape and RMS above ambient.',
        "",
        "        Relative digital level, not calibrated dB SPL. A 1D ULA measures bearing,",
        "        not source range; radial weighting is only for display.",
        '        """',
        "        raw = np.asarray(raw_2d_heatmap, dtype=np.float64)",
        "        raw = np.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0)",
        "        spatial = np.maximum(raw, 0.0)",
        "        peak = float(np.max(spatial)) if spatial.size else 0.0",
        "        if peak <= 1e-12:",
        "            return np.zeros_like(raw, dtype=np.float64)",
        "        # Normalize spatial shape only; never normalize final intensity per frame.",
        "        spatial /= peak",
        '        spatial = gaussian_filter(spatial, sigma=1.15, mode="nearest")',
        "        spatial_peak = float(np.max(spatial))",
        "        if spatial_peak <= 1e-12:",
        "            return np.zeros_like(raw, dtype=np.float64)",
        "        spatial /= spatial_peak",
        "        r_grid = np.sqrt(geometry.xx ** 2 + geometry.yy ** 2)",
        "        dist_weight = 1.0 / np.sqrt(np.maximum(r_grid, 0.40))",
        "        dist_weight /= max(float(np.max(dist_weight)), 1e-12)",
        "        spatial *= dist_weight",
        "        spatial /= max(float(np.max(spatial)), 1e-12)",
        "        # Fixed monotonic 1.5–16 dB-above-ambient digital scale.",
        "        floor = max(float(noise_floor), 0.0008)",
        "        level_db = 20.0 * math.log10(max(float(avg_rms), 1e-9) / floor)",
        "        level = float(np.clip((level_db - 1.5) / 14.5, 0.0, 1.0))",
        "        return np.clip(np.power(np.clip(spatial, 0.0, 1.0), 1.35) * level, 0.0, 1.0)",
        "",
        ""
    ]
    s = s[:a] + "\n".join(heat_lines) + s[b:]

    s = replace_one(s,
        '            vmin=0.0, vmax=1.0,\n            cmap="inferno",\n            interpolation="bicubic"\n',
        '            vmin=0.0, vmax=1.0,\n            cmap=ACOUSTIC_HEAT_CMAP,\n            interpolation="bicubic"\n', "imshow palette")
    s = replace_one(s,
        '        self.cb.set_label("Live Acoustic Intensity (SRP-PHAT Relative Power)", color="#b8c2d1", fontsize=10)\n',
        '        self.cb.set_label("Relative acoustic level above ambient (not dB SPL)", color="#b8c2d1", fontsize=10)\n',
        "colorbar label")

    # SRP coherence/GCC gates remain the authority for a directional source.
    # The old VAD-derived threshold had been allowed to suppress the same incoming sound.
    s = replace_one(s,
        "            # Process frame through production Schmitt-Trigger VAD with frozen adaptation\n"
        "            is_active_sound, self.noise_floor, attack_thresh = self.vad.process_frame(avg_rms, now)\n\n"
        "            # 2. Multi-Pair Steered Response Power & 2D Backprojection\n"
        "            min_srp_rms = max(0.0020, self.noise_floor * 1.20)\n"
        "            best_bearing, composite_prom, srp_angles, srp_norm, raw_2d_heatmap, side_metrics = (\n"
        "                SRPPhatEngine.compute_srp_frame(raw_frame, fs, self.geometry, min_active_rms=min_srp_rms)\n"
        "            )\n\n"
        '            is_source_valid = bool(is_active_sound and side_metrics.get("is_valid", False))\n',
        "            # RMS VAD gives ambient status, but a contaminated startup baseline must not\n"
        "            # veto SRP's independent spatial-coherence and GCC-PHAT source validation.\n"
        "            is_active_sound, self.noise_floor, attack_thresh = self.vad.process_frame(avg_rms, now)\n\n"
        "            min_srp_rms = 0.0020\n"
        "            best_bearing, composite_prom, srp_angles, srp_norm, raw_2d_heatmap, side_metrics = (\n"
        "                SRPPhatEngine.compute_srp_frame(raw_frame, fs, self.geometry, min_active_rms=min_srp_rms)\n"
        "            )\n\n"
        "            is_audio_energy_present = bool(avg_rms >= min_srp_rms)\n"
        '            is_source_valid = bool(side_metrics.get("is_valid", False))\n'
        "            if is_source_valid:\n"
        "                self.vad.note_coherent_source(now)\n"
        "                self.noise_floor = self.vad.noise_floor\n"
        "                is_active_sound = True\n"
        "            else:\n"
        "                # Energy is present, but do not invent a bearing or hotspot.\n"
        "                is_active_sound = bool(is_active_sound or is_audio_energy_present)\n",
        "source activation logic")
    s = replace_one(s,
        "                    raw_2d_heatmap, target_range, avg_rms, is_locked, self.geometry\n",
        "                    raw_2d_heatmap, target_range, avg_rms, is_locked, self.geometry, self.noise_floor\n",
        "heatmap noise floor")
    s = replace_one(s,
        '            if not is_source_valid:\n'
        '                self.sector_banner_text.set_text("■ AMBIENT MONITORING — READY FOR TARGET SOUND ■")\n'
        '                self.sector_banner_text.set_color("#6b7d99")\n',
        '            if not is_source_valid and is_active_sound:\n'
        '                reject_reason = str(side_metrics.get("status", "SPATIAL GATE"))\n'
        '                self.sector_banner_text.set_text(f"■ SOUND PRESENT — BEARING UNRESOLVED ({reject_reason}) ■")\n'
        '                self.sector_banner_text.set_color("#ffaa00")\n'
        '            elif not is_source_valid:\n'
        '                self.sector_banner_text.set_text("■ AMBIENT MONITORING — READY FOR TARGET SOUND ■")\n'
        '                self.sector_banner_text.set_color("#6b7d99")\n',
        "unresolved sound banner")

    test_anchor = '\n\nif __name__ == "__main__":\n    unittest.main()\n'
    test_methods = "\n".join([
        "    def test_09_coherent_source_recovers_from_contaminated_startup_calibration(self):",
        '        """A coherent source must not remain masked by startup ambient calibration."""',
        "        vad = AcousticActivityDetector(min_active_threshold=0.0035, calib_frames=25, hangover_sec=0.350)",
        "        dt = self.frame_len / self.fs",
        "        now = 1000.0",
        "        for _ in range(25):",
        "            now += dt",
        "            vad.process_frame(0.0060, now)",
        "        self.assertFalse(vad.is_calibrating)",
        "        self.assertGreater(vad.noise_floor, 0.0050)",
        "        now += dt",
        "        vad.note_coherent_source(now)",
        "        self.assertLessEqual(vad.noise_floor, 0.0030)",
        "        active, _, _ = vad.process_frame(0.0050, now + dt)",
        "        self.assertTrue(active)",
        "",
        "    def test_10_heatmap_intensity_is_monotonic_and_silence_is_black(self):",
        '        """Production heatmap brightens with RMS and returns zero for silence."""',
        "        xx, yy = self.geometry.xx, self.geometry.yy",
        "        raw = np.exp(-(((xx - 0.30) / 0.35) ** 2 + ((yy - 1.60) / 0.45) ** 2))",
        "        quiet = SRPPhatEngine.compute_true_2d_acoustic_heatmap(raw, 1.25, 0.0040, True, self.geometry, noise_floor=0.0030)",
        "        loud = SRPPhatEngine.compute_true_2d_acoustic_heatmap(raw, 1.25, 0.0200, True, self.geometry, noise_floor=0.0030)",
        "        silent = SRPPhatEngine.compute_true_2d_acoustic_heatmap(np.zeros_like(raw), 1.25, 0.0200, True, self.geometry, noise_floor=0.0030)",
        "        self.assertTrue(np.all(np.isfinite(quiet)))",
        "        self.assertTrue(np.all(np.isfinite(loud)))",
        "        self.assertGreater(float(np.max(loud)), float(np.max(quiet)))",
        "        self.assertEqual(float(np.max(silent)), 0.0)",
    ])
    t = replace_one(t, test_anchor, "\n\n" + test_methods + "\n\nif __name__ == \"__main__\":\n    unittest.main()\n", "new regression tests")

    # Validate syntax and backup targets before writing either file.
    compile(s, str(src), "exec")
    compile(t, str(test), "exec")
    for rel in (src, test):
        backup = (root / rel).with_suffix((root / rel).suffix + ".pre-live-radar-fix.bak")
        if backup.exists():
            raise RuntimeError(f"Backup already exists: {backup}; move it aside before retrying")

    for rel, content in ((src, s), (test, t)):
        target = root / rel
        backup = target.with_suffix(target.suffix + ".pre-live-radar-fix.bak")
        shutil.copy2(target, backup)
        temp = target.with_suffix(target.suffix + ".tmp")
        temp.write_text(content, encoding="utf-8", newline="\n")
        temp.replace(target)
        print(f"[PATCHED] {rel}")
        print(f"[BACKUP]  {backup.name}")

    print("\nNext:")
    print("  python tests/test_noise_gating_regression.py")
    print("  python -m unittest discover -s tests -p 'test_*.py' -v")
    print("Restart RUN_LIVE_RADAR.bat and inspect the [ACOUSTIC GATE] HUD line.")
    return 0

if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}")
        raise SystemExit(10)
