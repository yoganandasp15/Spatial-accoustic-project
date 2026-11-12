"""
=============================================================================
SPATIAL MULTIMODAL EDGE-AI — RIGHT-SIDE DIAGNOSTIC HUD (STANDALONE)
=============================================================================
Single-panel compact window showing ONLY the right diagnostic column:
  • Shaft Speed / FC-03 Tachometer + Rotational Order bars (1X / 2X / 3X)
  • Multimodal Motor Health card, WHAT/WHERE Cards, full telemetry HUD

Reuses all engine/source classes from spatial_acoustic_radar_pro.py.
Launch via:   python diagnostic_hud.py --port COM8
Or via:       Launch_Spatial_Acoustic_Radar.bat  (auto-detected)
=============================================================================
"""

import math
import os
import sys
import time
import threading
import json
from typing import Optional, Dict

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch
import matplotlib.patheffects as pe

# ---------------------------------------------------------------------------
# Import all engine classes from the main radar script (same directory)
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)

from spatial_acoustic_radar_pro import (
    ESP32MultimodalSource,
    Synthetic4ChSource,
    LaptopAudioSource,
    MultimodalMotorEngine,
    AcousticActivityDetector,
    SRPPhatEngine,
    DynamicTargetTracker,
    PRESETS,
    PRELIM_MIC_SENSITIVITY_GAINS,
    UPDATE_INTERVAL_MS,
    apply_pair_phase_compensation,
)

# ─── Colour palette (matches main HUD exactly) ────────────────────────────
BG_DARK    = "#070b12"
BG_PANEL   = "#0b111d"
BG_CARD    = "#101b2d"
CYAN       = "#00ffcc"
BLUE       = "#38bdf8"
AMBER      = "#f59e0b"
MAGENTA    = "#e879f9"
RED        = "#ef4444"
RED_HOT    = "#ff0055"
SLATE      = "#8da2c0"
SLATE_DIM  = "#64748b"
WHITE_DIM  = "#e2e8f0"
BORDER_DIM = "#1e293b"
BORDER_MID = "#2a3b5c"
GRID_COL   = "#34496d"


def _make_vu_bar(pct: float, width: int = 14) -> str:
    filled = int(round((pct / 100.0) * width))
    filled = max(0, min(width, filled))
    return "█" * filled + "░" * (width - filled)


# =============================================================================
# DIAGNOSTIC HUD APP — RIGHT PANEL ONLY
# =============================================================================
class DiagnosticHUDApp:
    """
    Standalone compact diagnostic window.
    Layout: 2 rows × 1 column
      Row 0 (top):    Shaft RPM + Rotational Order bar chart  [ax_order]
      Row 1 (bottom): Multimodal fusion HUD cards + telemetry [ax_telemetry]
    """

    def __init__(self, port: str = "auto", baud: int = 921600, pulses_per_rev: int = 1):
        self.active_source_type = "ESP32"
        self.geometry = PRESETS["ESP32_4MIC_LINEAR"]

        self.sources = {
            "ESP32": ESP32MultimodalSource(port=port, baudrates=(baud, 921600, 1152000, 115200)),
            "SYNTHETIC": Synthetic4ChSource(),
            "LAPTOP": LaptopAudioSource(),
        }
        self.current_source = self.sources[self.active_source_type]
        self.current_source.start()

        self.invert_channels = False
        self.tracker = DynamicTargetTracker(alpha=0.40, gate_deg=30.0)
        self.vad = AcousticActivityDetector(min_active_threshold=0.0035, calib_frames=25, hangover_sec=0.350)
        self.noise_floor = self.vad.noise_floor

        # Phase calibration
        self.calib_delay_sec = 0.0
        self.apply_calibration = False
        proj_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        for c_path in [
            os.path.join(proj_dir, "stage2_acoustic_localization", "evidence", "i2s_phase_calibration.json"),
            os.path.join(proj_dir, "stage4_realtime_fusion_radar", "evidence", "i2s_phase_calibration.json"),
            os.path.join(proj_dir, "evidence", "stage2", "i2s_phase_calibration.json"),
            "i2s_phase_calibration.json",
        ]:
            if os.path.exists(c_path):
                try:
                    with open(c_path, "r") as f:
                        c_data = json.load(f)
                    if "inter_pair_comp_delay_sec" in c_data:
                        self.calib_delay_sec = float(c_data["inter_pair_comp_delay_sec"])
                    elif "inter_pair_mean_skew_us" in c_data:
                        self.calib_delay_sec = float(c_data["inter_pair_mean_skew_us"]) * 1e-6
                    if c_data.get("is_sync_healthy", False):
                        self.apply_calibration = True
                    break
                except Exception:
                    pass

        self.motor_engine = MultimodalMotorEngine()
        self.learn_baseline_requested = False

        self.pulses_options = [1, 2, 4, 10, 20]
        self.pulses_per_rev = max(1, int(pulses_per_rev))
        if self.pulses_per_rev in self.pulses_options:
            self.pulses_per_rev_idx = self.pulses_options.index(self.pulses_per_rev)
        else:
            self.pulses_options.append(self.pulses_per_rev)
            self.pulses_per_rev_idx = len(self.pulses_options) - 1

        self.demo_profile_idx = 0
        self.demo_profiles = [
            "LIVE_HARDWARE",
            "DEMO_HEALTHY_1800RPM",
            "DEMO_UNBALANCE_1X",
            "DEMO_MISALIGNMENT_2X",
            "DEMO_DISTURBANCE_PROOF",
        ]
        self.demo_phase = 0.0
        self.demo_pulses = 120
        self._last_seen_pulses = -1
        self._last_pulse_blink_time = 0.0
        self.paused = False

        self._setup_ui()

    # ─────────────────────────────────────────────────────────────────────
    # UI SETUP — RIGHT PANEL ONLY
    # ─────────────────────────────────────────────────────────────────────
    def _setup_ui(self):
        plt.style.use("dark_background")

        # Narrow portrait window matching the right column aspect
        self.fig = plt.figure(figsize=(7.2, 10.0), facecolor=BG_DARK)

        try:
            if hasattr(self.fig.canvas, "manager") and self.fig.canvas.manager:
                self.fig.canvas.manager.set_window_title(
                    "Spatial Multimodal Edge-AI | Diagnostic HUD (Motor + Fault + DOA)"
                )
        except Exception:
            pass

        # 2 rows: top = RPM/Orders (40%), bottom = Fusion HUD (60%)
        gs = gridspec.GridSpec(
            2, 1,
            height_ratios=[1.0, 1.55],
            hspace=0.30,
            left=0.055, right=0.978, top=0.890, bottom=0.040,
        )

        # ── TOP ROW: Shaft Speed + Rotational Order Bars ─────────────────
        self.ax_order = self.fig.add_subplot(gs[0, 0], facecolor=BG_PANEL)
        self.ax_order.set_facecolor(BG_PANEL)
        for spine in self.ax_order.spines.values():
            spine.set_edgecolor(BORDER_MID)
            spine.set_linewidth(1.1)

        # ── Speedometer text (live big readout)
        self.speedometer_text = self.ax_order.text(
            0.50, 0.93, "○  MOTOR STOPPED / STANDBY (0 RPM)",
            transform=self.ax_order.transAxes,
            ha="center", va="center", color=CYAN, fontsize=11.5, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.30", facecolor=BG_CARD, edgecolor=CYAN, linewidth=1.5),
        )

        # ── Tachometer pulse blinker line
        self.tach_indicator_text = self.ax_order.text(
            0.50, 0.78, "○  FC-03 TACH: READY (0 Pulses — Shaft Idle)",
            transform=self.ax_order.transAxes,
            ha="center", va="center", color=SLATE, fontsize=9.0, fontweight="bold", family="monospace",
        )

        # ── Rotational order bars (1X / 2X / 3X) with subtitle
        bar_x = [0, 1, 2]
        self.order_bars = self.ax_order.bar(
            bar_x, [0.0, 0.0, 0.0],
            width=0.52,
            color=[CYAN, BLUE, MAGENTA],
            edgecolor="#ffffff", linewidth=1.1, zorder=3,
        )
        self.ax_order.set_xticks(bar_x)
        self.ax_order.set_xticklabels(
            ["1X\nunbalance", "2X\nmisalignment", "3X\nharmonic"],
            fontsize=9.0, color="#b8c2d1", fontweight="bold",
        )
        self.ax_order.set_ylim(0, 118)
        self.ax_order.set_ylabel("share of rotational energy (%)", color=SLATE, fontsize=8.5)
        self.ax_order.set_title("HARMONIC ORDERS", color=WHITE_DIM, fontsize=11, weight="bold", pad=6)
        self.ax_order.tick_params(colors="#b8c2d1", labelsize=8.5)
        self.ax_order.grid(alpha=0.15, color=GRID_COL, axis="y")

        # ── Percentage labels on top of bars
        self.bar_labels = [
            self.ax_order.text(
                i, 3.0, "0.0%",
                ha="center", va="bottom", fontsize=9.5, fontweight="bold", color="#ffffff",
            )
            for i in range(3)
        ]

        # ── Right-side label: "share of rotational energy"
        # (already in ylabel; redundant inline tag matches screenshot)
        self._order_subtitle = self.ax_order.text(
            0.98, 1.025, "share of rotational energy",
            transform=self.ax_order.transAxes,
            ha="right", va="bottom", color=SLATE_DIM, fontsize=7.5,
        )

        # ── BOTTOM ROW: Multimodal Fusion HUD ────────────────────────────
        self.ax_telemetry = self.fig.add_subplot(gs[1, 0], facecolor=BG_PANEL)
        self.ax_telemetry.set_title("MOTOR DIAGNOSTICS", color=CYAN, fontsize=11, weight="bold", pad=6)
        self.ax_telemetry.set_xticks([])
        self.ax_telemetry.set_yticks([])
        for spine in self.ax_telemetry.spines.values():
            spine.set_edgecolor(BORDER_MID)
            spine.set_linewidth(1.1)

        # ── Health score big display (top of panel, like screenshot "95.1%")
        self.health_score_text = self.ax_telemetry.text(
            0.50, 0.955,
            "--.-% —— INITIALIZING",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color=CYAN, fontsize=15.0, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.35", facecolor=BG_CARD, edgecolor=CYAN, linewidth=1.8),
        )

        # ── Secondary label below health score ("motor health index")
        self.health_label = self.ax_telemetry.text(
            0.50, 0.900, "motor health index",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color=SLATE_DIM, fontsize=8.0, style="italic",
        )

        # ── WHAT IS WRONG card
        self.what_box_text = self.ax_telemetry.text(
            0.50, 0.826,
            "DIAGNOSIS\nINITIALIZING...",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color=CYAN, fontsize=8.5, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.28", facecolor=BG_CARD, edgecolor=CYAN, linewidth=1.3),
        )

        # ── WHERE card (DOA bearing)
        self.where_box_text = self.ax_telemetry.text(
            0.50, 0.740,
            "DOA BEARING: SCANNING...",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color=BLUE, fontsize=8.5, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.26", facecolor=BG_CARD, edgecolor=BLUE, linewidth=1.2),
        )

        # ── Monospace telemetry block (matches screenshot table rows)
        self.hud_text = self.ax_telemetry.text(
            0.030, 0.670, "Acquiring multimodal telemetry...",
            transform=self.ax_telemetry.transAxes,
            va="top", ha="left", color=WHITE_DIM, fontsize=8.0, family="monospace", linespacing=1.22,
        )

        # ── Top header ────────────────────────────────────────────────────
        self.fig.text(
            0.50, 0.966,
            "SPATIAL MULTIMODAL EDGE-AI  ·  FAULT DETECTION & LOCALIZATION",
            ha="center", va="center", color=CYAN, fontsize=10.0, weight="bold",
        )
        self.title_text = self.fig.text(
            0.50, 0.939,
            "LIVE 4-MIC ULA HARDWARE (COM8, CHK VALIDATED)",
            ha="center", va="center", color=WHITE_DIM, fontsize=8.5, weight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.25", facecolor=BG_CARD, edgecolor=BORDER_MID, linewidth=1.1),
        )

        # ── Source tag line (below hardware badge)
        self.source_tag_text = self.fig.text(
            0.50, 0.912,
            "ESP32 Quad INMP441 Linear Array (20.32mm Pitch, 60.96mm Aperture)",
            ha="center", va="center", color=SLATE, fontsize=7.8,
        )

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    # ─────────────────────────────────────────────────────────────────────
    # KEY HANDLER
    # ─────────────────────────────────────────────────────────────────────
    def _on_key(self, event):
        if event.key in ("q", "escape"):
            plt.close(self.fig)
        elif event.key == " ":
            self.paused = not self.paused
        elif event.key in ("b", "B"):
            self.learn_baseline_requested = True
            print("[+] Golden Baseline capture triggered!")
        elif event.key in ("c", "C"):
            self.motor_engine.reset_baseline()
            print("[+] Golden Baseline cleared.")
        elif event.key in ("t", "T"):
            self.demo_profile_idx = (self.demo_profile_idx + 1) % len(self.demo_profiles)
            print(f"[+] Demo profile: {self.demo_profiles[self.demo_profile_idx]}")
        elif event.key in ("p", "P"):
            self.pulses_per_rev_idx = (self.pulses_per_rev_idx + 1) % len(self.pulses_options)
            self.pulses_per_rev = self.pulses_options[self.pulses_per_rev_idx]
            print(f"[+] Tach PPR = {self.pulses_per_rev}")
        elif event.key in ("k", "K"):
            self.apply_calibration = not self.apply_calibration
            print(f"[+] Phase calib: {'ON' if self.apply_calibration else 'BYPASSED'}")
        elif event.key in ("i", "I"):
            self.invert_channels = not self.invert_channels
        elif event.key in ("1",):
            self._switch_source("LAPTOP")
        elif event.key in ("2",):
            self._switch_source("ESP32")
        elif event.key in ("3",):
            self._switch_source("SYNTHETIC")

    def _switch_source(self, source_type: str):
        if source_type == self.active_source_type:
            return
        self.current_source.stop()
        self.active_source_type = source_type
        self.current_source = self.sources[source_type]
        self.current_source.start()

    # ─────────────────────────────────────────────────────────────────────
    # ANIMATION UPDATE LOOP
    # ─────────────────────────────────────────────────────────────────────
    def update(self, _frame):
        all_artists = [
            self.speedometer_text, self.tach_indicator_text,
            self.health_score_text, self.health_label,
            self.what_box_text, self.where_box_text,
            self.hud_text, self.title_text, self.source_tag_text,
        ] + list(self.order_bars) + self.bar_labels

        if self.paused:
            return all_artists

        try:
            raw_frame = self.current_source.get_frame()
            fs = self.current_source.get_fs()
            channels = raw_frame.shape[1] if raw_frame.ndim > 1 else 1
            now = time.time()

            if self.invert_channels:
                raw_frame = np.fliplr(raw_frame)

            if self.apply_calibration and abs(self.calib_delay_sec) > 1e-12 and raw_frame.shape[1] >= 4:
                raw_frame = apply_pair_phase_compensation(raw_frame, fs, self.calib_delay_sec)

            # ── VAD / RMS ─────────────────────────────────────────────
            gains_vec = PRELIM_MIC_SENSITIVITY_GAINS[:channels].reshape(1, -1)
            calib_audio = raw_frame * gains_vec
            n_samples = min(raw_frame.shape[0], 512)
            rms_per_ch = []
            rms_calib_per_ch = []
            for i in range(min(channels, 4)):
                sig_ac = raw_frame[:n_samples, i] - np.mean(raw_frame[:n_samples, i])
                r_val = float(np.sqrt(np.mean(sig_ac ** 2)))
                gain = float(PRELIM_MIC_SENSITIVITY_GAINS[i]) if i < len(PRELIM_MIC_SENSITIVITY_GAINS) else 1.0
                rms_per_ch.append(r_val)
                rms_calib_per_ch.append(r_val * gain)

            avg_rms = float(np.mean(rms_per_ch)) if rms_per_ch else 0.0
            avg_calib_rms = float(np.mean(rms_calib_per_ch)) if rms_calib_per_ch else avg_rms
            is_active_sound, self.noise_floor, _ = self.vad.process_frame(avg_rms, now)

            # ── SRP-PHAT (needed for DOA + side metrics) ─────────────
            min_srp_rms = max(0.0008, min(0.0022, self.noise_floor * 1.25))
            best_bearing, composite_prom, srp_angles, srp_norm, _, side_metrics = (
                SRPPhatEngine.compute_srp_frame(
                    raw_frame, fs, self.geometry,
                    min_active_rms=min_srp_rms,
                    calib_gains=PRELIM_MIC_SENSITIVITY_GAINS,
                )
            )
            is_source_valid = bool(side_metrics.get("is_valid", False))
            if is_source_valid:
                self.vad.note_coherent_source(now)
                is_active_sound = True
            else:
                is_active_sound = bool(is_active_sound or avg_calib_rms >= min_srp_rms)

            target_angle, target_range, confidence, is_locked = self.tracker.update(
                best_bearing, composite_prom, is_source_valid, nominal_range=1.25
            )
            dir_str = "CENTER" if abs(target_angle) < 6.0 else ("RIGHT" if target_angle > 0 else "LEFT")
            pct_l = float(side_metrics.get("pct_left", 33.3))
            pct_c = float(side_metrics.get("pct_center", 33.3))
            pct_r = float(side_metrics.get("pct_right", 33.3))
            dom_s = str(side_metrics.get("dom_side", "NONE"))

            # ── Telemetry ─────────────────────────────────────────────
            telemetry = self.current_source.get_multimodal_telemetry()
            ax_g = float(telemetry.get("ax_g", 0.0))
            ay_g = float(telemetry.get("ay_g", 0.0))
            az_g = float(telemetry.get("az_g", -0.82))
            pulses = int(telemetry.get("rpm_pulses", 0))
            raw_rpm = float(telemetry.get("estimated_rpm", 0.0))
            rpm = raw_rpm / max(1, self.pulses_per_rev)
            hw_alive = bool(telemetry.get("is_hardware_alive", False))
            vib_hist = self.current_source.get_vibration_history(max_samples=512)

            # ── Demo injection ────────────────────────────────────────
            if self.demo_profile_idx > 0:
                self.demo_phase += 0.06
                t_now = time.time()
                demo_name = self.demo_profiles[self.demo_profile_idx]
                n_sim = 128
                t_arr = np.linspace(t_now - n_sim / 400.0, t_now, n_sim)

                if demo_name == "DEMO_HEALTHY_1800RPM":
                    rpm = 1800.0 + 8.0 * math.sin(self.demo_phase * 2.0)
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f0 = rpm / 60.0
                    ax_arr = 0.045 * np.sin(2.0 * np.pi * f0 * t_arr) + 0.008 * np.random.randn(n_sim)
                    ay_arr = 0.055 * np.cos(2.0 * np.pi * f0 * t_arr) + 0.008 * np.random.randn(n_sim)
                    az_arr = -0.82 + 0.025 * np.sin(4.0 * np.pi * f0 * t_arr) + 0.008 * np.random.randn(n_sim)
                    vib_hist = (t_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                elif demo_name == "DEMO_UNBALANCE_1X":
                    rpm = 1765.0 + 15.0 * math.sin(self.demo_phase * 2.5)
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f0 = rpm / 60.0
                    ax_arr = 0.380 * np.sin(2.0 * np.pi * f0 * t_arr) + 0.025 * np.random.randn(n_sim)
                    ay_arr = 0.420 * np.cos(2.0 * np.pi * f0 * t_arr) + 0.025 * np.random.randn(n_sim)
                    az_arr = -0.82 + 0.110 * np.sin(4.0 * np.pi * f0 * t_arr) + 0.015 * np.random.randn(n_sim)
                    vib_hist = (t_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                elif demo_name == "DEMO_MISALIGNMENT_2X":
                    rpm = 1750.0 + 12.0 * math.sin(self.demo_phase * 3.0)
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f0 = rpm / 60.0
                    ax_arr = 0.140 * np.sin(2.0 * np.pi * f0 * t_arr) + 0.020 * np.random.randn(n_sim)
                    ay_arr = 0.160 * np.cos(2.0 * np.pi * f0 * t_arr) + 0.020 * np.random.randn(n_sim)
                    az_arr = -0.82 + 0.380 * np.sin(4.0 * np.pi * f0 * t_arr) + 0.025 * np.random.randn(n_sim)
                    vib_hist = (t_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                elif demo_name == "DEMO_DISTURBANCE_PROOF":
                    rpm = 1800.0
                    self.demo_pulses += 1
                    pulses = self.demo_pulses
                    f0 = rpm / 60.0
                    ax_arr = 0.040 * np.sin(2.0 * np.pi * f0 * t_arr) + 0.008 * np.random.randn(n_sim)
                    ay_arr = 0.050 * np.cos(2.0 * np.pi * f0 * t_arr) + 0.008 * np.random.randn(n_sim)
                    az_arr = -0.82 + 0.020 * np.sin(4.0 * np.pi * f0 * t_arr) + 0.008 * np.random.randn(n_sim)
                    vib_hist = (t_arr, ax_arr.astype(np.float32), ay_arr.astype(np.float32), az_arr.astype(np.float32))
                    ax_g, ay_g, az_g = float(ax_arr[-1]), float(ay_arr[-1]), float(az_arr[-1])
                    raw_frame = (raw_frame + 0.15 * np.random.randn(*raw_frame.shape)).astype(np.float32)

            motor_diag = self.motor_engine.analyze(
                raw_frame, fs, ax_g, ay_g, az_g, rpm, vib_history=vib_hist
            )
            dyn_vib_g = float(motor_diag.get("dyn_vib_g", 0.0))

            if self.learn_baseline_requested:
                self.motor_engine.learn_baseline(rpm, dyn_vib_g, raw_frame, fs)
                self.learn_baseline_requested = False

            is_running = bool(motor_diag.get("is_running", False))
            iso_short = motor_diag["iso_severity"].split(":")[0]
            f0_hz = float(motor_diag.get("f0_hz", 0.0))
            e1 = float(motor_diag.get("e_1x_pct", 0.0))
            e2 = float(motor_diag.get("e_2x_pct", 0.0))
            e3 = float(motor_diag.get("e_3x_pct", 0.0))

            # ── Health score ──────────────────────────────────────────
            if not is_running:
                h_score_str = " --.-%"
                h_status = "STANDBY"
                h_color = SLATE
            else:
                h_score = float(motor_diag["health_score"])
                h_score_str = f"{h_score:5.1f}%"
                if h_score >= 85:
                    h_status = "EXCELLENT"
                    h_color = CYAN
                elif h_score >= 65:
                    h_status = "PERMISSIBLE"
                    h_color = "#a3e635"
                elif h_score >= 40:
                    h_status = "WARNING"
                    h_color = AMBER
                else:
                    h_status = "CRITICAL"
                    h_color = RED_HOT

            self.health_score_text.set_text(f"{h_score_str}   {h_status}")
            self.health_score_text.set_color(h_color)
            self.health_score_text.get_bbox_patch().set_edgecolor(h_color)

            # ── Speedometer ───────────────────────────────────────────
            if is_running:
                self.speedometer_text.set_text(f"● RUNNING  ·  f0 = {f0_hz:.1f} Hz")
                self.speedometer_text.set_color(CYAN)
                self.speedometer_text.get_bbox_patch().set_edgecolor(CYAN)
            else:
                self.speedometer_text.set_text("○  MOTOR STOPPED / STANDBY (0 RPM)")
                self.speedometer_text.set_color(SLATE)
                self.speedometer_text.get_bbox_patch().set_edgecolor(SLATE)

            # ── FC-03 blinker ─────────────────────────────────────────
            if pulses > self._last_seen_pulses:
                self._last_pulse_blink_time = now
                self._last_seen_pulses = pulses
            is_blinking = (now - self._last_pulse_blink_time) < 0.25
            if is_running:
                rpm_str = f"{rpm:.0f} RPM"
            else:
                rpm_str = "0 RPM"

            if is_blinking or (is_running and (int(now * 8) % 2 == 0)):
                self.tach_indicator_text.set_text(f"● FC-03 pulse active  ·  {rpm_str}  ·  PPR {self.pulses_per_rev}")
                self.tach_indicator_text.set_color(CYAN)
            elif pulses > 0:
                self.tach_indicator_text.set_text(f"○  FC-03 pulse active  ·  {rpm_str}  ·  {pulses:,} pulses")
                self.tach_indicator_text.set_color(SLATE)
            else:
                self.tach_indicator_text.set_text(f"○  FC-03 TACH: READY (0 Pulses — Shaft Idle)")
                self.tach_indicator_text.set_color(SLATE_DIM)

            # ── Order bars ────────────────────────────────────────────
            c1 = RED if (e1 > 18.0 and dyn_vib_g >= 0.14) else (AMBER if (e1 > 10.0 and dyn_vib_g >= 0.10) else CYAN)
            c2 = MAGENTA if e2 > 12.0 else ("#a855f7" if e2 > 6.0 else BLUE)
            c3 = RED_HOT if e3 > 10.0 else SLATE_DIM

            self.order_bars[0].set_height(e1)
            self.order_bars[0].set_color(c1)
            self.order_bars[1].set_height(e2)
            self.order_bars[1].set_color(c2)
            self.order_bars[2].set_height(e3)
            self.order_bars[2].set_color(c3)

            self.bar_labels[0].set_position((0, max(2.5, e1 + 2.5)))
            self.bar_labels[0].set_text(f"{e1:.1f}%")
            self.bar_labels[0].set_color(c1)
            self.bar_labels[1].set_position((1, max(2.5, e2 + 2.5)))
            self.bar_labels[1].set_text(f"{e2:.1f}%")
            self.bar_labels[1].set_color(c2)
            self.bar_labels[2].set_position((2, max(2.5, e3 + 2.5)))
            self.bar_labels[2].set_text(f"{e3:.1f}%")
            self.bar_labels[2].set_color(c3)

            # ── WHAT card ─────────────────────────────────────────────
            fault_str = str(motor_diag["primary_fault"])
            adxl_short = "ONLINE (FIFO)" if telemetry.get("adxl_hw_ok", False) else "CHECK I2C"
            if self.demo_profile_idx > 0:
                adxl_short = "DEMO (400Hz SIM)"

            if "DISTURBANCE" in fault_str:
                what_title = "ACOUSTIC DISTURBANCE DETECTED (Vibration Nominal\n→ Non-Mechanical)"
                what_col = AMBER
            elif not is_running:
                what_title = "MOTOR STOPPED / STANDBY\nAwaiting shaft rotation..."
                what_col = SLATE
            elif "NOMINAL" in fault_str:
                what_title = "NOMINAL / HEALTHY MOTOR"
                what_col = CYAN
            else:
                what_title = fault_str[:46]
                what_col = RED_HOT

            self.what_box_text.set_text(f"DIAGNOSIS\n{what_title}")
            self.what_box_text.set_color(what_col)
            self.what_box_text.get_bbox_patch().set_edgecolor(what_col)

            # ── WHERE card ────────────────────────────────────────────
            if is_source_valid:
                where_title = f"DOA: {target_angle:+.1f}°  [{dir_str}]  Lock: {'YES' if is_locked else 'SEARCHING'}"
                where_col = CYAN if is_locked else AMBER
            elif is_active_sound:
                where_title = f"SOUND PRESENT  ·  Diffuse / Unresolved"
                where_col = AMBER
            else:
                where_title = "QUIET AMBIENT FIELD  ·  No Target"
                where_col = SLATE_DIM

            self.where_box_text.set_text(f"BEARING  →  {where_title}")
            self.where_box_text.set_color(where_col)
            self.where_box_text.get_bbox_patch().set_edgecolor(where_col)

            # ── Telemetry text block ──────────────────────────────────
            if motor_diag["baseline_locked"]:
                d = motor_diag["delta"]
                base_str = f"LOCKED (ΔVib:{d['d_vib_g']:+.2f}g, ΔRPM:{d['d_rpm']:+.0f})"
            else:
                base_str = "UNLOCKED — press B to learn"

            port_label = str(telemetry.get("port", "COM8"))
            calib_short = "ON" if self.apply_calibration else "RAW"
            demo_badge = f"DEMO: {self.demo_profiles[self.demo_profile_idx]}" if self.demo_profile_idx > 0 else "LIVE HARDWARE"

            hud_lines = [
                f" BASELINE   {base_str[:40]}",
                f" TACH       {rpm:.0f} RPM — f0 = {f0_hz:.1f} Hz — PPR {self.pulses_per_rev}",
                f" VIBRATION  a_dyn {dyn_vib_g:.3f} g — {motor_diag.get('v_rms_mms', 0.0):.2f} mm/s — {iso_short}",
                f" ACOUSTIC   Kurtosis {motor_diag['kurtosis']:.2f} — Crest {motor_diag['crest_factor']:.1f}",
                f" ORDERS     1X {e1:.1f}%  2X {e2:.1f}%  3X {e3:.1f}%",
                f" FLANKS     L {pct_l:.0f}%  C {pct_c:.0f}%  R {pct_r:.0f}%",
                f" LINK       {port_label} @ 921.6k  OK:{telemetry.get('packets_valid', 0):,}  drop {telemetry.get('dropped_packets', 0)}",
                f" CALIB/VAD  Phase {'BYPASS' if not self.apply_calibration else 'ACTIVE'} — VAD floor {self.noise_floor:.4f}",
                " ──────────────────────────────────────────",
                " [B] Learn Baseline  [C] Clear  [P] PPR  [T] Demo",
                " [1] Laptop  [2] ESP32  [K] Cal  [I] Flip  [Q] Quit",
            ]
            self.hud_text.set_text("\n".join(hud_lines))

            # ── Top header update ─────────────────────────────────────
            if self.demo_profile_idx > 0:
                tag = f"DEMO MODE: {self.demo_profiles[self.demo_profile_idx]}"
                tag_col = AMBER
            elif hw_alive:
                tag = f"LIVE 4-MIC ULA HARDWARE ({port_label}, CHK VALIDATED)"
                tag_col = CYAN
            else:
                tag = f"ESP32 STANDBY / DISCONNECTED ({port_label})  |  Press T for Demo"
                tag_col = SLATE
            self.title_text.set_text(tag)
            self.title_text.set_color(tag_col)
            self.title_text.get_bbox_patch().set_edgecolor(tag_col)

        except Exception as e:
            if not hasattr(self, "_last_err") or time.time() - self._last_err > 3.0:
                print(f"[!] HUD update error: {e}", file=sys.stderr)
                self._last_err = time.time()

        return all_artists

    def run(self):
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            interval=UPDATE_INTERVAL_MS,
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout(rect=[0, 0, 1, 0.91])
        plt.show()


# =============================================================================
# ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Spatial Multimodal Diagnostic HUD (Right Panel Only)")
    parser.add_argument("--port", type=str, default="auto", help="ESP32 serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baudrate (default: 921600)")
    parser.add_argument("--ppr", "--pulses-per-rev", type=int, default=1, help="Tachometer pulses per revolution")
    args, _ = parser.parse_known_args()

    print("=" * 65)
    print(" SPATIAL MULTIMODAL EDGE-AI — DIAGNOSTIC HUD (RIGHT PANEL)")
    print(f" Port: {args.port} @ {args.baud} Baud | PPR: {args.ppr}")
    print("=" * 65)

    app = DiagnosticHUDApp(port=args.port, baud=args.baud, pulses_per_rev=args.ppr)
    app.run()
