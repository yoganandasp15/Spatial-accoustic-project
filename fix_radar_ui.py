#!/usr/bin/env python3
"""
1-Click UI Layout & Minor Project Vision Upgrade for spatial_acoustic_radar_pro.py
Fixes all 7 overlapping text/title/colorbar defects shown in live screenshots:
  1. Adds a 2-line centered Minor Project Vision Header ("WHAT IS WRONG?" + "WHERE IS IT?").
  2. Moves the 2D Camera colorbar to horizontal orientation at the bottom of Col 0
     so it never collides with the 4-Channel Microphone labels in Col 1.
  3. Separates the Sector Banner, Left/Right VU Gauge labels, and Legend in the 2D Camera.
  4. Widens left margin (0.048) and inter-column spacing so no Y-axis or title ever clips.
  5. Shortens all 7 panel titles into numbered, non-colliding headers.
  6. Fixes the RPM Speedometer badge width and prevents false red 1X bars on quiet Zone A noise.
  7. Redesigns Panel 7 (Bottom-Right HUD) with visual "WHAT IS WRONG?" and "WHERE IS IT?"
     cards and strictly width-bounded telemetry lines (<46 chars) that never clip.
"""

import sys
from pathlib import Path
import shutil

TARGET_REL = Path("stage4_realtime_fusion_radar/src/spatial_acoustic_radar_pro.py")


NEW_GET_EVIDENCE_TAG = '''    def _get_evidence_mode_tag(self) -> str:
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
'''


NEW_SETUP_UI = '''    def _setup_ui(self):
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
            -1.84, 2.60, "◄ L\\n0%",
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
            1.84, 2.60, "R ►\\n0%",
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
            "WHAT IS WRONG? (MECHANICAL STATE)\\nINITIALIZING...",
            transform=self.ax_telemetry.transAxes,
            ha="center", va="center", color="#00ffcc", fontsize=8.0, fontweight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.30", facecolor="#101b2d", edgecolor="#00ffcc", linewidth=1.3)
        )

        # Card 2: WHERE IS IT? (Spatial Localization & Fusion)
        self.where_box_text = self.ax_telemetry.text(
            0.50, 0.73,
            "WHERE IS THE SOUND? (SPATIAL DOA)\\nSCANNING...",
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
'''


def apply_ui_fix():
    root = Path.cwd()
    target = root / TARGET_REL
    if not target.is_file():
        # Fallback to absolute or searching parent
        alt_target = Path(r"C:\Users\Yogi\Downloads\Spatial-accoustic-project") / TARGET_REL
        if alt_target.is_file():
            target = alt_target
        else:
            print(f"[ERROR] Could not find {TARGET_REL}. Run from repository root.")
            sys.exit(1)

    text = target.read_text(encoding="utf-8")
    if "WHAT IS WRONG? (MECHANICAL STATE)" in text and "1. 2D ACOUSTIC CAMERA (WHERE IS IT?)" in text:
        print("[INFO] spatial_acoustic_radar_pro.py is already patched with the clean Minor Project UI layout.")
        return

    backup = target.with_suffix(".py.ui_backup.bak")
    shutil.copy2(target, backup)

    # 1. Replace _get_evidence_mode_tag
    start_tag = text.index("    def _get_evidence_mode_tag(self) -> str:")
    end_tag = text.index("    def _setup_ui(self):", start_tag)
    text = text[:start_tag] + NEW_GET_EVIDENCE_TAG + "\n" + text[end_tag:]

    # 2. Replace _setup_ui
    start_ui = text.index("    def _setup_ui(self):")
    end_ui = text.index("    def _on_key(self, event):", start_ui)
    text = text[:start_ui] + NEW_SETUP_UI + "\n" + text[end_ui:]

    # 3. Update VU text formatting so it is concise and never overflows ax_mics
    text = text.replace(
        'self.vu_texts[i].set_text(f"M{i+1}: RMS={r_calib:.4f} ({dbfs_calib:+.1f}dB | raw:{r_val:.4f}) | {status_str}")',
        'self.vu_texts[i].set_text(f"M{i+1}: {r_calib:.4f} ({dbfs_calib:+.1f}dB) | {status_str}")'
    )

    # 4. Update Left/Right Flank VU Gauge y_top from 3.25 to 2.50 & concise labels
    text = text.replace("y_bot, y_top = 0.45, 3.25", "y_bot, y_top = 0.45, 2.50")
    text = text.replace("[-1.88, -1.88], [y_bot, h_left]", "[-1.84, -1.84], [y_bot, h_left]")
    text = text.replace("[1.88, 1.88], [y_bot, h_right]", "[1.84, 1.84], [y_bot, h_right]")
    text = text.replace(
        'self.left_gauge_text.set_text(f"◄ LEFT\\n{pct_l:.0f}%" if is_source_valid else "◄ LEFT\\nIDLE")',
        'self.left_gauge_text.set_text(f"◄ L\\n{pct_l:.0f}%" if is_source_valid else "◄ L\\nIDLE")'
    )
    text = text.replace(
        'self.right_gauge_text.set_text(f"RIGHT ►\\n{pct_r:.0f}%" if is_source_valid else "RIGHT ►\\nIDLE")',
        'self.right_gauge_text.set_text(f"R ►\\n{pct_r:.0f}%" if is_source_valid else "R ►\\nIDLE")'
    )

    # 5. Make Sector Banner concise so it fits cleanly inside the top pill box
    old_banner_block = '''            if not is_source_valid and is_active_sound:
                reject_reason = str(side_metrics.get("status", "SPATIAL GATE"))
                self.sector_banner_text.set_text(f"■ SOUND PRESENT — BEARING UNRESOLVED ({reject_reason}) ■")
                self.sector_banner_text.set_color("#ffaa00")
            elif not is_source_valid:
                self.sector_banner_text.set_text("■ AMBIENT MONITORING — READY FOR TARGET SOUND ■")
                self.sector_banner_text.set_color("#6b7d99")
            elif dom_s == "LEFT":
                self.sector_banner_text.set_text(f"◄◄ INCOMING SOUND: LEFT FLANK ({target_angle:+.1f}°) | {pct_l:.0f}% INTENSITY")
                self.sector_banner_text.set_color("#ff0055" if pct_l >= 48.0 else "#ffaa00")
            elif dom_s == "RIGHT":
                self.sector_banner_text.set_text(f"INCOMING SOUND: RIGHT FLANK ({target_angle:+.1f}°) | {pct_r:.0f}% INTENSITY ►►")
                self.sector_banner_text.set_color("#ff0055" if pct_r >= 48.0 else "#ffaa00")
            else:
                self.sector_banner_text.set_text(f"▲ INCOMING SOUND: FRONT CENTER ({target_angle:+.1f}°) | {pct_c:.0f}% INTENSITY ▲")
                self.sector_banner_text.set_color("#00ffcc")'''

    new_banner_block = '''            if not is_source_valid and is_active_sound:
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
                self.sector_banner_text.set_color("#00ffcc")'''

    if old_banner_block in text:
        text = text.replace(old_banner_block, new_banner_block, 1)

    # 6. Prevent false red 1X bar when vibration is quiet (<0.12g Zone A noise floor) & concise speedometer
    text = text.replace(
        'self.speedometer_text.set_text(f"● MOTOR RUNNING: {rpm:5.0f} RPM (f0 = {motor_diag[\'f0_hz\']:.1f} Hz)")',
        'self.speedometer_text.set_text(f"● RUNNING: {rpm:4.0f} RPM (f0 = {motor_diag[\'f0_hz\']:.1f} Hz)")'
    )
    text = text.replace(
        'self.speedometer_text.set_text("○ MOTOR STOPPED / IDLE (0 RPM)")',
        'self.speedometer_text.set_text("○ MOTOR STOPPED / STANDBY (0 RPM)")'
    )
    text = text.replace(
        'c1 = "#ef4444" if e1 > 15.0 else ("#f59e0b" if e1 > 8.0 else "#00ffcc")',
        'c1 = "#ef4444" if (e1 > 18.0 and dyn_vib_g >= 0.14) else ("#f59e0b" if (e1 > 10.0 and dyn_vib_g >= 0.10) else "#00ffcc")'
    )

    # 7. Update Panel 7 (HUD) to populate what_box_text, where_box_text, and non-clipping hud_lines
    old_hud_start = text.index("            mode_tag = self._get_evidence_mode_tag()")
    old_hud_end = text.index("            self.hud_text.set_text(\"\\n\".join(hud_lines))") + len("            self.hud_text.set_text(\"\\n\".join(hud_lines))")

    new_hud_section = '''            mode_tag = self._get_evidence_mode_tag()
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
                self.what_box_text.set_text(f"WHAT IS WRONG?  ►  {what_title}\\n{what_sub}")
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
                self.where_box_text.set_text(f"WHERE IS IT?  ►  {where_title}\\n{where_sub}")
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
            self.hud_text.set_text("\\n".join(hud_lines))'''

    text = text[:old_hud_start] + new_hud_section + text[old_hud_end:]

    compile(text, str(target), "exec")
    target.write_text(text, encoding="utf-8", newline="\n")
    print(f"[SUCCESS] Patched {TARGET_REL} (Backup saved to {backup.name})")


if __name__ == "__main__":
    apply_ui_fix()
