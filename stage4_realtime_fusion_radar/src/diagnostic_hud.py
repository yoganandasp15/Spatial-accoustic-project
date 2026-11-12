#!/usr/bin/env python3
"""
Diagnostic HUD: Compact Real-Time Multimodal Diagnostic & Tachometer Station.
Focuses on:
  - High-Speed Shaft Tachometer (0 to 120,000 RPM with dynamic PPR scaling)
  - ADXL345 3-Axis Vibration & ISO-10816 Zone Evaluation
  - Multimodal Fault Diagnosis (Healthy vs Unbalance 1X vs Misalignment 2X)
  - GCC-PHAT Direction of Arrival (DOA) Acoustic Bearing Angle
Supports ESP32 Hardware, Laptop/PC Microphone, and Interactive High-Speed Simulator.
"""

import sys
import os
import time
import math
import argparse
import threading
from typing import Dict, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

# Ensure stage4 src is in sys.path
SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from spatial_acoustic_radar_pro import (
    AudioSourceBase, ESP32MultimodalSource, Synthetic4ChSource, LaptopAudioSource,
    MultimodalMotorEngine, DynamicTargetTracker, PRESETS, SPEED_OF_SOUND
)

class DiagnosticHUDApp:
    def __init__(self, port: str = "auto", baud: int = 921600, pulses_per_rev: int = 1,
                 source: str = "auto", initial_rpm: float = 0.0):
        self.port = port
        self.baud = baud
        self.pulses_per_rev = max(1, int(pulses_per_rev))
        self.pulses_options = [1, 2, 4, 10, 20]
        if self.pulses_per_rev in self.pulses_options:
            self.ppr_idx = self.pulses_options.index(self.pulses_per_rev)
        else:
            self.pulses_options.append(self.pulses_per_rev)
            self.ppr_idx = len(self.pulses_options) - 1

        self.sources: Dict[str, AudioSourceBase] = {
            "ESP32": ESP32MultimodalSource(port=port, baudrates=(baud, 921600, 1152000, 115200)),
            "SYNTHETIC": Synthetic4ChSource(target_rpm=initial_rpm, ppr=self.pulses_per_rev),
            "LAPTOP": LaptopAudioSource()
        }

        src_norm = (source or "auto").lower()
        if src_norm == "laptop":
            self.active_source_type = "LAPTOP"
        elif src_norm == "synthetic":
            self.active_source_type = "SYNTHETIC"
        elif src_norm == "esp32":
            self.active_source_type = "ESP32"
        else:
            is_esp32_present = False
            try:
                import serial.tools.list_ports as lp
                for p in lp.comports():
                    h = ((p.hwid or "") + " " + (p.description or "")).lower()
                    if any(b in h for b in ["bthenum", "bluetooth", "acpi", "pnp0501"]): continue
                    if any(k in h for k in ["10c4:ea60", "cp210", "1a86:7523", "ch340", "0403:6001", "ftdi", "303a:", "esp32"]):
                        is_esp32_present = True
                        break
            except Exception:
                pass
            self.active_source_type = "ESP32" if is_esp32_present else "SYNTHETIC"

        self.current_source = self.sources[self.active_source_type]
        self.current_source.start()

        self.motor_engine = MultimodalMotorEngine()
        self.tracker = DynamicTargetTracker(alpha=0.35, gate_deg=30.0)
        self.learn_baseline_requested = False

        self._setup_ui()

    def _setup_ui(self):
        plt.style.use("dark_background")
        self.fig = plt.figure(figsize=(15.5, 8.8), facecolor="#080d1a")
        try:
            if hasattr(self.fig.canvas, "manager") and self.fig.canvas.manager:
                self.fig.canvas.manager.set_window_title(
                    "Multimodal Diagnostic HUD | High-Speed Tachometer & Machine Health Station"
                )
        except Exception:
            pass

        gs = gridspec.GridSpec(2, 2, hspace=0.32, wspace=0.25, left=0.06, right=0.96, top=0.90, bottom=0.08)

        # ---------------------------------------------------------------------
        # Top Header Banner
        # ---------------------------------------------------------------------
        self.title_text = self.fig.text(
            0.50, 0.96, "MULTIMODAL DIAGNOSTIC HUD | HIGH-SPEED TACHOMETER & FAULT DETECTOR",
            ha="center", va="center", color="#38bdf8", fontsize=14, weight="bold", family="monospace"
        )
        self.sub_text = self.fig.text(
            0.50, 0.925, "Source: ESP32 | Controls: [1] PC Mic  [2] ESP32  [3] Sim  [UP/DN] Speed  [P] PPR  [Q] Quit",
            ha="center", va="center", color="#94a3b8", fontsize=9.5, family="monospace"
        )

        # ---------------------------------------------------------------------
        # Panel 1: High-Speed Tachometer Meter (gs[0, 0])
        # ---------------------------------------------------------------------
        self.ax_tach = self.fig.add_subplot(gs[0, 0], facecolor="#0f172a")
        self.ax_tach.set_title("SHAFT TACHOMETER & SPEED TELEMETRY", color="#38bdf8", fontsize=11, weight="bold", pad=8)
        self.ax_tach.set_xlim(-1.2, 1.2)
        self.ax_tach.set_ylim(-0.4, 1.2)
        self.ax_tach.axis("off")

        # Gauge arc background
        angles = np.linspace(np.pi, 0, 100)
        self.ax_tach.plot(np.cos(angles), np.sin(angles), color="#334155", lw=12, solid_capstyle="round")
        self.tach_fill, = self.ax_tach.plot([0], [0], color="#22c55e", lw=12, solid_capstyle="round")
        self.tach_needle, = self.ax_tach.plot([0, 0], [0, 0.85], color="#f8fafc", lw=2.5)

        self.tach_val_text = self.ax_tach.text(
            0.0, 0.15, "0 RPM", ha="center", va="center", color="#22c55e",
            fontsize=22, weight="bold", family="monospace"
        )
        self.tach_detail_text = self.ax_tach.text(
            0.0, -0.22, "f0: 0.0 Hz | Pulses: 0 | PPR: 1", ha="center", va="center",
            color="#94a3b8", fontsize=10.5, family="monospace"
        )

        # ---------------------------------------------------------------------
        # Panel 2: Real-Time DOA Bearing Compass (gs[0, 1])
        # ---------------------------------------------------------------------
        self.ax_doa = self.fig.add_subplot(gs[0, 1], polar=True, facecolor="#0f172a")
        self.ax_doa.set_title("ACOUSTIC DIRECTION OF ARRIVAL (BEARING)", color="#38bdf8", fontsize=11, weight="bold", pad=12)
        self.ax_doa.set_theta_zero_location("N")
        self.ax_doa.set_theta_direction(-1)
        self.ax_doa.set_thetalim(-np.pi/2, np.pi/2)
        self.ax_doa.set_rticks([])
        self.ax_doa.grid(color="#334155", linestyle="--", alpha=0.7)
        self.doa_needle, = self.ax_doa.plot([0, 0], [0, 1.0], color="#ec4899", lw=3.0, zorder=5)
        self.doa_text = self.ax_doa.text(
            0.0, -0.25, "BEARING: 0.0°", ha="center", va="center", color="#ec4899",
            fontsize=12, weight="bold", family="monospace", transform=self.ax_doa.transAxes
        )

        # ---------------------------------------------------------------------
        # Panel 3: 3-Axis Dynamic Vibration Scope (gs[1, 0])
        # ---------------------------------------------------------------------
        self.ax_vib = self.fig.add_subplot(gs[1, 0], facecolor="#0f172a")
        self.ax_vib.set_title("ADXL345 3-AXIS DYNAMIC VIBRATION (g)", color="#38bdf8", fontsize=11, weight="bold", pad=8)
        self.ax_vib.set_ylim(-2.0, 2.0)
        self.ax_vib.grid(color="#1e293b", linestyle=":", alpha=0.8)
        self.ax_vib.tick_params(colors="#94a3b8", labelsize=8)
        self.line_vx, = self.ax_vib.plot([], [], color="#38bdf8", lw=1.2, label="X (Radial)")
        self.line_vy, = self.ax_vib.plot([], [], color="#a855f7", lw=1.2, label="Y (Radial)")
        self.line_vz, = self.ax_vib.plot([], [], color="#22c55e", lw=1.2, label="Z (Axial)")
        self.ax_vib.legend(loc="upper right", fontsize=8, facecolor="#0f172a", edgecolor="#334155")
        self.vib_status_text = self.ax_vib.text(
            0.02, 0.08, "a_dyn: 0.000g | Zone: N/A",
            transform=self.ax_vib.transAxes, color="#f8fafc", fontsize=9.5, family="monospace"
        )

        # ---------------------------------------------------------------------
        # Panel 4: Multimodal Fault Classifier & Order Energy (gs[1, 1])
        # ---------------------------------------------------------------------
        self.ax_diag = self.fig.add_subplot(gs[1, 1], facecolor="#0f172a")
        self.ax_diag.set_title("MULTIMODAL FAULT CLASSIFIER & ORDER SPECTRUM", color="#38bdf8", fontsize=11, weight="bold", pad=8)
        self.ax_diag.set_xlim(0, 4)
        self.ax_diag.set_ylim(0, 100)
        self.ax_diag.set_xticks([0.8, 1.8, 2.8])
        self.ax_diag.set_xticklabels(["1X (Unbalance)", "2X (Misalign)", "3X (Harmonic)"], color="#94a3b8", fontsize=9)
        self.ax_diag.set_ylabel("Energy Distribution (%)", color="#94a3b8", fontsize=8.5)
        self.ax_diag.grid(color="#1e293b", linestyle=":", alpha=0.8)
        self.bars = self.ax_diag.bar([0.8, 1.8, 2.8], [0, 0, 0], width=0.6, color=["#38bdf8", "#f59e0b", "#ef4444"])
        self.fault_banner = self.ax_diag.text(
            0.5, 0.88, "HEALTH STATUS: INITIALIZING", transform=self.ax_diag.transAxes,
            ha="center", va="center", color="#22c55e", fontsize=12, weight="bold", family="monospace",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#0f172a", edgecolor="#22c55e", lw=1.5)
        )

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

    def _on_key(self, event):
        if event.key in ("q", "escape"):
            plt.close(self.fig)
        elif event.key in ("1",):
            self._switch_source("LAPTOP")
        elif event.key in ("2",):
            self._switch_source("ESP32")
        elif event.key in ("3",):
            self._switch_source("SYNTHETIC")
        elif event.key in ("up", "+", "="):
            if self.active_source_type == "SYNTHETIC":
                self.sources["SYNTHETIC"].adjust_rpm(+2500.0)
                rpm = self.sources["SYNTHETIC"].target_rpm
                print(f"[+] Simulator RPM: {rpm:,.0f} RPM")
        elif event.key in ("down", "-", "_"):
            if self.active_source_type == "SYNTHETIC":
                self.sources["SYNTHETIC"].adjust_rpm(-2500.0)
                rpm = self.sources["SYNTHETIC"].target_rpm
                print(f"[+] Simulator RPM: {rpm:,.0f} RPM")
        elif event.key in ("p", "P"):
            self.ppr_idx = (self.ppr_idx + 1) % len(self.pulses_options)
            self.pulses_per_rev = self.pulses_options[self.ppr_idx]
            if hasattr(self.sources["SYNTHETIC"], "set_ppr"):
                self.sources["SYNTHETIC"].set_ppr(self.pulses_per_rev)
            print(f"[+] Tachometer PPR: {self.pulses_per_rev}")
        elif event.key in ("b", "B"):
            self.learn_baseline_requested = True
            print("[+] Golden Baseline Learning Triggered")
        elif event.key in ("c", "C"):
            self.motor_engine.reset_baseline()
            print("[+] Golden Baseline Cleared")

    def _switch_source(self, stype: str):
        if stype == self.active_source_type:
            return
        print(f"[+] Switching Source to: {stype}")
        self.current_source.stop()
        self.active_source_type = stype
        self.current_source = self.sources[stype]
        self.current_source.start()

    def update(self, _frame):
        frame = self.current_source.get_frame()
        fs = self.current_source.get_fs()
        telem = self.current_source.get_multimodal_telemetry()
        t_hist, ax_hist, ay_hist, az_hist = self.current_source.get_vibration_history()

        pulses = int(telem.get("rpm_pulses", 0))
        raw_rpm = float(telem.get("estimated_rpm", 0.0))
        if self.active_source_type == "SYNTHETIC":
            rpm = float(getattr(self.sources["SYNTHETIC"], "target_rpm", 0.0))
        else:
            rpm = raw_rpm / max(1, self.pulses_per_rev)

        f0 = rpm / 60.0
        ax_g = float(telem.get("ax_g", 0.0))
        ay_g = float(telem.get("ay_g", 0.0))
        az_g = float(telem.get("az_g", -0.82))

        motor_diag = self.motor_engine.analyze(frame, fs, ax_g, ay_g, az_g, rpm, vib_history=(t_hist, ax_hist, ay_hist, az_hist))
        dyn_vib_g = float(motor_diag.get("dyn_vib_g", 0.0))

        if self.learn_baseline_requested:
            self.motor_engine.learn_baseline(rpm, dyn_vib_g, frame, fs)
            self.learn_baseline_requested = False

        # 1. Update Tachometer Dial
        max_rpm_gauge = 120000.0 if rpm > 20000.0 else 30000.0
        gauge_frac = min(1.0, max(0.0, rpm / max_rpm_gauge))
        gauge_ang = np.pi * (1.0 - gauge_frac)
        fill_angles = np.linspace(np.pi, gauge_ang, max(2, int(gauge_frac * 50)))
        self.tach_fill.set_data(np.cos(fill_angles), np.sin(fill_angles))
        needle_x = 0.85 * np.cos(gauge_ang)
        needle_y = 0.85 * np.sin(gauge_ang)
        self.tach_needle.set_data([0, needle_x], [0, needle_y])

        tach_col = "#22c55e" if rpm < 35000.0 else ("#f59e0b" if rpm < 75000.0 else "#ef4444")
        self.tach_val_text.set_text(f"{rpm:,.0f} RPM")
        self.tach_val_text.set_color(tach_col)
        self.tach_fill.set_color(tach_col)
        self.tach_detail_text.set_text(f"f0: {f0:.1f} Hz | Pulses: {pulses:,d} | PPR: {self.pulses_per_rev}")

        # 2. Update Vibration Scope
        if len(ax_hist) > 0:
            x_pts = np.arange(len(ax_hist))
            self.line_vx.set_data(x_pts, ax_hist)
            self.line_vy.set_data(x_pts, ay_hist)
            self.line_vz.set_data(x_pts, az_hist)
            self.ax_vib.set_xlim(0, max(10, len(ax_hist)))
            y_max = max(1.5, np.max(np.abs([ax_hist, ay_hist, az_hist])) * 1.25)
            self.ax_vib.set_ylim(-y_max, y_max)

        iso_str = motor_diag.get("iso_zone_label", "Zone A (Good)")
        self.vib_status_text.set_text(f"a_dyn: {dyn_vib_g:.3f}g | {iso_str}")

        # 3. Update DOA
        target_deg = 0.0
        if frame.shape[1] >= 2 and np.max(np.abs(frame)) > 0.01:
            try:
                L = frame[:, 0]
                R = frame[:, 1]
                cross = np.fft.rfft(L) * np.conj(np.fft.rfft(R))
                phat = cross / (np.abs(cross) + 1e-6)
                gcc = np.fft.fftshift(np.fft.irfft(phat))
                lags = np.arange(-len(gcc)//2, len(gcc)//2) / fs
                max_tau = (0.02032 / SPEED_OF_SOUND) + 1.0/fs
                valid = np.abs(lags) <= max_tau
                if np.any(valid):
                    best_lag = lags[valid][np.argmax(gcc[valid])]
                    sin_th = np.clip((SPEED_OF_SOUND * best_lag) / 0.02032, -1.0, 1.0)
                    target_deg = float(np.degrees(np.arcsin(sin_th)))
            except Exception:
                pass
        self.tracker.update(target_deg, 1.0, True)
        th_rad = np.deg2rad(self.tracker.tracked_angle)
        self.doa_needle.set_data([th_rad, th_rad], [0, 1.0])
        self.doa_text.set_text(f"BEARING: {self.tracker.tracked_angle:+.1f}°")

        # 4. Update Fault Classifier & Order Energy
        e1 = float(motor_diag.get("e_1x_pct", 50.0))
        e2 = float(motor_diag.get("e_2x_pct", 25.0))
        e3 = float(motor_diag.get("e_3x_pct", 10.0))
        for bar, h in zip(self.bars, [e1, e2, e3]):
            bar.set_height(h)

        fault_label = motor_diag.get("diagnostic_label", "NOMINAL / HEALTHY")
        f_col = "#22c55e" if "HEALTHY" in fault_label or "NOMINAL" in fault_label else "#ef4444"
        self.fault_banner.set_text(f"{fault_label[:36]}")
        self.fault_banner.set_color(f_col)
        self.fault_banner.get_bbox_patch().set_edgecolor(f_col)

        # Update Top Subtext
        src_mode_str = f"{self.active_source_type} ({'SIMULATED' if self.active_source_type == 'SYNTHETIC' else ('MIC' if self.active_source_type == 'LAPTOP' else 'HARDWARE')})"
        self.sub_text.set_text(
            f"Active: {src_mode_str} | Speed: {rpm:,.0f} RPM | Controls: [1] PC Mic  [2] ESP32  [3] Sim  [UP/DN] Speed  [P] PPR  [Q] Quit"
        )

        return [self.tach_fill, self.tach_needle, self.tach_val_text, self.tach_detail_text,
                self.doa_needle, self.doa_text, self.line_vx, self.line_vy, self.line_vz,
                self.vib_status_text, self.fault_banner, self.sub_text]

    def run(self):
        self.anim = FuncAnimation(self.fig, self.update, interval=50, blit=False, cache_frame_data=False)
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Multimodal Diagnostic HUD (Right-Panel Focus)")
    parser.add_argument("--port", type=str, default="auto", help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--ppr", type=int, default=1, help="Pulses per revolution (default: 1)")
    parser.add_argument("--source", type=str, default="auto", choices=["auto", "esp32", "laptop", "synthetic"], help="Source mode")
    parser.add_argument("--rpm", type=float, default=0.0, help="Initial RPM for simulator mode (default: 24000.0)")
    args, _ = parser.parse_known_args()

    app = DiagnosticHUDApp(port=args.port, baud=args.baud, pulses_per_rev=args.ppr,
                           source=args.source, initial_rpm=args.rpm)
    app.run()

if __name__ == "__main__":
    main()
