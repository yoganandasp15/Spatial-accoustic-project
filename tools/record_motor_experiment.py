#!/usr/bin/env python3
"""
=============================================================================
 1-CLICK PHYSICAL MOTOR EXPERIMENT RECORDER
=============================================================================
Module: tools/record_motor_experiment.py
Target: Connects to ESP32 on COM8 at 921,600 baud (or runs synthetic physical
        testbench if unpowered) to record 10 seconds of synchronized:
          1. 400 Hz 3-Axis ADXL345 Dynamic Vibration
          2. FC-03 Optical Tachometer Shaft Pulses & Instantaneous RPM
          3. 10 kHz 4-Channel INMP441 Microphone Array PCM Audio
Author: Antigravity Autonomous Agent (Minor Project Suite)
=============================================================================
"""

import os
import sys
import time
import struct
import argparse
import json
import wave
import numpy as np
from typing import Tuple, List, Dict, Optional

# Ensure project imports resolve
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAGE4_SRC = os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src")
if STAGE4_SRC not in sys.path:
    sys.path.insert(0, STAGE4_SRC)

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def compute_checksum(data: bytes) -> int:
    chk = 0x55AA
    for i, b in enumerate(data):
        chk ^= ((b << (i % 8)) & 0xFFFF)
    return chk & 0xFFFF


class MotorExperimentRecorder:
    def __init__(
        self,
        port: str = "COM8",
        baudrate: int = 921600,
        duration: float = 10.0,
        mode: str = "healthy",
        output_dir: str = os.path.join(BASE_DIR, "data", "experiments"),
        use_synthetic: bool = False
    ):
        self.port = port
        self.baudrate = baudrate
        self.duration = duration
        self.mode = mode.lower()
        self.output_dir = output_dir
        self.use_synthetic = use_synthetic

        os.makedirs(self.output_dir, exist_ok=True)

        self.timestamps_us: List[int] = []
        self.accel_x: List[float] = []
        self.accel_y: List[float] = []
        self.accel_z: List[float] = []
        self.dyn_mag: List[float] = []
        self.tach_pulses: List[int] = []
        self.rpms: List[float] = []
        self.mic_rms: List[List[float]] = [[], [], [], []]
        self.audio_frames: List[np.ndarray] = []

        self.last_pulse_count = 0
        self.last_pulse_time = 0.0
        self.current_rpm = 0.0

    def record_hardware(self) -> bool:
        if not SERIAL_AVAILABLE:
            print("[!] pySerial is not installed. Falling back to synthetic motor testbench.")
            return False

        print(f"[*] Opening serial connection to {self.port} at {self.baudrate} baud...")
        try:
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baudrate
            ser.dtr = False
            ser.rts = False
            ser.timeout = 0.25
            ser.open()
            time.sleep(0.2)
            ser.reset_input_buffer()
            # Trigger continuous streaming mode on ESP32
            ser.write(b"S\n")
            ser.flush()
        except Exception as exc:
            print(f"[!] Unable to open {self.port}: {exc}")
            return False

        print(f"[+] Connected to {self.port}! Streaming {self.mode.upper()} motor condition for {self.duration}s...")
        print("    [Press Ctrl+C to terminate recording early]")

        HEADER_FORMAT = "<BBBBHHHBBI"
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
        rx_buf = bytearray()
        t_start = time.time()
        self.last_pulse_time = t_start
        pkt_count = 0

        # Gravity trackers for DC gravity removal
        grav_x, grav_y, grav_z = 0.0, 0.0, -0.82

        try:
            while time.time() - t_start < self.duration:
                n = ser.in_waiting
                if n > 0:
                    rx_buf.extend(ser.read(n))

                # Parse packets
                while len(rx_buf) >= 3:
                    sync_idx = -1
                    for i in range(len(rx_buf) - 1):
                        if rx_buf[i] == 0xAA and rx_buf[i+1] == 0x55:
                            sync_idx = i
                            break
                    if sync_idx == -1:
                        if len(rx_buf) > 0 and rx_buf[-1] == 0xAA:
                            del rx_buf[:-1]
                        else:
                            rx_buf.clear()
                        break
                    if sync_idx > 0:
                        del rx_buf[:sync_idx]
                    if len(rx_buf) < HEADER_SIZE:
                        break

                    pkt_type = rx_buf[2]
                    if pkt_type == 0x05:
                        s1, s2, pt, ch, seq, fs_val, blk, num_vib, res, pulses = struct.unpack_from(HEADER_FORMAT, rx_buf, 0)
                        vib_len = num_vib * 6
                        audio_len = blk * ch * 2
                        total_len = HEADER_SIZE + vib_len + audio_len + 2
                        if len(rx_buf) < total_len:
                            break

                        # Checksum
                        exp_chk = struct.unpack_from("<H", rx_buf, total_len - 2)[0]
                        chk = compute_checksum(bytes(rx_buf[:total_len - 2]))
                        if chk != exp_chk:
                            del rx_buf[:2]
                            continue

                        # Valid Type 0x05 packet
                        pkt_count += 1
                        now = time.time()
                        dt_p = now - self.last_pulse_time
                        if dt_p >= 0.5:
                            dp = pulses - self.last_pulse_count
                            if dp >= 0 and dt_p > 0:
                                self.current_rpm = (dp / dt_p) * 60.0
                            self.last_pulse_count = pulses
                            self.last_pulse_time = now

                        # Unpack vibration
                        if num_vib > 0:
                            vib_raw = np.frombuffer(
                                bytes(rx_buf[HEADER_SIZE : HEADER_SIZE + vib_len]),
                                dtype=np.int16
                            ).reshape((num_vib, 3)).astype(np.float32) / 1000.0

                            dt_vib = 1.0 / 400.0
                            t_pkt_us = int((now - t_start) * 1e6)
                            for v_idx in range(num_vib):
                                vx, vy, vz = float(vib_raw[v_idx, 0]), float(vib_raw[v_idx, 1]), float(vib_raw[v_idx, 2])
                                grav_x = 0.95 * grav_x + 0.05 * vx
                                grav_y = 0.95 * grav_y + 0.05 * vy
                                grav_z = 0.95 * grav_z + 0.05 * vz
                                dx, dy, dz = vx - grav_x, vy - grav_y, vz - grav_z
                                d_mag = float(np.sqrt(dx**2 + dy**2 + dz**2))

                                self.timestamps_us.append(t_pkt_us - int((num_vib - 1 - v_idx) * dt_vib * 1e6))
                                self.accel_x.append(vx)
                                self.accel_y.append(vy)
                                self.accel_z.append(vz)
                                self.dyn_mag.append(d_mag)
                                self.tach_pulses.append(pulses)
                                self.rpms.append(self.current_rpm)

                        # Unpack audio
                        audio_raw = np.frombuffer(
                            bytes(rx_buf[HEADER_SIZE + vib_len : HEADER_SIZE + vib_len + audio_len]),
                            dtype=np.int16
                        ).reshape((blk, ch)).astype(np.float32) / 32768.0
                        self.audio_frames.append(audio_raw)

                        for c in range(4):
                            r = float(np.sqrt(np.mean(audio_raw[:, c]**2)))
                            self.mic_rms[c].append(r)

                        del rx_buf[:total_len]

                        # Progress update
                        elapsed = now - t_start
                        pct = min(100.0, (elapsed / self.duration) * 100.0)
                        mean_dyn = np.mean(self.dyn_mag[-10:]) if self.dyn_mag else 0.0
                        sys.stdout.write(
                            f"\r[{self.mode.upper()}] {elapsed:4.1f}s / {self.duration:4.1f}s ({pct:4.1f}%) | "
                            f"Pkts: {pkt_count:4d} | RPM: {self.current_rpm:6.1f} | "
                            f"Vib: {mean_dyn:5.3f}g | Audio RMS: {self.mic_rms[0][-1]:5.4f}"
                        )
                        sys.stdout.flush()
                    else:
                        del rx_buf[:2]

                time.sleep(0.005)

        except KeyboardInterrupt:
            print("\n[*] Recording interrupted by user. Saving captured data...")
        finally:
            try:
                ser.close()
            except Exception:
                pass

        print()
        return len(self.timestamps_us) > 20

    def generate_synthetic_physical_run(self):
        """
        Generates realistic physical motor experiment data matching the exact
        governing equations from Minor_project_topic_1.pdf when hardware is unpowered.
        """
        print(f"[*] Simulating physical motor testbench in '{self.mode.upper()}' condition...")
        fs_vib = 400.0
        fs_audio = 10000.0
        t_vib = np.arange(0, self.duration, 1.0 / fs_vib)
        n_vib = len(t_vib)

        # Baseline motor speed: 1800 RPM (30 Hz fundamental shaft frequency)
        if self.mode == "healthy":
            base_rpm = 1800.0
            rpm_noise = np.random.normal(0, 4.0, n_vib)
            rpm_arr = base_rpm + rpm_noise
            # Low dynamic vibration (Zone A: Good / Smooth < 0.15g)
            f0 = base_rpm / 60.0
            ax_dyn = 0.025 * np.sin(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.012, n_vib)
            ay_dyn = 0.020 * np.cos(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.012, n_vib)
            az_dyn = 0.015 * np.sin(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.012, n_vib)
            # Normal acoustic emissions
            audio_amp = 0.008

        elif self.mode == "defective":
            # Defective motor with Mass Unbalance: Strong 1X vibration component (PDF Page 3)
            base_rpm = 1765.0
            rpm_noise = np.random.normal(0, 9.0, n_vib)
            rpm_arr = base_rpm + rpm_noise
            f0 = base_rpm / 60.0
            # Strong 1X shaft unbalance: 0.45g dynamic vibration (Zone C / D Warning)
            ax_dyn = 0.42 * np.sin(2 * np.pi * f0 * t_vib) + 0.08 * np.sin(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.025, n_vib)
            ay_dyn = 0.38 * np.cos(2 * np.pi * f0 * t_vib) + 0.07 * np.cos(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.025, n_vib)
            az_dyn = 0.18 * np.sin(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.025, n_vib)
            # Elevated acoustic hum
            audio_amp = 0.032

        elif self.mode == "disturbance":
            # Crucial PDF Page 13 Test: Healthy motor running + Loud external acoustic disturbance
            base_rpm = 1800.0
            rpm_arr = base_rpm + np.random.normal(0, 4.0, n_vib)
            f0 = base_rpm / 60.0
            # Vibration remains completely NORMAL in Zone A (< 0.15g)
            ax_dyn = 0.025 * np.sin(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.012, n_vib)
            ay_dyn = 0.020 * np.cos(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.012, n_vib)
            az_dyn = 0.015 * np.sin(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.012, n_vib)
            # Loud acoustic bursts (clapping, speech, external noise)
            audio_amp = 0.085
        else:
            base_rpm = 1800.0
            rpm_arr = np.full(n_vib, base_rpm)
            ax_dyn = np.random.normal(0, 0.02, n_vib)
            ay_dyn = np.random.normal(0, 0.02, n_vib)
            az_dyn = np.random.normal(0, 0.02, n_vib)
            audio_amp = 0.010

        grav_z = -0.82
        pulses = (t_vib * (base_rpm / 60.0)).astype(int)

        self.timestamps_us = (t_vib * 1e6).astype(int).tolist()
        self.accel_x = ax_dyn.tolist()
        self.accel_y = ay_dyn.tolist()
        self.accel_z = (az_dyn + grav_z).tolist()
        self.dyn_mag = np.sqrt(ax_dyn**2 + ay_dyn**2 + az_dyn**2).tolist()
        self.tach_pulses = pulses.tolist()
        self.rpms = rpm_arr.tolist()

        # Generate 4-channel audio
        n_audio = int(self.duration * fs_audio)
        t_aud = np.arange(n_audio) / fs_audio
        d_pitch = 0.02032
        source_angle_deg = 20.0 if self.mode != "disturbance" else -45.0
        source_rad = np.deg2rad(source_angle_deg)
        c_sound = 343.0

        audio_mat = np.zeros((n_audio, 4), dtype=np.float32)
        for m in range(4):
            dx = (m - 1.5) * d_pitch
            tau = (dx * np.sin(source_rad)) / c_sound
            t_m = t_aud + tau
            sig = audio_amp * (
                0.70 * np.sin(2 * np.pi * 1250.0 * t_m)
                + 0.30 * np.sin(2 * np.pi * 2500.0 * t_m)
                + 0.05 * np.random.randn(n_audio)
            )
            audio_mat[:, m] = np.clip(sig, -0.95, 0.95)

        self.audio_frames = [audio_mat]
        for c in range(4):
            self.mic_rms[c] = [float(np.sqrt(np.mean(audio_mat[:, c]**2)))]

    def save_artifacts(self) -> Dict[str, str]:
        prefix = f"motor_{self.mode}_run"
        csv_path = os.path.join(self.output_dir, f"{prefix}.csv")
        wav_path = os.path.join(self.output_dir, f"{prefix}_audio.wav")
        meta_path = os.path.join(self.output_dir, f"{prefix}_meta.json")

        # 1. Save CSV
        print(f"[*] Writing {len(self.timestamps_us)} physical records to {os.path.basename(csv_path)}...")
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("Timestamp_us,Accel_X_g,Accel_Y_g,Accel_Z_g,Dyn_Mag_g,Tach_Pulses,Shaft_RPM\n")
            for i in range(len(self.timestamps_us)):
                f.write(
                    f"{self.timestamps_us[i]},{self.accel_x[i]:.4f},{self.accel_y[i]:.4f},"
                    f"{self.accel_z[i]:.4f},{self.dyn_mag[i]:.4f},{self.tach_pulses[i]},"
                    f"{self.rpms[i]:.2f}\n"
                )

        # 2. Save 4-Channel WAV Audio
        if self.audio_frames:
            full_audio = np.vstack(self.audio_frames)
            audio_int16 = (np.clip(full_audio, -1.0, 1.0) * 32767.0).astype(np.int16)
            with wave.open(wav_path, "wb") as wf:
                wf.setnchannels(4)
                wf.setsampwidth(2)
                wf.setframerate(10000)
                wf.writeframes(audio_int16.tobytes())
            print(f"[+] Saved 4-channel 10kHz audio to {os.path.basename(wav_path)}")

        # 3. Compute Summary Metrics
        mean_rpm = float(np.mean(self.rpms)) if self.rpms else 0.0
        rpm_jitter = float(np.std(self.rpms)) if self.rpms else 0.0
        mean_dyn = float(np.mean(self.dyn_mag)) if self.dyn_mag else 0.0
        peak_dyn = float(np.max(self.dyn_mag)) if self.dyn_mag else 0.0

        # FFT Order Extraction
        fs_vib = 400.0
        n_samples = len(self.dyn_mag)
        if n_samples >= 64:
            dyn_ac = np.array(self.dyn_mag) - mean_dyn
            fft_mag = np.abs(np.fft.rfft(dyn_ac))
            freqs = np.fft.rfftfreq(n_samples, d=1.0 / fs_vib)
            f0 = max(1.0, mean_rpm / 60.0)

            def get_order_amp(f_target):
                mask = (freqs >= max(0.5, f_target - 1.5)) & (freqs <= f_target + 1.5)
                return float(np.max(fft_mag[mask])) if np.any(mask) else 0.0

            amp_1x = get_order_amp(f0)
            amp_2x = get_order_amp(2.0 * f0)
            amp_3x = get_order_amp(3.0 * f0)
            tot_energy = float(np.sum(fft_mag)) + 1e-12
            e_1x_pct = (amp_1x / tot_energy) * 100.0
            e_2x_pct = (amp_2x / tot_energy) * 100.0
        else:
            e_1x_pct, e_2x_pct = 0.0, 0.0

        meta = {
            "mode": self.mode,
            "data_provenance": "SYNTHETIC_SIMULATION (EXPLICIT USER FLAG)" if self.use_synthetic else "PHYSICAL_HARDWARE (COM8 ESP32 Stream)",
            "duration_s": self.duration,
            "total_samples": len(self.timestamps_us),
            "sample_rate_vib_hz": 400.0,
            "sample_rate_audio_hz": 10000.0,
            "mean_rpm": round(mean_rpm, 1),
            "rpm_jitter": round(rpm_jitter, 2),
            "mean_dyn_vib_g": round(mean_dyn, 4),
            "peak_dyn_vib_g": round(peak_dyn, 4),
            "iso_zone": "Zone A: Good / Smooth" if mean_dyn < 0.15 else ("Zone B: Permissible" if mean_dyn < 0.35 else "Zone C: Warning"),
            "order_1x_energy_pct": round(e_1x_pct, 1),
            "order_2x_energy_pct": round(e_2x_pct, 1),
            "csv_path": csv_path,
            "wav_path": wav_path
        }

        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        print(f"[+] Saved metadata summary to {os.path.basename(meta_path)}")

        # Print Pretty Table
        print("\n" + "=" * 70)
        print(f" EXPERIMENT COMPLETE: {self.mode.upper()} MOTOR RUN")
        print("=" * 70)
        print(f" Condition            : {self.mode.upper()}")
        print(f" Shaft Speed          : {mean_rpm:6.1f} RPM (Jitter: ±{rpm_jitter:3.1f} RPM)")
        print(f" Dynamic Vibration    : {mean_dyn:6.4f}g (Peak: {peak_dyn:6.4f}g)")
        print(f" ISO Vibration Rating : {meta['iso_zone']}")
        print(f" 1x Order Energy      : {e_1x_pct:5.1f}% (Mass Unbalance Indicator)")
        print(f" 2x Order Energy      : {e_2x_pct:5.1f}% (Misalignment Indicator)")
        print(f" Output CSV File      : {os.path.relpath(csv_path, BASE_DIR)}")
        print("=" * 70 + "\n")

        return {"csv": csv_path, "wav": wav_path, "meta": meta_path}

    def run(self):
        if not self.use_synthetic:
            success = self.record_hardware()
            if not success:
                print("\n[-] ERROR: Physical hardware recording failed. No packets received on COM8.")
                print("    Please ensure:")
                print("      1. ESP32 is plugged into COM8 at 921600 baud.")
                print("      2. Radar GUI (RUN_LIVE_RADAR.bat) is closed to release serial port.")
                print("      3. DC Motor is running and sensor breadboard is powered.")
                print("    Physical motor run aborted without generating or writing fake data.")
                return False
        else:
            print("\n[!] NOTE: Explicit --synthetic flag provided. Generating simulated run...")
            self.generate_synthetic_physical_run()

        return self.save_artifacts()


def main():
    parser = argparse.ArgumentParser(description="Record Physical Motor Experiment Suite")
    parser.add_argument("--mode", choices=["healthy", "defective", "disturbance"], default="healthy",
                        help="Motor condition: healthy, defective (imbalance/misalignment), or disturbance (external acoustic noise)")
    parser.add_argument("--duration", type=float, default=10.0, help="Recording duration in seconds (default: 10.0)")
    parser.add_argument("--port", type=str, default="COM8", help="Serial port (default: COM8)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baudrate (default: 921600)")
    parser.add_argument("--synthetic", action="store_true", help="Force synthetic physical data generation")
    args = parser.parse_args()

    recorder = MotorExperimentRecorder(
        port=args.port,
        baudrate=args.baud,
        duration=args.duration,
        mode=args.mode,
        use_synthetic=args.synthetic
    )
    recorder.run()


if __name__ == "__main__":
    main()
