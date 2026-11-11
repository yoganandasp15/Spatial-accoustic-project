#!/usr/bin/env python3
"""
=============================================================================
 PHYSICAL MOTOR EXPERIMENT RECORDER & DATASET INGESTION PIPELINE
=============================================================================
Module: tools/record_motor_experiment.py
Description: Connects to ESP32 on COM8 at 921,600 baud using the validated
             Type 0x05 binary protocol to record synchronized:
               1. 400 Hz 3-Axis ADXL345 Acceleration (+/-2g Full Resolution)
                  Signed X, Y, Z + Dynamic Acceleration (Gravity Decoupled)
               2. FC-03 Optical Tachometer Pulses & Shaft RPM (GPIO 4 Interrupt)
               3. 10,000 Hz 4-Channel INMP441 Microphone Array PCM Audio
               4. Monotonic host arrival timestamps & sequence integrity logs

Timing Architecture:
  - Host arrival timestamps: time.monotonic() elapsed offset from stream start.
  - Recording start/end metadata: ISO 8601 UTC/local wall-clock timestamps.
  - Accelerometer sample times: Estimated via nominal 400Hz FIFO spacing
    relative to packet receive time. (Hardware per-sample clocks are absent).
  - Packet-level cross-modal sync is bounded by ~12.8ms Type 0x05 packet duration.

Dataset Structure:
  data/dataset/<run_id>/
    ├── metadata.json       (Provenance, configuration, RPM, vibration metrics, status)
    ├── vibration.csv       (Signed X, Y, Z acceleration + dynamic vibration)
    ├── packet_log.csv      (Packet seq, monotonic arrival time, sample indices, checksum)
    └── audio_4ch.wav       (4-channel 16-bit 10kHz PCM audio)
  data/dataset/manifest.json (Global registry of all recorded runs)

Author: Antigravity Autonomous Agent (TinyML Motor Diagnostics Suite)
=============================================================================
"""

import os
import sys
import time
import uuid
import struct
import argparse
import json
import wave
from datetime import datetime, timezone
from typing import Tuple, List, Dict, Optional, Any
import numpy as np

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


def find_esp32_port(requested_port: str = "auto") -> str:
    """
    Intelligently detects the active ESP32 COM port.
    If a specific non-default port (e.g. COM_NONEXISTENT_99) is passed, respects it so unit tests fail cleanly.
    Otherwise scans for CP210x, CH340, FTDI, or ESP32 USB VID/PID hardware descriptors,
    filtering out virtual Bluetooth and ACPI motherboard serial ports.
    """
    if not SERIAL_AVAILABLE:
        return requested_port if requested_port != "auto" else "COM8"

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return requested_port if requested_port != "auto" else "COM8"

    # If an explicit custom port was passed (e.g. for testing)
    if requested_port != "auto":
        for p in ports:
            if p.device.upper() == requested_port.upper():
                return p.device
        # If requested_port was explicitly non-COM8 (e.g. COM_NONEXISTENT_99), return it so caller fails cleanly
        if requested_port.upper() != "COM8":
            return requested_port

    # Scan for CP210x (10c4:ea60), CH340 (1a86:7523), FTDI (0403:6001), or Espressif ESP32 (303a:)
    for p in ports:
        h = ((p.hwid or "") + " " + (p.description or "")).lower()
        if "bthenum" in h or "bluetooth" in h or "acpi" in h or "pnp0501" in h:
            continue
        if any(k in h for k in ["10c4:ea60", "cp210", "1a86:7523", "ch340", "0403:6001", "ftdi", "303a:", "esp32"]):
            return p.device

    # Match any genuine USB serial device
    for p in ports:
        h = ((p.hwid or "") + " " + (p.description or "")).lower()
        if "bthenum" in h or "bluetooth" in h or "acpi" in h or "pnp0501" in h:
            continue
        if "usb" in h:
            return p.device

    # Fallback to COM8 if present, otherwise default COM8
    for p in ports:
        if p.device.upper() == "COM8":
            return p.device

    return "COM8"


def compute_checksum(data: bytes) -> int:
    """16-bit XOR rolling checksum matching ESP32 firmware."""
    chk = 0x55AA
    for i, b in enumerate(data):
        chk ^= ((b << (i % 8)) & 0xFFFF)
    return chk & 0xFFFF


class MotorExperimentRecorder:
    def __init__(
        self,
        motor_id: str = "motor_A",
        health_label: str = "healthy",
        environment: str = "quiet",
        fault_type: str = "none",
        disturbance_desc: str = "",
        duration: float = 30.0,
        port: str = "auto",
        baudrate: int = 921600,
        supply_voltage: Optional[float] = None,
        pulses_per_rev: int = 1,
        session_id: Optional[str] = None,
        output_dir: Optional[str] = None,
        notes: str = "",
        use_synthetic: bool = False,
        stall_timeout_s: float = 2.0
    ):
        self.motor_id = motor_id.strip()
        self.health_label = health_label.lower().strip()
        self.environment = environment.lower().strip()
        self.fault_type = fault_type.lower().strip()
        self.disturbance_desc = disturbance_desc.strip()
        self.duration = float(duration)
        self.requested_port = port
        self.port = find_esp32_port(port)
        self.baudrate = baudrate
        self.supply_voltage = supply_voltage
        self.pulses_per_rev = max(1, int(pulses_per_rev))
        self.notes = notes
        self.use_synthetic = use_synthetic
        self.stall_timeout_s = float(stall_timeout_s)

        # Firmware hardware configuration constants
        self.accel_range_g = 2.0  # Firmware DATA_FORMAT = 0x08 (+/-2g Full Res, 3.9 mg/LSB)

        # Generate collision-resistant unique run ID with microsecond timestamp + short UUID
        now_dt = datetime.now()
        time_micro_str = now_dt.strftime("%Y%m%d_%H%M%S_%f")
        short_uuid = uuid.uuid4().hex[:6]

        self.session_id = session_id.strip() if session_id else "session_pilot_01"
        self.run_id = f"run_{time_micro_str}_{short_uuid}_{self.motor_id}_{self.health_label}_{self.environment}"

        self.base_output_dir = output_dir if output_dir else os.path.join(BASE_DIR, "data", "dataset")
        self.run_dir = os.path.join(self.base_output_dir, self.run_id)
        # NOTE: Do NOT create directory here. Directory is created exclusively on successful save.

        # Recording status tracking: COMPLETE, INTERRUPTED, INCOMPLETE, FAILED
        self.recording_status = "INITIALIZING"

        # Buffers for recording
        self.packet_records: List[Dict[str, Any]] = []
        self.vibration_records: List[Dict[str, Any]] = []
        self.audio_frames: List[np.ndarray] = []

        # Tracking state
        self.total_packets = 0
        self.dropped_packets = 0
        self.checksum_errors = 0
        self.last_seq: Optional[int] = None
        self.tach_baseline_established = False
        self.last_pulse_count = 0
        self.last_pulse_time_mono = 0.0
        self.current_rpm: Optional[float] = None
        self.rpm_history: List[float] = []
        self.all_tach_pulses: List[int] = []

        # Monotonic and wall-clock times
        self.start_mono = 0.0
        self.end_mono = 0.0
        self.start_iso = ""
        self.end_iso = ""
        self.actual_duration_s = 0.0

    def record_hardware(self) -> bool:
        """Acquires physical streaming telemetry from ESP32 on COM8."""
        if not SERIAL_AVAILABLE:
            print("[ERROR] pySerial is not installed. Run: pip install pyserial", file=sys.stderr)
            self.recording_status = "FAILED"
            return False

        print(f"[*] Connecting to ESP32 on {self.port} at {self.baudrate} baud...")
        try:
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baudrate
            ser.dtr = False
            ser.rts = False
            ser.timeout = 0.25
            ser.open()
            time.sleep(0.15)
            ser.reset_input_buffer()
            # Send 'S' command to trigger streaming mode if previously in ASCII test mode
            ser.write(b"S\n")
            ser.flush()
        except Exception as exc:
            print(f"[-] ERROR: Failed to open serial port {self.port}: {exc}", file=sys.stderr)
            print("    Check: Is the ESP32 plugged in? Is another app (GUI/serial monitor) locking the port?", file=sys.stderr)
            self.recording_status = "FAILED"
            return False

        print(f"[+] Connected to {self.port}!")
        print(f"[*] Run ID       : {self.run_id}")
        print(f"[*] Session ID   : {self.session_id}")
        print(f"[*] Target State : Motor={self.motor_id} | Health={self.health_label.upper()} | Env={self.environment.upper()}")
        print(f"[*] Target Time  : {self.duration:.1f} seconds (Press Ctrl+C to stop cleanly)\n")

        HEADER_FORMAT = "<BBBBHHHBBI"
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
        rx_buf = bytearray()

        self.start_mono = time.monotonic()
        self.last_pulse_time_mono = self.start_mono
        last_packet_time_mono = self.start_mono
        self.start_iso = datetime.now(timezone.utc).astimezone().isoformat()

        # Detrending state for dynamic vibration
        grav_x, grav_y, grav_z = 0.0, 0.0, -0.82
        total_audio_samples = 0
        total_vib_samples = 0

        first_packet_received = False
        watchdog_timeout_s = 3.0
        stream_stalled = False

        try:
            while (time.monotonic() - self.start_mono) < self.duration:
                now_mono = time.monotonic()

                # Stall detection: If packets started arriving, but stop for > stall_timeout_s
                if first_packet_received and (now_mono - last_packet_time_mono) > self.stall_timeout_s:
                    print(f"\n[-] ERROR: Serial stream stalled! No packets received for {now_mono - last_packet_time_mono:.1f}s.", file=sys.stderr)
                    stream_stalled = True
                    self.recording_status = "INTERRUPTED"
                    break

                n = ser.in_waiting
                if n > 0:
                    rx_buf.extend(ser.read(n))

                # Scan for sync bytes 0xAA, 0x55
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

                        # Validate Type 0x05 Packet Contract invariants
                        if pt != 0x05 or ch != 4 or fs_val != 10000 or blk != 128 or num_vib > 8:
                            self.checksum_errors += 1
                            del rx_buf[:2]
                            continue

                        vib_len = num_vib * 6
                        audio_len = blk * ch * 2
                        total_len = HEADER_SIZE + vib_len + audio_len + 2
                        if len(rx_buf) < total_len:
                            break  # Await complete packet

                        # Checksum verification
                        exp_chk = struct.unpack_from("<H", rx_buf, total_len - 2)[0]
                        chk = compute_checksum(bytes(rx_buf[:total_len - 2]))
                        if chk != exp_chk:
                            self.checksum_errors += 1
                            # Log corrupt packet in packet records
                            self.packet_records.append({
                                "pkt_seq": seq,
                                "host_recv_time_s": round(now_mono - self.start_mono, 6),
                                "num_vib": num_vib,
                                "block_frames": blk,
                                "audio_sample_idx": total_audio_samples,
                                "vib_sample_idx": total_vib_samples,
                                "tach_pulses": pulses,
                                "checksum_valid": 0,
                                "seq_gap": 0
                            })
                            del rx_buf[:2]
                            continue

                        # Valid Type 0x05 packet received
                        first_packet_received = True
                        last_packet_time_mono = now_mono
                        self.total_packets += 1

                        seq_gap = 0
                        if self.last_seq is not None:
                            expected_seq = (self.last_seq + 1) & 0xFFFF
                            if seq != expected_seq:
                                seq_gap = (seq - expected_seq) & 0xFFFF
                                if seq_gap < 500:
                                    self.dropped_packets += seq_gap
                        self.last_seq = seq

                        # Tachometer & RPM calculation using monotonic interval and baseline alignment
                        if not self.tach_baseline_established:
                            self.last_pulse_count = pulses
                            self.last_pulse_time_mono = now_mono
                            self.tach_baseline_established = True
                            # Baseline established from first packet: do not compute initial RPM spike
                        else:
                            dt_p = now_mono - self.last_pulse_time_mono
                            if dt_p >= 0.5:
                                dp = (pulses - self.last_pulse_count) & 0xFFFFFFFF
                                if dp > 0 and dp < 50000 and dt_p > 0:
                                    self.current_rpm = ((dp / self.pulses_per_rev) / dt_p) * 60.0
                                    self.rpm_history.append(self.current_rpm)
                                elif dp == 0:
                                    self.current_rpm = 0.0
                                    self.rpm_history.append(0.0)
                                self.last_pulse_count = pulses
                                self.last_pulse_time_mono = now_mono

                        self.all_tach_pulses.append(pulses)

                        # Log valid packet record
                        t_pkt = now_mono - self.start_mono
                        self.packet_records.append({
                            "pkt_seq": seq,
                            "host_recv_time_s": round(t_pkt, 6),
                            "num_vib": num_vib,
                            "block_frames": blk,
                            "audio_sample_idx": total_audio_samples,
                            "vib_sample_idx": total_vib_samples,
                            "tach_pulses": pulses,
                            "checksum_valid": 1,
                            "seq_gap": seq_gap
                        })

                        # Parse vibration payload (int16 signed mg / 1000.0 -> g)
                        if num_vib > 0:
                            vib_raw = np.frombuffer(
                                bytes(rx_buf[HEADER_SIZE : HEADER_SIZE + vib_len]),
                                dtype=np.int16
                            ).reshape((num_vib, 3))

                            dt_vib = 1.0 / 400.0
                            t_v_start = t_pkt - (num_vib - 1) * dt_vib

                            for v_idx in range(num_vib):
                                raw_x = int(vib_raw[v_idx, 0])
                                raw_y = int(vib_raw[v_idx, 1])
                                raw_z = int(vib_raw[v_idx, 2])
                                vx = raw_x / 1000.0
                                vy = raw_y / 1000.0
                                vz = raw_z / 1000.0

                                # Dynamic gravity tracking
                                grav_x = 0.95 * grav_x + 0.05 * vx
                                grav_y = 0.95 * grav_y + 0.05 * vy
                                grav_z = 0.95 * grav_z + 0.05 * vz
                                dx = vx - grav_x
                                dy = vy - grav_y
                                dz = vz - grav_z
                                dyn_mag = float(np.sqrt(dx**2 + dy**2 + dz**2))

                                smp_time = max(0.0, t_v_start + v_idx * dt_vib)
                                self.vibration_records.append({
                                    "sample_idx": total_vib_samples + v_idx,
                                    "host_time_s": round(smp_time, 6),
                                    "accel_x_g": round(vx, 4),
                                    "accel_y_g": round(vy, 4),
                                    "accel_z_g": round(vz, 4),
                                    "dyn_x_g": round(dx, 4),
                                    "dyn_y_g": round(dy, 4),
                                    "dyn_z_g": round(dz, 4),
                                    "dyn_mag_g": round(dyn_mag, 4),
                                    "tach_pulses": pulses,
                                    "instant_rpm": round(self.current_rpm if self.current_rpm is not None else 0.0, 1)
                                })
                            total_vib_samples += num_vib

                        # Parse audio payload (16-bit PCM normalized float32)
                        audio_raw = np.frombuffer(
                            bytes(rx_buf[HEADER_SIZE + vib_len : HEADER_SIZE + vib_len + audio_len]),
                            dtype=np.int16
                        ).reshape((blk, ch)).astype(np.float32) / 32768.0
                        self.audio_frames.append(audio_raw)
                        total_audio_samples += blk

                        del rx_buf[:total_len]

                        # Progress ticker
                        elapsed = now_mono - self.start_mono
                        pct = min(100.0, (elapsed / self.duration) * 100.0)
                        cur_rpm_str = f"{self.current_rpm:.0f}" if self.current_rpm is not None else "WAIT"
                        mean_dyn = np.mean([r["dyn_mag_g"] for r in self.vibration_records[-20:]]) if self.vibration_records else 0.0
                        sys.stdout.write(
                            f"\r[{self.health_label.upper()}|{self.environment.upper()}] "
                            f"{elapsed:4.1f}s/{self.duration:4.1f}s ({pct:4.1f}%) | "
                            f"Pkts:{self.total_packets:4d} | RPM:{cur_rpm_str:>5} (Pulses:{pulses}) | "
                            f"DynVib:{mean_dyn:5.3f}g | Drops:{self.dropped_packets} | ChkErr:{self.checksum_errors}"
                        )
                        sys.stdout.flush()
                    else:
                        del rx_buf[:2]

                # Watchdog for first valid packet arrival
                if not first_packet_received and (now_mono - self.start_mono) > watchdog_timeout_s:
                    print(f"\n[-] ERROR: Watchdog timeout ({watchdog_timeout_s:.1f}s) with 0 valid Type 0x05 packets received.", file=sys.stderr)
                    print("    Physical acquisition aborted. Check ESP32 power, firmware & baudrate.", file=sys.stderr)
                    self.recording_status = "FAILED"
                    return False

                time.sleep(0.003)

        except KeyboardInterrupt:
            print("\n[*] Recording interrupted by user. Finalizing captured buffers...")
            self.recording_status = "INTERRUPTED"
        finally:
            try:
                ser.close()
            except Exception:
                pass

        self.end_mono = time.monotonic()
        self.end_iso = datetime.now(timezone.utc).astimezone().isoformat()
        self.actual_duration_s = round(self.end_mono - self.start_mono, 3)
        print()

        # Evaluate final status
        expected_vib_min = int(self.duration * 400.0 * 0.90)
        expected_aud_min = int(self.duration * 10000.0 * 0.90)

        if not first_packet_received:
            self.recording_status = "FAILED"
            return False

        if stream_stalled or self.recording_status == "INTERRUPTED":
            self.recording_status = "INTERRUPTED"
        elif total_vib_samples >= expected_vib_min and total_audio_samples >= expected_aud_min and self.actual_duration_s >= (self.duration * 0.90):
            self.recording_status = "COMPLETE"
        else:
            self.recording_status = "INCOMPLETE"

        # Return True if enough data was captured to write diagnostic artifacts
        return len(self.vibration_records) > 50 and len(self.audio_frames) > 5

    def generate_synthetic_testbench_run(self):
        """
        Explicit offline testbench simulation for software regression verification.
        Provenance is strictly labeled as SYNTHETIC_TESTBENCH.
        """
        print(f"[*] Generating SYNTHETIC TESTBENCH run ({self.health_label.upper()} | {self.environment.upper()}) for {self.duration}s...")
        self.start_mono = time.monotonic()
        self.start_iso = datetime.now(timezone.utc).astimezone().isoformat()

        fs_vib = 400.0
        fs_audio = 10000.0
        t_vib = np.arange(0, self.duration, 1.0 / fs_vib)
        n_vib = len(t_vib)

        # Base speed
        base_rpm = 1800.0 if self.health_label == "healthy" else 1760.0
        f0 = base_rpm / 60.0

        if self.health_label == "healthy":
            ax_dyn = 0.025 * np.sin(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.010, n_vib)
            ay_dyn = 0.020 * np.cos(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.010, n_vib)
            az_dyn = 0.015 * np.sin(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.010, n_vib)
            audio_base_amp = 0.010
        else:
            # Defective motor: elevated vibration within +/-2g range
            ax_dyn = 0.380 * np.sin(2 * np.pi * f0 * t_vib) + 0.08 * np.sin(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.020, n_vib)
            ay_dyn = 0.350 * np.cos(2 * np.pi * f0 * t_vib) + 0.07 * np.cos(4 * np.pi * f0 * t_vib) + np.random.normal(0, 0.020, n_vib)
            az_dyn = 0.160 * np.sin(2 * np.pi * f0 * t_vib) + np.random.normal(0, 0.020, n_vib)
            audio_base_amp = 0.035

        if self.environment == "disturbance":
            audio_base_amp = 0.085  # External acoustic noise injection

        grav_z = -0.82
        pulses = (t_vib * (base_rpm / 60.0) * self.pulses_per_rev).astype(int)

        for i in range(n_vib):
            vx = float(ax_dyn[i])
            vy = float(ay_dyn[i])
            vz = float(az_dyn[i] + grav_z)
            dyn_mag = float(np.sqrt(ax_dyn[i]**2 + ay_dyn[i]**2 + az_dyn[i]**2))
            cur_rpm = base_rpm + float(np.random.normal(0, 2.0))

            self.vibration_records.append({
                "sample_idx": i,
                "host_time_s": round(float(t_vib[i]), 6),
                "accel_x_g": round(vx, 4),
                "accel_y_g": round(vy, 4),
                "accel_z_g": round(vz, 4),
                "dyn_x_g": round(float(ax_dyn[i]), 4),
                "dyn_y_g": round(float(ay_dyn[i]), 4),
                "dyn_z_g": round(float(az_dyn[i]), 4),
                "dyn_mag_g": round(dyn_mag, 4),
                "tach_pulses": int(pulses[i]),
                "instant_rpm": round(cur_rpm, 1)
            })
            self.rpm_history.append(cur_rpm)
            self.all_tach_pulses.append(int(pulses[i]))

        # Synthesize 4-channel audio
        n_audio = int(self.duration * fs_audio)
        t_aud = np.arange(n_audio) / fs_audio
        audio_mat = np.zeros((n_audio, 4), dtype=np.float32)
        d_pitch = 0.02032
        source_rad = np.deg2rad(15.0 if self.environment == "quiet" else -45.0)

        for m in range(4):
            tau = ((m - 1.5) * d_pitch * np.sin(source_rad)) / 343.0
            sig = audio_base_amp * (
                0.70 * np.sin(2 * np.pi * 1200.0 * (t_aud + tau))
                + 0.30 * np.sin(2 * np.pi * 2400.0 * (t_aud + tau))
                + 0.05 * np.random.randn(n_audio)
            )
            audio_mat[:, m] = np.clip(sig, -0.95, 0.95)

        self.audio_frames = [audio_mat]
        self.total_packets = int(self.duration * 78.1)

        # Synthesize packet log with sequence numbers
        n_pkts = max(1, self.total_packets)
        cum_aud = 0
        cum_vib = 0
        for p in range(n_pkts):
            p_time = p * (self.duration / n_pkts)
            cum_aud += 128
            cum_vib += 5
            self.packet_records.append({
                "pkt_seq": p,
                "host_recv_time_s": round(p_time, 6),
                "num_vib": 5,
                "block_frames": 128,
                "audio_sample_idx": cum_aud,
                "vib_sample_idx": cum_vib,
                "tach_pulses": int(p * (base_rpm / 60.0) * (self.duration / n_pkts) * self.pulses_per_rev),
                "checksum_valid": 1,
                "seq_gap": 0
            })

        self.end_mono = time.monotonic()
        self.end_iso = datetime.now(timezone.utc).astimezone().isoformat()
        self.actual_duration_s = round(self.duration, 3)
        self.recording_status = "COMPLETE"

    def save_dataset_artifacts(self) -> Dict[str, str]:
        """Saves vibration.csv, audio_4ch.wav, packet_log.csv, and metadata.json to run_dir."""
        # Create run_dir exclusively. Never use exist_ok=True
        os.makedirs(self.run_dir, exist_ok=False)

        vibration_csv_path = os.path.join(self.run_dir, "vibration.csv")
        audio_wav_path = os.path.join(self.run_dir, "audio_4ch.wav")
        packet_log_path = os.path.join(self.run_dir, "packet_log.csv")
        metadata_json_path = os.path.join(self.run_dir, "metadata.json")

        # 1. Write vibration.csv
        print(f"[*] Writing {len(self.vibration_records)} vibration records to {os.path.basename(vibration_csv_path)}...")
        with open(vibration_csv_path, "w", encoding="utf-8") as f:
            f.write("sample_idx,host_time_s,accel_x_g,accel_y_g,accel_z_g,dyn_x_g,dyn_y_g,dyn_z_g,dyn_mag_g,tach_pulses,instant_rpm\n")
            for r in self.vibration_records:
                f.write(
                    f"{r['sample_idx']},{r['host_time_s']:.6f},"
                    f"{r['accel_x_g']:.4f},{r['accel_y_g']:.4f},{r['accel_z_g']:.4f},"
                    f"{r['dyn_x_g']:.4f},{r['dyn_y_g']:.4f},{r['dyn_z_g']:.4f},{r['dyn_mag_g']:.4f},"
                    f"{r['tach_pulses']},{r['instant_rpm']:.1f}\n"
                )

        # 2. Write audio_4ch.wav
        full_audio = None
        if self.audio_frames:
            full_audio = np.vstack(self.audio_frames)
            audio_int16 = (np.clip(full_audio, -1.0, 1.0) * 32767.0).astype(np.int16)
            with wave.open(audio_wav_path, "wb") as wf:
                wf.setnchannels(4)
                wf.setsampwidth(2)
                wf.setframerate(10000)
                wf.writeframes(audio_int16.tobytes())
            print(f"[+] Saved 4-channel 10kHz audio to {os.path.basename(audio_wav_path)}")

        # 3. Write packet_log.csv
        print(f"[*] Writing packet sequence log to {os.path.basename(packet_log_path)}...")
        with open(packet_log_path, "w", encoding="utf-8") as f:
            f.write("pkt_seq,host_recv_time_s,num_vib,block_frames,audio_sample_idx,vib_sample_idx,tach_pulses,checksum_valid,seq_gap\n")
            for p in self.packet_records:
                f.write(
                    f"{p['pkt_seq']},{p['host_recv_time_s']:.6f},"
                    f"{p['num_vib']},{p['block_frames']},"
                    f"{p['audio_sample_idx']},{p['vib_sample_idx']},"
                    f"{p['tach_pulses']},{p['checksum_valid']},{p['seq_gap']}\n"
                )

        # 4. Compute Summary Metrics for metadata
        valid_rpms = [r for r in self.rpm_history if r > 30.0]
        has_valid_pulses = len(self.all_tach_pulses) > 0 and (max(self.all_tach_pulses) - min(self.all_tach_pulses)) >= 2
        rpm_valid = bool(has_valid_pulses and len(valid_rpms) > 0)
        mean_rpm = round(float(np.mean(valid_rpms)), 1) if rpm_valid else None
        rpm_jitter = round(float(np.std(valid_rpms)), 2) if rpm_valid else None

        dyn_mags = [r["dyn_mag_g"] for r in self.vibration_records]
        mean_dyn = round(float(np.mean(dyn_mags)), 4) if dyn_mags else 0.0
        peak_dyn = round(float(np.max(dyn_mags)), 4) if dyn_mags else 0.0

        # Rotational order metrics (calculated on signed axes if RPM valid)
        e1_pct = 0.0
        e2_pct = 0.0
        if rpm_valid and mean_rpm and len(self.vibration_records) >= 64:
            f0 = mean_rpm / 60.0
            fs_v = 400.0
            n_fft = max(256, len(dyn_mags))
            freqs = np.fft.rfftfreq(n_fft, d=1.0 / fs_v)

            # Sum power spectra of signed axes (orientation invariant)
            power_spec = np.zeros_like(freqs, dtype=np.float64)
            for axis_k in ("dyn_x_g", "dyn_y_g", "dyn_z_g"):
                ax_arr = np.array([r[axis_k] for r in self.vibration_records], dtype=np.float64)
                power_spec += np.abs(np.fft.rfft(ax_arr, n=n_fft)) ** 2

            def get_order_energy(f_target, bw=1.5):
                mask = (freqs >= max(0.5, f_target - bw)) & (freqs <= f_target + bw)
                return float(np.sum(power_spec[mask])) if np.any(mask) else 0.0

            tot_power = float(np.sum(power_spec)) + 1e-12
            e1_pct = round((get_order_energy(f0) / tot_power) * 100.0, 1)
            e2_pct = round((get_order_energy(2.0 * f0) / tot_power) * 100.0, 1)

        provenance = "SYNTHETIC_TESTBENCH" if self.use_synthetic else "PHYSICAL_HARDWARE (COM8 ESP32 Stream)"

        metadata = {
            "run_id": self.run_id,
            "session_id": self.session_id,
            "motor_id": self.motor_id,
            "motor_health_label": self.health_label,
            "acoustic_environment": self.environment,
            "fault_type": self.fault_type,
            "disturbance_description": self.disturbance_desc,
            "recording_status": self.recording_status,
            "supply_voltage_v": self.supply_voltage,
            "pulses_per_rev": self.pulses_per_rev,
            "target_duration_s": self.duration,
            "actual_duration_s": self.actual_duration_s,
            "start_timestamp_iso": self.start_iso,
            "end_timestamp_iso": self.end_iso,
            "sample_rate_audio_hz": 10000.0,
            "sample_rate_vib_nominal_hz": 400.0,
            "audio_channels": 4,
            "accel_configured_range_g": self.accel_range_g,
            "audio_frames_total": len(full_audio) if full_audio is not None else 0,
            "vibration_samples_total": len(self.vibration_records),
            "total_packets_received": self.total_packets,
            "dropped_packets": self.dropped_packets,
            "checksum_errors": self.checksum_errors,
            "measured_rpm": mean_rpm,
            "rpm_valid": rpm_valid,
            "rpm_jitter": rpm_jitter,
            "total_tach_pulses": (max(self.all_tach_pulses) - min(self.all_tach_pulses)) if self.all_tach_pulses else 0,
            "mean_dyn_vib_g": mean_dyn,
            "peak_dyn_vib_g": peak_dyn,
            "order_1x_energy_pct": e1_pct,
            "order_2x_energy_pct": e2_pct,
            "is_synthetic": bool(self.use_synthetic),
            "data_provenance": provenance,
            "firmware_protocol": "ESP32 Type 0x05 (10kHz 4-Mic Dual-I2S + 400Hz ADXL345 FIFO + FC-03)",
            "timing_note": "Host arrival timestamps are monotonic offsets (time.monotonic) from stream start. Accelerometer sample timestamps are estimated based on nominal 400Hz FIFO spacing relative to packet arrival. The current Type 0x05 firmware does not provide independent per-sample hardware clock timestamps.",
            "sync_disclaimer": "Dual-I2S audio (10kHz) and ADXL345 vibration (400Hz) are co-streamed in Type 0x05 packets. Packet-level synchronization is bounded by packet duration (~12.8ms). Sub-millisecond cross-modal hardware synchronization is not provided.",
            "notes": self.notes
        }

        with open(metadata_json_path, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)
        print(f"[+] Saved metadata manifest to {os.path.basename(metadata_json_path)}")

        # 5. Append to Global Dataset Manifest (data/dataset/manifest.json)
        manifest_path = os.path.join(self.base_output_dir, "manifest.json")
        self._update_global_manifest(manifest_path, metadata)

        # 6. Legacy compatibility: Mirror latest run into data/experiments/ for compare_motor_experiments.py
        if not self.use_synthetic and self.recording_status == "COMPLETE":
            self._mirror_legacy_experiment(vibration_csv_path, audio_wav_path, metadata)

        # Print Pretty Summary Card
        print("\n" + "=" * 76)
        print(f" EXPERIMENT RECORDING COMPLETE: {self.run_id}")
        print("=" * 76)
        print(f" Status               : {self.recording_status}")
        print(f" Motor ID             : {self.motor_id}")
        print(f" Health Label         : {self.health_label.upper()} (Fault: {self.fault_type})")
        print(f" Acoustic Environment : {self.environment.upper()} ({self.disturbance_desc or 'None'})")
        print(f" Shaft Speed          : {f'{mean_rpm:.1f} RPM' if rpm_valid else 'UNAVAILABLE (0 pulses or stopped)'}")
        print(f" Dynamic Vibration    : {mean_dyn:.4f}g (Peak: {peak_dyn:.4f}g, Range: +/-{self.accel_range_g}g)")
        print(f" 1X / 2X Order Energy : 1X={e1_pct:.1f}% | 2X={e2_pct:.1f}%")
        print(f" Packets / Drops      : {self.total_packets} received / {self.dropped_packets} dropped | Checksum Errors: {self.checksum_errors}")
        print(f" Data Provenance      : {provenance}")
        print(f" Saved Run Directory  : {os.path.relpath(self.run_dir, BASE_DIR)}")
        print("=" * 76 + "\n")

        return {
            "run_dir": self.run_dir,
            "vibration_csv": vibration_csv_path,
            "audio_wav": audio_wav_path,
            "packet_log": packet_log_path,
            "metadata": metadata_json_path
        }

    def _update_global_manifest(self, manifest_path: str, meta: Dict[str, Any]):
        manifest_data = {"version": "1.0", "last_updated": datetime.now(timezone.utc).astimezone().isoformat(), "runs": []}
        if os.path.exists(manifest_path):
            try:
                with open(manifest_path, "r", encoding="utf-8") as f:
                    manifest_data = json.load(f)
            except Exception:
                pass

        existing_ids = [r["run_id"] for r in manifest_data.get("runs", [])]
        summary_entry = {
            "run_id": meta["run_id"],
            "session_id": meta["session_id"],
            "motor_id": meta["motor_id"],
            "motor_health_label": meta["motor_health_label"],
            "acoustic_environment": meta["acoustic_environment"],
            "fault_type": meta["fault_type"],
            "recording_status": meta["recording_status"],
            "duration_s": meta["actual_duration_s"],
            "measured_rpm": meta["measured_rpm"],
            "rpm_valid": meta["rpm_valid"],
            "mean_dyn_vib_g": meta["mean_dyn_vib_g"],
            "order_1x_pct": meta["order_1x_energy_pct"],
            "is_synthetic": meta.get("is_synthetic", False),
            "data_provenance": meta["data_provenance"],
            "run_dir_rel": os.path.relpath(self.run_dir, BASE_DIR)
        }

        if meta["run_id"] not in existing_ids:
            manifest_data.setdefault("runs", []).append(summary_entry)
        else:
            idx = existing_ids.index(meta["run_id"])
            manifest_data["runs"][idx] = summary_entry

        manifest_data["total_runs"] = len(manifest_data["runs"])
        manifest_data["last_updated"] = datetime.now(timezone.utc).astimezone().isoformat()

        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(manifest_data, f, indent=2)
        print(f"[+] Updated dataset-level registry: {os.path.basename(manifest_path)} ({manifest_data['total_runs']} runs total)")

    def _mirror_legacy_experiment(self, csv_src: str, wav_src: str, meta: Dict[str, Any]):
        """Preserves backward compatibility with tools/compare_motor_experiments.py."""
        legacy_dir = os.path.join(BASE_DIR, "data", "experiments")
        os.makedirs(legacy_dir, exist_ok=True)

        if self.health_label == "healthy" and self.environment == "quiet":
            prefix = "motor_healthy_run"
        elif self.health_label == "defective" and self.environment == "quiet":
            prefix = "motor_defective_run"
        elif self.environment == "disturbance":
            prefix = "motor_disturbance_run"
        else:
            return

        import shutil
        target_csv = os.path.join(legacy_dir, f"{prefix}.csv")
        target_wav = os.path.join(legacy_dir, f"{prefix}_audio.wav")
        target_meta = os.path.join(legacy_dir, f"{prefix}_meta.json")

        try:
            shutil.copy2(csv_src, target_csv)
            shutil.copy2(wav_src, target_wav)
            with open(target_meta, "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)
        except Exception:
            pass

    def run(self) -> bool:
        if not self.use_synthetic:
            self.record_hardware()
            if self.recording_status == "FAILED":
                return False
        else:
            if not self.vibration_records:
                self.generate_synthetic_testbench_run()

        # Save artifacts (preserving interrupted/incomplete state for diagnostic inspection)
        self.save_dataset_artifacts()

        # Process exit status: strictly return True (exit code 0) ONLY if recording_status == "COMPLETE"
        # If INTERRUPTED or INCOMPLETE, files are saved for debugging, but process returns False (exit code 1)
        return self.recording_status == "COMPLETE"


def main():
    parser = argparse.ArgumentParser(description="Record Physical Motor Experiment Suite (TinyML Phase 1)")
    # New Standard CLI
    parser.add_argument("--motor-id", type=str, default="motor_A", help="Physical motor identifier (e.g. motor_A, motor_B)")
    parser.add_argument("--health-label", choices=["healthy", "defective"], default=None,
                        help="Motor mechanical condition: healthy or defective")
    parser.add_argument("--environment", choices=["quiet", "disturbance"], default=None,
                        help="Acoustic environmental context: quiet or disturbance")
    parser.add_argument("--fault-type", choices=["none", "unknown", "unbalance", "misalignment", "bearing", "other"],
                        default=None, help="Specific verified mechanical fault type (default: 'unknown' for defective, 'none' for healthy)")
    parser.add_argument("--disturbance-desc", type=str, default="", help="Description of acoustic disturbance (e.g. loudspeaker, clap)")
    parser.add_argument("--duration", type=float, default=30.0, help="Recording duration in seconds (default: 30.0s)")
    parser.add_argument("--port", type=str, default="auto", help="ESP32 serial port (default: auto, detects active CP210x/CH340/COM8)")
    parser.add_argument("--baud", type=int, default=921600, help="ESP32 baudrate (default: 921600)")
    parser.add_argument("--supply-voltage", type=float, default=None, help="Motor DC power supply voltage (V)")
    parser.add_argument("--pulses-per-rev", "--ppr", type=int, default=1, help="Tachometer optical pulses per revolution")
    parser.add_argument("--session-id", type=str, default=None, help="Grouping identifier for recording session (e.g. session_01_pilot)")
    parser.add_argument("--output-dir", type=str, default=None, help="Base dataset directory (default: data/dataset)")
    parser.add_argument("--notes", type=str, default="", help="Observational notes")
    parser.add_argument("--synthetic", action="store_true", help="Explicit offline testbench simulation (for unit tests only)")

    # Legacy argument compatibility
    parser.add_argument("--mode", choices=["healthy", "defective", "disturbance"], default=None,
                        help="[LEGACY] Motor mode alias")

    args = parser.parse_args()

    # Map legacy --mode if passed
    health_label = args.health_label
    environment = args.environment
    fault_type = args.fault_type

    if args.mode is not None and health_label is None:
        if args.mode == "healthy":
            health_label = "healthy"
            environment = "quiet"
            fault_type = "none"
        elif args.mode == "defective":
            health_label = "defective"
            environment = "quiet"
            fault_type = "unknown"
        elif args.mode == "disturbance":
            health_label = "healthy"
            environment = "disturbance"
            fault_type = "none"

    if health_label is None:
        health_label = "healthy"
    if environment is None:
        environment = "quiet"
    if fault_type is None:
        fault_type = "unknown" if health_label == "defective" else "none"

    recorder = MotorExperimentRecorder(
        motor_id=args.motor_id,
        health_label=health_label,
        environment=environment,
        fault_type=fault_type,
        disturbance_desc=args.disturbance_desc,
        duration=args.duration,
        port=args.port,
        baudrate=args.baud,
        supply_voltage=args.supply_voltage,
        pulses_per_rev=args.pulses_per_rev,
        session_id=args.session_id,
        output_dir=args.output_dir,
        notes=args.notes,
        use_synthetic=args.synthetic
    )
    success = recorder.run()
    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
