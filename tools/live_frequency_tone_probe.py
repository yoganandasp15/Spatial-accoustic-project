#!/usr/bin/env python3
"""
ESP32 Real-Time Continuous 4-Microphone Audio Tone & Multimodal Sensor Probe.
Auto-detects active ESP32 COM port and streams:
  - 4-Channel INMP441 Microphone RMS Levels (Mic 1, Mic 2, Mic 3, Mic 4)
  - FFT Peak Dominant Frequency & GCC-PHAT Acoustic Bearing
  - ADXL345 3-Axis Accelerometer Telemetry (ax, ay, az in g)
  - FC-03 Optical Tachometer Marker Pulses & Instantaneous Shaft RPM
"""

import sys
import time
import struct
import argparse
import os
from pathlib import Path
import numpy as np

# Ensure tools directory is in sys.path for port detection
TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

try:
    from find_port import detect_port
except ImportError:
    def detect_port() -> str:
        return "COM8"

FS_DEFAULT = 10000.0
BASELINE_M = 0.02032  # 8 pin pitches = 20.32 mm
C_SOUND = 343.0


def run_continuous_probe(port: str = "auto", baud: int = 921600):
    if port == "auto" or not port:
        port = detect_port()

    try:
        import serial
    except ImportError:
        print("[-] ERROR: pyserial is required. Install via: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    if sys.stdout.isatty():
        os.system('cls' if os.name == 'nt' else 'clear')

    print("=" * 88, flush=True)
    print("      ESP32 REAL-TIME 4-CHANNEL AUDIO TONE & MULTIMODAL PROBE (PRO V6)", flush=True)
    print(f"      Port: {port} @ {baud} Baud | ULA Baseline: {BASELINE_M*1000:.2f} mm | Speed of Sound: {C_SOUND} m/s", flush=True)
    print("=" * 88, flush=True)
    print(f"Connecting to ESP32 on {port}...", flush=True)

    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.dtr = False
        ser.rts = False
        ser.timeout = 0.25
        ser.open()
    except Exception as e:
        print(f"\n[-] ERROR: Failed to open serial port {port}: {e}")
        print("    Ensure no other program (such as the Radar HUD or Arduino IDE) is using this port.")
        input("\nPress Enter to exit...")
        return

    time.sleep(0.15)
    ser.reset_input_buffer()
    ser.write(b'S\n')
    ser.flush()

    print("[+] Connected! Streaming continuous 4-channel audio & sensor data.", flush=True)
    print("    Play a test tone (e.g. 500 Hz, 1000 Hz, 2000 Hz) from your phone near the array.", flush=True)
    print("    Press Ctrl+C to stop streaming.\n", flush=True)
    print("-" * 88, flush=True)
    print(f"{'TIME':<7} | {'MIC 1':<6} {'MIC 2':<6} {'MIC 3':<6} {'MIC 4':<6} | {'PEAK FREQ':<9} | {'BEARING':<8} | {'VIBRATION [X, Y, Z] (g)':<20} | {'RPM PULSES'}", flush=True)
    print("-" * 88, flush=True)

    buf_len = 1024
    rolling_audio = np.zeros((buf_len, 4), dtype=np.float32)

    last_print = 0.0
    t0 = time.time()
    last_pulse_count = 0
    last_pulse_time = time.time()
    instant_rpm = 0.0

    try:
        while True:
            b1 = ser.read(1)
            if b1 != b'\xaa':
                continue
            b2 = ser.read(1)
            if b2 != b'\x55':
                continue

            hdr_bytes = ser.read(18)
            if len(hdr_bytes) != 18:
                continue

            pkt_type, seq, sr, ch, blk, ax, ay, az, pulses = struct.unpack('<BHHBHHhhI', hdr_bytes)
            if ch not in (2, 4) or blk == 0 or blk > 2048:
                continue

            payload_len = blk * ch * 2
            pcm_data = ser.read(payload_len)
            chk_bytes = ser.read(2)

            if len(pcm_data) != payload_len or len(chk_bytes) != 2:
                continue

            samples = np.frombuffer(pcm_data, dtype=np.int16)
            matrix = samples.reshape((blk, ch))

            # Push into rolling buffer
            rolling_audio = np.roll(rolling_audio, -blk, axis=0)
            if ch == 4:
                rolling_audio[-blk:, :] = matrix
            elif ch == 2:
                rolling_audio[-blk:, :2] = matrix
                rolling_audio[-blk:, 2:] = 0.0

            now = time.time()
            if now - last_print >= 0.20:
                last_print = now
                elapsed = now - t0

                # Calculate RMS per channel
                ac = rolling_audio - np.mean(rolling_audio, axis=0, keepdims=True)
                rms = np.sqrt(np.mean(ac**2, axis=0))

                # Tachometer RPM estimation
                dt_p = now - last_pulse_time
                if dt_p >= 0.50:
                    dp = pulses - last_pulse_count
                    if dp > 0:
                        instant_rpm = (dp / dt_p) * 60.0
                    else:
                        instant_rpm = 0.0
                    last_pulse_count = pulses
                    last_pulse_time = now

                # FFT frequency analysis on mean signal
                fs_actual = float(sr) if sr > 0 else FS_DEFAULT
                mono = np.mean(ac, axis=1)
                win = np.hanning(buf_len)
                fft_mag = np.abs(np.fft.rfft(mono * win))
                freqs = np.fft.rfftfreq(buf_len, 1.0 / fs_actual)

                mask = (freqs >= 100.0) & (freqs <= (fs_actual / 2.0 - 50.0))
                if np.any(mask) and np.max(fft_mag[mask]) > 10.0:
                    peak_idx = np.argmax(fft_mag[mask])
                    peak_freq = freqs[mask][peak_idx]
                else:
                    peak_freq = 0.0

                # GCC-PHAT DOA Bearing (Pair A: Mic 1 vs Mic 2)
                bearing_deg = 0.0
                if rms[0] > 100 and rms[1] > 100:
                    L_fft = np.fft.rfft(ac[:, 0])
                    R_fft = np.fft.rfft(ac[:, 1])
                    cross = L_fft * np.conj(R_fft)
                    phat = cross / (np.abs(cross) + 1e-5)
                    gcc = np.fft.fftshift(np.fft.irfft(phat))
                    lags = np.arange(-len(gcc)//2, len(gcc)//2) / fs_actual
                    max_tau = (BASELINE_M / C_SOUND) + (1.0 / fs_actual)
                    valid = np.abs(lags) <= max_tau
                    if np.any(valid):
                        best_lag = lags[valid][np.argmax(gcc[valid])]
                        sin_th = np.clip((C_SOUND * best_lag) / BASELINE_M, -1.0, 1.0)
                        bearing_deg = float(np.degrees(np.arcsin(sin_th)))

                # ADXL & Tach
                ax_g = ax / 1000.0
                ay_g = ay / 1000.0
                az_g = az / 1000.0

                tone_flag = ""
                if np.mean(rms) > 300 and peak_freq > 150:
                    tone_flag = f" [TONE: {peak_freq:.0f}Hz]"

                rpm_str = f"{pulses:5d} ({instant_rpm:4.0f} RPM)" if instant_rpm > 0 else f"{pulses:5d}"
                print(
                    f"{elapsed:5.1f}s  | {rms[0]:5.0f}  {rms[1]:5.0f}  {rms[2]:5.0f}  {rms[3]:5.0f} | "
                    f"{peak_freq:5.0f} Hz   | {bearing_deg:+6.1f}° | "
                    f"[{ax_g:+.2f}, {ay_g:+.2f}, {az_g:+.2f}] | {rpm_str}{tone_flag}",
                    flush=True
                )

    except KeyboardInterrupt:
        print("\n[*] Stopping live audio probe...")
    finally:
        ser.close()
        print("[*] Serial port closed.")


def main():
    parser = argparse.ArgumentParser(description="ESP32 4-Channel Live Audio Tone & Multimodal Probe")
    parser.add_argument("--port", type=str, default="auto", help="Serial port (default: auto)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    args = parser.parse_args()
    run_continuous_probe(port=args.port, baud=args.baud)


if __name__ == "__main__":
    main()
