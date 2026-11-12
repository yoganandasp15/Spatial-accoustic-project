#!/usr/bin/env python3
"""
Stand-alone Active ESP32 COM Port Detector.
Scans for Silicon Labs CP210x, WCH CH340, FTDI, or Espressif USB serial devices.
Filters out virtual Bluetooth COM ports and motherboard ACPI serial ports.
Outputs the detected device name (e.g. COM8) on stdout.
Optionally verifies that Type 0x05 binary telemetry is actively streaming.
"""
import sys
import time
import argparse

def detect_port(strict: bool = False, check_streaming: bool = False) -> str:
    try:
        import serial.tools.list_ports as lp
        ports = list(lp.comports())
        if not ports:
            return "NONE" if (strict or check_streaming) else "COM8"

        candidate_port = None

        # 1. Match known USB-UART bridge hardware descriptors (CP210x, CH340, FTDI, ESP32)
        for p in ports:
            h = ((p.hwid or "") + " " + (p.description or "")).lower()
            if "bthenum" in h or "bluetooth" in h or "acpi" in h or "pnp0501" in h:
                continue
            if any(k in h for k in ["10c4:ea60", "cp210", "1a86:7523", "ch340", "0403:6001", "ftdi", "303a:", "esp32"]):
                candidate_port = p.device
                break

        # 2. Match any real USB Serial device
        if not candidate_port:
            for p in ports:
                h = ((p.hwid or "") + " " + (p.description or "")).lower()
                if "bthenum" in h or "bluetooth" in h or "acpi" in h or "pnp0501" in h:
                    continue
                if "usb" in h:
                    candidate_port = p.device
                    break

        if not candidate_port:
            return "NONE" if (strict or check_streaming) else "COM8"

        # 3. If check_streaming requested, verify the ESP32 is actively streaming Type 0x05 sync bytes
        if check_streaming:
            try:
                import serial
                s = serial.Serial()
                s.port = candidate_port
                s.baudrate = 921600
                s.dtr = False
                s.rts = False
                s.timeout = 0.35
                s.open()
                s.reset_input_buffer()
                s.write(b"S\n")
                s.flush()
                time.sleep(0.15)
                rx = s.read(1500)
                s.close()
                if b"\xaa\x55" in rx:
                    return candidate_port
                else:
                    return "NONE"
            except Exception:
                return "NONE"

        return candidate_port
    except Exception:
        return "NONE" if (strict or check_streaming) else "COM8"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Active ESP32 COM Port Detector")
    parser.add_argument("--strict", action="store_true", help="Return NONE if no genuine USB device is active")
    parser.add_argument("--check-streaming", action="store_true", help="Verify that Type 0x05 binary telemetry is actively streaming")
    args = parser.parse_args()
    print(detect_port(strict=args.strict, check_streaming=args.check_streaming))
