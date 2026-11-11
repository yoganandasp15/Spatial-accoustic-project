#!/usr/bin/env python3
"""
Stand-alone Active ESP32 COM Port Detector.
Scans for Silicon Labs CP210x, WCH CH340, FTDI, or Espressif USB serial devices.
Filters out virtual Bluetooth COM ports and motherboard ACPI serial ports.
Outputs the detected device name (e.g. COM8) on stdout.
"""
import sys

def detect_port() -> str:
    try:
        import serial.tools.list_ports as lp
        ports = list(lp.comports())
        if not ports:
            return "COM8"

        # 1. Match known USB-UART bridge hardware descriptors (CP210x, CH340, FTDI, ESP32)
        for p in ports:
            h = ((p.hwid or "") + " " + (p.description or "")).lower()
            if "bthenum" in h or "bluetooth" in h or "acpi" in h or "pnp0501" in h:
                continue
            if any(k in h for k in ["10c4:ea60", "cp210", "1a86:7523", "ch340", "0403:6001", "ftdi", "303a:", "esp32"]):
                return p.device

        # 2. Match any real USB Serial device
        for p in ports:
            h = ((p.hwid or "") + " " + (p.description or "")).lower()
            if "bthenum" in h or "bluetooth" in h or "acpi" in h or "pnp0501" in h:
                continue
            if "usb" in h:
                return p.device

        # 3. Check for COM8 explicitly if active
        for p in ports:
            if p.device.upper() == "COM8":
                return p.device

        # Default fallback
        return "COM8"
    except Exception:
        return "COM8"

if __name__ == "__main__":
    print(detect_port())
