#!/usr/bin/env python3
"""
Real-time 3D Acoustic & Multimodal Telemetry Web Server Bridge
Serves the Three.js 3D Acoustic Camera frontend at http://localhost:8080/
and streams real-time JSON telemetry from ESP32 serial COM port or high-precision simulation.
"""

import os
import sys
import json
import time
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

# Set root directory to frontend_3d
FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend_3d"

class Acoustic3DHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(FRONTEND_DIR), **kwargs)

    def log_message(self, format, *args):
        # Silence routine asset requests
        pass

def run_server(port=8080):
    server_address = ("0.0.0.0", port)
    httpd = HTTPServer(server_address, Acoustic3DHTTPHandler)
    print("=" * 65)
    print("   SPATIAL MULTIMODAL RADAR 3D WEB SERVER")
    print(f"   Listening on: http://localhost:{port}/")
    print(f"   Root Directory: {FRONTEND_DIR}")
    print("=" * 65)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
        httpd.server_close()

if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    run_server(port)
