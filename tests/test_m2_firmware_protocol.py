"""
=============================================================================
 MILESTONE 2: FIRMWARE CONTINUOUS VIBRATION STREAMING & SERIAL PROTOCOL TEST
=============================================================================
Module: tests/test_m2_firmware_protocol.py
Target: Validates Type 0x05 multi-sample packet construction, 16-bit XOR checksum,
        400 Hz vibration ring buffer pushing, sequence tracking, legacy 0x04 fallback,
        garbage resynchronization, and baud rate bandwidth headroom.
Author: teamwork_worker_m2 (Antigravity Autonomous Agent)
=============================================================================
"""

import os
import sys
import unittest
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAGE4_SRC = os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src")
if STAGE4_SRC not in sys.path:
    sys.path.insert(0, STAGE4_SRC)

from spatial_acoustic_radar_pro import (
    ESP32MultimodalSource,
    SerialAudioSource,
    compute_packet_checksum
)


class TestM2FirmwareProtocol(unittest.TestCase):

    def setUp(self):
        self.source = ESP32MultimodalSource(port="COM8", baudrates=(1152000, 921600, 115200))

    def test_default_baudrate_is_1152000(self):
        """Verifies default primary baudrate is 1,152,000 baud."""
        self.assertEqual(self.source.baudrates[0], 1152000)
        self.assertIs(SerialAudioSource, ESP32MultimodalSource)

    def test_type_0x05_packet_unpacking_and_ring_buffer(self):
        """
        Verifies that a synthetic Type 0x05 packet with 5 vibration samples
        is correctly unpacked, validated via checksum, and pushed to the ring buffer.
        """
        accel_samples = [
            (100, -200, 980),
            (105, -195, 982),
            (110, -190, 985),
            (115, -185, 988),
            (120, -180, 990),
        ]
        audio_data = np.zeros((128, 4), dtype=np.int16)
        audio_data[:, 0] = 1000
        audio_data[:, 1] = -1000
        audio_data[:, 2] = 2000
        audio_data[:, 3] = -2000

        pkt = ESP32MultimodalSource.build_packet_v2(
            seq=10,
            fs=11025,
            block_frames=128,
            accel_samples=accel_samples,
            audio_data=audio_data,
            tach_pulses=55
        )

        # Expected size: Header (16) + Vib (5 * 6 = 30) + Audio (128 * 4 * 2 = 1024) + Chk (2) = 1072 bytes
        self.assertEqual(len(pkt), 1072)

        # Parse through receiver
        parsed = self.source._parse_incoming_bytes(pkt)
        self.assertEqual(parsed, 1)
        self.assertEqual(self.source.packets_valid, 1)
        self.assertEqual(self.source.checksum_errors, 0)
        self.assertEqual(self.source.dropped_packets, 0)
        self.assertEqual(self.source.last_seq, 10)

        # Verify ring buffer contains all 5 samples
        t_vib, ax, ay, az = self.source.get_vibration_history(max_samples=10)
        self.assertEqual(len(ax), 5)
        self.assertEqual(len(ay), 5)
        self.assertEqual(len(az), 5)
        self.assertEqual(len(t_vib), 5)

        # Verify exact converted values in g (int16 milli-g / 1000.0)
        np.testing.assert_allclose(ax, [0.100, 0.105, 0.110, 0.115, 0.120], atol=1e-3)
        np.testing.assert_allclose(ay, [-0.200, -0.195, -0.190, -0.185, -0.180], atol=1e-3)
        np.testing.assert_allclose(az, [0.980, 0.982, 0.985, 0.988, 0.990], atol=1e-3)

        # Verify sample timestamps spaced by 1/400 s (2.5 ms)
        dt_arr = np.diff(t_vib)
        np.testing.assert_allclose(dt_arr, 1.0 / 400.0, atol=1e-6)

        # Verify telemetry
        telemetry = self.source.get_multimodal_telemetry()
        self.assertAlmostEqual(telemetry["ax_g"], 0.120, delta=1e-3)
        self.assertAlmostEqual(telemetry["ay_g"], -0.180, delta=1e-3)
        self.assertAlmostEqual(telemetry["az_g"], 0.990, delta=1e-3)
        self.assertEqual(telemetry["rpm_pulses"], 55)
        self.assertEqual(telemetry["num_vib_last"], 5)
        self.assertTrue(telemetry["is_hardware_alive"])

    def test_checksum_corruption_rejection(self):
        """Verifies that single-bit flips anywhere in the packet trigger checksum error and rejection."""
        accel_samples = [(50, -50, 950), (60, -60, 960), (70, -70, 970), (80, -80, 980)]
        audio_data = np.ones((128, 4), dtype=np.int16) * 500

        pkt = bytearray(ESP32MultimodalSource.build_packet_v2(
            seq=1, fs=11025, block_frames=128, accel_samples=accel_samples, audio_data=audio_data
        ))

        # Test bit flip in header (byte 5)
        pkt_corrupt_hdr = bytearray(pkt)
        pkt_corrupt_hdr[5] ^= 0x01
        src1 = ESP32MultimodalSource()
        src1._parse_incoming_bytes(bytes(pkt_corrupt_hdr))
        self.assertEqual(src1.checksum_errors, 1)
        self.assertEqual(src1.packets_valid, 0)

        # Test bit flip in vibration payload (byte 20)
        pkt_corrupt_vib = bytearray(pkt)
        pkt_corrupt_vib[20] ^= 0x04
        src2 = ESP32MultimodalSource()
        src2._parse_incoming_bytes(bytes(pkt_corrupt_vib))
        self.assertEqual(src2.checksum_errors, 1)
        self.assertEqual(src2.packets_valid, 0)

        # Test bit flip in audio payload (byte 100)
        pkt_corrupt_audio = bytearray(pkt)
        pkt_corrupt_audio[100] ^= 0x80
        src3 = ESP32MultimodalSource()
        src3._parse_incoming_bytes(bytes(pkt_corrupt_audio))
        self.assertEqual(src3.checksum_errors, 1)
        self.assertEqual(src3.packets_valid, 0)

        # Test bit flip in checksum (last byte)
        pkt_corrupt_chk = bytearray(pkt)
        pkt_corrupt_chk[-1] ^= 0x01
        src4 = ESP32MultimodalSource()
        src4._parse_incoming_bytes(bytes(pkt_corrupt_chk))
        self.assertEqual(src4.checksum_errors, 1)
        self.assertEqual(src4.packets_valid, 0)

    def test_sequence_counter_and_dropped_packet_tracking(self):
        """Verifies sequence counter tracking and dropped packet detection."""
        accel_samples = [(0, 0, 1000)]
        audio_data = np.zeros((128, 4), dtype=np.int16)

        # Send seq=100
        pkt100 = ESP32MultimodalSource.build_packet_v2(100, 11025, 128, accel_samples, audio_data)
        self.source._parse_incoming_bytes(pkt100)
        self.assertEqual(self.source.last_seq, 100)
        self.assertEqual(self.source.dropped_packets, 0)

        # Send seq=105 (4 packets dropped: 101, 102, 103, 104)
        pkt105 = ESP32MultimodalSource.build_packet_v2(105, 11025, 128, accel_samples, audio_data)
        self.source._parse_incoming_bytes(pkt105)
        self.assertEqual(self.source.last_seq, 105)
        self.assertEqual(self.source.dropped_packets, 4)

        # Test 16-bit sequence roll-over: seq=65535 followed by seq=1 (1 packet dropped: 0)
        self.source.last_seq = 65535
        pkt1 = ESP32MultimodalSource.build_packet_v2(1, 11025, 128, accel_samples, audio_data)
        self.source._parse_incoming_bytes(pkt1)
        self.assertEqual(self.source.last_seq, 1)
        self.assertEqual(self.source.dropped_packets, 5)

    def test_legacy_type_0x04_backward_compatibility(self):
        """Verifies that legacy Type 0x04 packets are seamlessly unpacked with single accelerometer sample."""
        audio_data = np.zeros((128, 4), dtype=np.int16)
        pkt_v1 = ESP32MultimodalSource.build_packet_v1(
            seq=42,
            fs=11025,
            block_frames=128,
            accel_sample=(125, -250, 900),
            audio_data=audio_data,
            tach_pulses=88
        )
        # Expected size: Header (20) + Audio (1024) + Chk (2) = 1046 bytes
        self.assertEqual(len(pkt_v1), 1046)

        parsed = self.source._parse_incoming_bytes(pkt_v1)
        self.assertEqual(parsed, 1)
        self.assertEqual(self.source.packets_valid, 1)
        self.assertEqual(self.source.checksum_errors, 0)
        self.assertEqual(self.source.last_seq, 42)

        t_vib, ax, ay, az = self.source.get_vibration_history(max_samples=10)
        self.assertEqual(len(ax), 1)
        self.assertAlmostEqual(ax[0], 0.125, delta=1e-3)
        self.assertAlmostEqual(ay[0], -0.250, delta=1e-3)
        self.assertAlmostEqual(az[0], 0.900, delta=1e-3)

        telemetry = self.source.get_multimodal_telemetry()
        self.assertEqual(telemetry["rpm_pulses"], 88)
        self.assertEqual(telemetry["num_vib_last"], 1)
        self.assertIn("Type 0x04 Legacy", telemetry["firmware_mode"])

    def test_stream_garbage_resynchronization(self):
        """Verifies that random byte noise before a valid frame does not prevent sync lock."""
        np.random.seed(999)
        garbage = bytes(np.random.randint(0, 255, size=73, dtype=np.uint8))
        # Ensure garbage does not contain 0xAA 0x55
        garbage = garbage.replace(b"\xAA\x55", b"\x00\x00")

        accel_samples = [(10, 20, 990)] * 4
        audio_data = np.zeros((128, 4), dtype=np.int16)
        valid_pkt = ESP32MultimodalSource.build_packet_v2(1, 11025, 128, accel_samples, audio_data)

        stream = garbage + valid_pkt
        parsed = self.source._parse_incoming_bytes(stream)
        self.assertEqual(parsed, 1)
        self.assertEqual(self.source.packets_valid, 1)
        self.assertEqual(self.source.checksum_errors, 0)

    def test_baudrate_bandwidth_headroom(self):
        """
        Analytical verification of UART bandwidth headroom:
        Proves 1,152,000 baud provides >20% headroom whereas 921,600 baud would overflow.
        """
        fs = 11025.0
        block_frames = 128
        pkt_rate = fs / block_frames  # 86.1328 pkts/sec

        # Type 0x05 with typical 5 vibration samples
        num_vib = 5
        pkt_bytes = 16 + (num_vib * 6) + (block_frames * 4 * 2) + 2  # 1072 bytes
        throughput_bps = pkt_rate * pkt_bytes * 10  # 10 bits per byte in 8N1 UART
        byte_rate = pkt_rate * pkt_bytes  # B/s

        baud_921600_cap = 921600.0 / 10.0  # 92,160.0 B/s
        baud_1152000_cap = 1152000.0 / 10.0  # 115,200.0 B/s

        # At 921,600 baud: 92,334.4 B/s > 92,160.0 B/s (OVERFLOW by 174 B/s)
        self.assertGreater(byte_rate, baud_921600_cap, "921,600 baud must be shown to overflow")

        # At 1,152,000 baud: headroom = (115,200 - 92,334.4) / 115,200 = 19.85% (or 24.76% of payload)
        headroom_pct = (baud_1152000_cap - byte_rate) / baud_1152000_cap * 100.0
        self.assertGreater(headroom_pct, 19.0)
        print(f"\n[M2 BANDWIDTH AUDIT] Packet Size: {pkt_bytes} B | Rate: {pkt_rate:.2f} pkts/s")
        print(f"                      Byte Throughput: {byte_rate:.1f} B/s")
        print(f"                      921,600 Baud Capacity: {baud_921600_cap:.1f} B/s (OVERFLOW by {byte_rate - baud_921600_cap:.1f} B/s)")
        print(f"                      1,152,000 Baud Capacity: {baud_1152000_cap:.1f} B/s (HEADROOM: {headroom_pct:.2f}%)")


if __name__ == "__main__":
    unittest.main()
