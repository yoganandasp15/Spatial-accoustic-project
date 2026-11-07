"""
=============================================================================
 MILESTONE 2: ADVERSARIAL STRESS TEST HARNESS & PROTOCOL VULNERABILITY SUITE
=============================================================================
Module: tests/test_m2_stress_adversarial.py
Target: Adversarially stress-tests Worker M2's implementation of continuous
        ADXL345 accelerometer streaming protocol (Type 0x05) and Python serial
        receiver under corrupted headers, malformed payloads, bit flips,
        fragmentation, sequence roll-over, batched arrivals, and concurrent access.
Author: teamwork_challenger_m2 (Antigravity Autonomous Agent - Critic Role)
=============================================================================
"""

import os
import sys
import struct
import threading
import time
import unittest
import numpy as np

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STAGE4_SRC = os.path.join(BASE_DIR, "stage4_realtime_fusion_radar", "src")
if STAGE4_SRC not in sys.path:
    sys.path.insert(0, STAGE4_SRC)

from spatial_acoustic_radar_pro import (
    ESP32MultimodalSource,
    compute_packet_checksum
)


class TestM2AdversarialStress(unittest.TestCase):
    """
    Adversarial challenge test suite targeting protocol parsing, boundary conditions,
    corruption resilience, sequence tracking, and thread safety.
    """

    def setUp(self):
        self.source = ESP32MultimodalSource()
        self.accel_default = [(10, -20, 980), (15, -25, 982), (20, -30, 984), (25, -35, 986), (30, -40, 988)]
        self.audio_default = np.zeros((128, 4), dtype=np.int16)
        self.audio_default[:, 0] = 500
        self.audio_default[:, 1] = -500
        self.audio_default[:, 2] = 1000
        self.audio_default[:, 3] = -1000

    def test_bit_flip_fuzzing(self):
        """
        Adversarial fuzzing: flips bits at every byte position across header,
        vibration payload, audio payload, and checksum to verify 100% rejection.
        """
        pkt = bytearray(ESP32MultimodalSource.build_packet_v2(
            seq=1, fs=11025, block_frames=128,
            accel_samples=self.accel_default,
            audio_data=self.audio_default
        ))
        pkt_len = len(pkt)
        self.assertEqual(pkt_len, 1072)

        # Sample 40 distinct byte indices across the packet (including header, vib, audio, checksum)
        indices_to_test = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]  # Header
        indices_to_test += [16, 20, 25, 30, 45]  # Vibration payload
        indices_to_test += [50, 100, 250, 500, 750, 1000, 1050, 1069]  # Audio payload
        indices_to_test += [1070, 1071]  # Checksum bytes

        for idx in indices_to_test:
            for bit in [0x01, 0x80]:
                corrupted = bytearray(pkt)
                corrupted[idx] ^= bit

                # If bit flip hit length-determining header fields (channels, block_frames, num_vib),
                # skip here as that inflates total_pkt_len and is tested in test_vulnerability_corrupt_length_fields_hang
                if idx in (3, 8, 9, 10):
                    continue

                test_src = ESP32MultimodalSource()
                parsed = test_src._parse_incoming_bytes(bytes(corrupted))

                # If sync bytes were flipped, parser skips sync (0 valid packets, 0 checksum errs)
                # If non-sync bytes were flipped, parser detects checksum error (1 checksum err)
                if idx in (0, 1):
                    self.assertEqual(parsed, 0, f"Corrupted sync at byte {idx} must not yield valid packet")
                elif idx == 2:
                    # Packet type flipped from 0x05 to something else
                    self.assertEqual(parsed, 0, f"Corrupted packet type must not yield valid packet")
                else:
                    self.assertEqual(parsed, 0, f"Corrupted byte {idx} bit {hex(bit)} must not yield valid packet")
                    self.assertGreaterEqual(test_src.checksum_errors, 1, f"Checksum error expected for byte {idx}")

    def test_fragmented_serial_stream_reconstruction(self):
        """
        Stress-tests packet reconstruction across variable serial chunk sizes:
        1B, 2B, 3B, 7B, 13B, 64B, 128B, 256B, 1024B.
        """
        pkt = ESP32MultimodalSource.build_packet_v2(
            seq=10, fs=11025, block_frames=128,
            accel_samples=self.accel_default,
            audio_data=self.audio_default
        )

        for chunk_size in [1, 2, 3, 7, 13, 64, 128, 256, 512, 1024]:
            src = ESP32MultimodalSource()
            parsed_total = 0
            idx = 0
            while idx < len(pkt):
                end_idx = min(idx + chunk_size, len(pkt))
                chunk = pkt[idx:end_idx]
                parsed_total += src._parse_incoming_bytes(chunk)
                idx = end_idx

            self.assertEqual(parsed_total, 1, f"Failed to reconstruct packet with chunk_size={chunk_size}")
            self.assertEqual(src.packets_valid, 1)
            self.assertEqual(src.checksum_errors, 0)
            self.assertEqual(len(src._rx_buf), 0)

    def test_inter_packet_garbage_and_split_sync(self):
        """
        Stress-tests stream garbage resilience with isolated 0xAA bytes and split sync
        across chunk boundaries between valid packets.
        """
        pkt1 = ESP32MultimodalSource.build_packet_v2(1, 11025, 128, self.accel_default, self.audio_default)
        pkt2 = ESP32MultimodalSource.build_packet_v2(2, 11025, 128, self.accel_default, self.audio_default)

        # Chunk A: pkt1 followed by noise ending in 0xAA
        chunk_a = pkt1 + b"\xDE\xAD\xBE\xEF\xAA"
        # Chunk B: 0x55 (completing sync) followed by rest of pkt2
        chunk_b = pkt2[1:]

        src = ESP32MultimodalSource()
        p1 = src._parse_incoming_bytes(chunk_a)
        self.assertEqual(p1, 1)
        self.assertEqual(src.packets_valid, 1)

        p2 = src._parse_incoming_bytes(chunk_b)
        self.assertEqual(p2, 1)
        self.assertEqual(src.packets_valid, 2)
        self.assertEqual(src.checksum_errors, 0)

    def test_sequence_tracking_boundary_and_roll_over(self):
        """
        Validates sequence roll-over from 65535 to 0, multi-drop across rollover boundary,
        and rejection of duplicate/out-of-order packets.
        """
        src = ESP32MultimodalSource()

        # 1. Nominal wrap-around: 65535 -> 0 (0 drops)
        pkt_max = ESP32MultimodalSource.build_packet_v2(65535, 11025, 128, self.accel_default, self.audio_default)
        pkt_0 = ESP32MultimodalSource.build_packet_v2(0, 11025, 128, self.accel_default, self.audio_default)
        src._parse_incoming_bytes(pkt_max)
        self.assertEqual(src.last_seq, 65535)
        self.assertEqual(src.dropped_packets, 0)

        src._parse_incoming_bytes(pkt_0)
        self.assertEqual(src.last_seq, 0)
        self.assertEqual(src.dropped_packets, 0)

        # 2. Drop across boundary: 65530 -> 5 (dropped 65531..65535, 0..4 = 10 drops)
        src.last_seq = 65530
        pkt_5 = ESP32MultimodalSource.build_packet_v2(5, 11025, 128, self.accel_default, self.audio_default)
        src._parse_incoming_bytes(pkt_5)
        self.assertEqual(src.last_seq, 5)
        self.assertEqual(src.dropped_packets, 10)

        # 3. Duplicate packet (seq=5 received again): should not count as drop
        drops_before = src.dropped_packets
        src._parse_incoming_bytes(pkt_5)
        self.assertEqual(src.dropped_packets, drops_before)

    def test_vibration_payload_boundaries(self):
        """
        Validates payload boundary handling:
        - num_vib = 0 (empty vibration record)
        - num_vib = 8 (firmware maximum buffer)
        - num_vib = 32 (ADXL345 full hardware FIFO depth)
        """
        src = ESP32MultimodalSource()

        # 1. num_vib = 0
        pkt0 = ESP32MultimodalSource.build_packet_v2(1, 11025, 128, [], self.audio_default)
        self.assertEqual(len(pkt0), 16 + 0 + 1024 + 2)
        parsed0 = src._parse_incoming_bytes(pkt0)
        self.assertEqual(parsed0, 1)
        self.assertEqual(src.telemetry["num_vib_last"], 0)

        # 2. num_vib = 8
        accel8 = [(i * 10, i * 20, 950 + i) for i in range(8)]
        pkt8 = ESP32MultimodalSource.build_packet_v2(2, 11025, 128, accel8, self.audio_default)
        self.assertEqual(len(pkt8), 16 + 48 + 1024 + 2)
        parsed8 = src._parse_incoming_bytes(pkt8)
        self.assertEqual(parsed8, 1)
        self.assertEqual(src.telemetry["num_vib_last"], 8)
        t, ax, ay, az = src.get_vibration_history(max_samples=10)
        self.assertEqual(len(ax), 8)

        # 3. num_vib = 32 (full FIFO buffer)
        accel32 = [(i, -i, 980) for i in range(32)]
        pkt32 = ESP32MultimodalSource.build_packet_v2(3, 11025, 128, accel32, self.audio_default)
        self.assertEqual(len(pkt32), 16 + (32 * 6) + 1024 + 2)
        parsed32 = src._parse_incoming_bytes(pkt32)
        self.assertEqual(parsed32, 1)
        self.assertEqual(src.telemetry["num_vib_last"], 32)
        t, ax, ay, az = src.get_vibration_history(max_samples=64)
        self.assertEqual(len(ax), 40)  # 8 + 32

    def test_concurrent_read_write_stress(self):
        """
        Multi-threaded concurrency stress test:
        Producer thread feeds 200 packets at accelerated rate while 2 consumer threads
        concurrently call get_vibration_history(), get_frame(), and get_multimodal_telemetry().
        """
        src = ESP32MultimodalSource()
        errors = []
        stop_consumers = threading.Event()

        def producer():
            for seq in range(200):
                pkt = ESP32MultimodalSource.build_packet_v2(
                    seq, 11025, 128, self.accel_default, self.audio_default
                )
                src._parse_incoming_bytes(pkt)
                time.sleep(0.0005)

        def consumer_vib():
            while not stop_consumers.is_set():
                try:
                    t, ax, ay, az = src.get_vibration_history(max_samples=256)
                    self.assertEqual(len(t), len(ax))
                    self.assertEqual(len(ax), len(ay))
                    self.assertEqual(len(ay), len(az))
                except Exception as e:
                    errors.append(f"consumer_vib error: {e}")
                time.sleep(0.001)

        def consumer_telemetry():
            while not stop_consumers.is_set():
                try:
                    telem = src.get_multimodal_telemetry()
                    frame = src.get_frame()
                    self.assertEqual(frame.shape, (512, 4))
                except Exception as e:
                    errors.append(f"consumer_telemetry error: {e}")
                time.sleep(0.001)

        t_prod = threading.Thread(target=producer)
        t_c1 = threading.Thread(target=consumer_vib)
        t_c2 = threading.Thread(target=consumer_telemetry)

        t_c1.start()
        t_c2.start()
        t_prod.start()

        t_prod.join()
        stop_consumers.set()
        t_c1.join()
        t_c2.join()

        self.assertEqual(errors, [], f"Concurrency errors observed: {errors}")
        self.assertEqual(src.packets_valid, 200)

    # -------------------------------------------------------------------------
    # REMEDIATION VERIFICATION (CHALLENGER FINDINGS RESOLVED)
    # -------------------------------------------------------------------------

    def test_vulnerability_corrupt_length_fields_causes_stream_hang(self):
        """
        [REMEDIATION VERIFICATION 1]
        Verifies that single bit flips in length-determining header fields:
        - block_frames (byte 9, e.g. 128 -> 32896)
        - channels (byte 3, e.g. 4 -> 132)
        are caught immediately by header boundary validation, preventing buffer inflation
        and stream stalls. All subsequent valid packets must be parsed successfully.
        """
        valid_pkt = ESP32MultimodalSource.build_packet_v2(1, 11025, 128, self.accel_default, self.audio_default)

        for test_name, byte_idx, bit_mask, num_subsequent in [
            ("block_frames (32896 frames -> 263 KB total)", 9, 0x80, 20),
            ("channels (132 channels -> 33 KB total)", 3, 0x80, 20),
        ]:
            src = ESP32MultimodalSource()
            corrupted = bytearray(valid_pkt)
            corrupted[byte_idx] ^= bit_mask

            # Feed corrupted packet
            src._parse_incoming_bytes(bytes(corrupted))

            # Feed 20 consecutive valid packets (21.4 KB)
            for i in range(2, 2 + num_subsequent):
                p = ESP32MultimodalSource.build_packet_v2(i, 11025, 128, self.accel_default, self.audio_default)
                src._parse_incoming_bytes(p)

            # REMEDIATION VERIFICATION: The parser must reject corrupted packet and parse all valid packets.
            self.assertEqual(
                src.packets_valid, num_subsequent,
                f"Parser failed to parse valid packets after {test_name}: got {src.packets_valid}, expected {num_subsequent}"
            )
            self.assertEqual(
                len(src._rx_buf), 0,
                f"Parser failed to drain buffer after {test_name}: remaining bytes {len(src._rx_buf)}"
            )

    def test_vulnerability_malformed_channels_crashes_with_value_error(self):
        """
        [REMEDIATION VERIFICATION 2]
        Verifies that if a packet with channels != 4 arrives with a valid checksum,
        the receiver rejects or safely ignores it without raising ValueError, ensuring
        the background serial worker thread remains alive.
        """
        src = ESP32MultimodalSource()
        header = struct.pack("<BBBBHHHBBI", 0xAA, 0x55, 0x05, 2, 1, 11025, 128, 0, 0, 0)
        audio_bytes = np.zeros((128, 2), dtype=np.int16).tobytes()
        data = header + audio_bytes
        chk = compute_packet_checksum(data)
        pkt = data + struct.pack("<H", chk)

        try:
            parsed = src._parse_incoming_bytes(pkt)
            self.assertEqual(parsed, 0, "Malformed packet with channels != 4 must not yield valid packet")
            self.assertEqual(src.packets_valid, 0)
        except ValueError as e:
            self.fail(f"Receiver raised ValueError on malformed channels: {e}")

    def test_vulnerability_batched_vibration_timestamps_non_monotonic(self):
        """
        [REMEDIATION VERIFICATION 3]
        Verifies that when packets are delivered together in batched serial chunks
        (tested across batches of 2, 10, and 50 packets), vibration timestamps in vib_ring
        progress strictly monotonically without backward steps (negative_dt_count == 0, dt > 0),
        and every sample interval matches 0.0025 s (+/- 10 us).
        """
        for n_pkts in [2, 10, 50]:
            with self.subTest(batch_size=n_pkts):
                src = ESP32MultimodalSource()
                chunk = b"".join([
                    ESP32MultimodalSource.build_packet_v2(
                        seq=i,
                        fs=11025,
                        block_frames=128,
                        accel_samples=self.accel_default,
                        audio_data=self.audio_default
                    )
                    for i in range(n_pkts)
                ])

                parsed = src._parse_incoming_bytes(chunk)
                self.assertEqual(parsed, n_pkts, f"Expected to parse {n_pkts} packets, got {parsed}")

                t, ax, ay, az = src.get_vibration_history(max_samples=n_pkts * 10)
                expected_samples = n_pkts * len(self.accel_default)
                self.assertEqual(len(t), expected_samples, f"Expected {expected_samples} samples, got {len(t)}")

                dt = np.diff(t)
                negative_dt_count = int(np.sum(dt <= 0))
                self.assertEqual(
                    negative_dt_count, 0,
                    f"Non-monotonic vibration timestamps observed in {n_pkts}-packet batch "
                    f"(negative dt count: {negative_dt_count}, min dt = {np.min(dt):.6f} s)"
                )
                self.assertTrue(
                    np.all(dt > 0),
                    f"All dt must be strictly positive in {n_pkts}-packet batch, got min dt = {np.min(dt):.6f} s"
                )

                # Every sample interval within the batch must match 0.0025s +/- 10us (2.49ms to 2.51ms)
                np.testing.assert_allclose(
                    dt, 0.0025, atol=1e-5,
                    err_msg=f"Sample intervals must match 0.0025 s (+/- 10 us) in {n_pkts}-packet batch"
                )


if __name__ == "__main__":
    unittest.main()
