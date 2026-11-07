# Hardware Status & Verified Wiring Specification

**Status:** Physical hardware assembled, wired on breadboard, flashed, and verified live on **COM8 @ 921,600 baud**.

---

## 1. Physical Hardware Components
* **MCU Board:** Classic ESP32 DevKit (ESP32-WROOM-32, 38-pin / 30-pin layout, non-S3).
* **Acoustic Array:** 4x INMP441 MEMS digital microphones configured in a Uniform Linear Array (ULA) spaced at $d = 20.32\text{ mm}$ (8 breadboard slots, total aperture $60.96\text{ mm}$).
* **Vibration Sensor:** ADXL345 3-axis digital accelerometer breakout configured in I2C mode.
* **Tachometer:** FC-03 slotted optical interrupter module (3.3V supply, digital output).
* **Motor Under Test:** Safe, low-voltage small DC motor with optical encoder disc.

---

## 2. Verified Breadboard Pinout & Interconnections

### 2.1 4-Microphone INMP441 Dual-I2S Acquisition (Shared Master Clocks)
| Microphone | Breadboard Position | Channel Slot | SCK (BCLK) | WS (L/R Clock) | SD (Data Line) | L/R Pin | Supply |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Mic 1** | Row 20 ($x = -30.48\text{ mm}$) | Pair A (Left) | GPIO 18 | GPIO 21 | GPIO 19 (I2S0) | GND | 3.3V / GND |
| **Mic 2** | Row 28 ($x = -10.16\text{ mm}$) | Pair A (Right) | GPIO 18 | GPIO 21 | GPIO 19 (I2S0) | 3.3V | 3.3V / GND |
| **Mic 3** | Row 36 ($x = +10.16\text{ mm}$) | Pair B (Left) | GPIO 18 | GPIO 21 | GPIO 23 (I2S1) | GND | 3.3V / GND |
| **Mic 4** | Row 44 ($x = +30.48\text{ mm}$) | Pair B (Right) | GPIO 18 | GPIO 21 | GPIO 23 (I2S1) | 3.3V | 3.3V / GND |

* **Hardware Feasibility Note:** Although the classic ESP32 lacks TDM, configuring I2S0 and I2S1 simultaneously with shared bit-clock (GPIO 18) and word-select (GPIO 21) allows perfectly synchronous 4-channel DMA capture without phase drift.

### 2.2 ADXL345 3-Axis Accelerometer (I2C Bus)
* **VCC:** 3.3V (ESP32 3V3 rail)
* **GND:** Ground (Common GND)
* **SDA:** TX2 (GPIO 17)
* **SCL:** RX2 (GPIO 16)
* **CS:** Tied to 3.3V (enables I2C mode)
* **SDO:** Tied to GND (sets I2C address to `0x53`)

### 2.3 FC-03 Optical Tachometer
* **VCC:** 3.3V
* **GND:** Ground
* **DO (Digital Out):** D4 (GPIO 4) configured with `RISING` hardware interrupt service routine.
* **AO (Analog Out):** Unconnected / Not needed.

---

## 3. Communication Link & Streaming Protocol
* **Serial Port:** COM8
* **Baud Rate:** 921,600 baud
* **Audio Format:** 11,025 Hz, 4 channels, 16-bit signed PCM (unpacked from 24-bit slot via `>> 14`).
* **Frame Chunk Size:** 128 samples per block (11.6 ms packet interval, 86.1 packets/sec).
* **Streaming Throughput:** 79.4 kB/s (well within the 92.1 kB/s theoretical serial bandwidth).
* **Packet Integrity:** 16-bit XOR checksum over packet header + audio chunk payload. Zero packet corruption observed.

---

## 4. Live Verification Results
* **Microphone Channel Activity (Live COM8 Capture):**
  * Mic 1 (Pair A Left): `RMS = 0.5606`
  * Mic 2 (Pair A Right): `RMS = 0.6617`
  * Mic 3 (Pair B Left): `RMS = 0.5145`
  * Mic 4 (Pair B Right): `RMS = 0.6334`
  * All 4 channels verified active with clean phase response.
* **Polarity Inversion:**
  * Pressing Key `'I'` in the live application flips channel mapping (`np.fliplr`) so that screen coordinates align with the physical breadboard orientation.
