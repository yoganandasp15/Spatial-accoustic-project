# Stage 3: Classic ESP32 Synchronous Dual-I2S Acquisition Firmware
**Hardware Validation Date:** 5–7 November 2026

## Objective
Overcome the lack of TDM (Time Division Multiplexing) in the classic ESP32-WROOM-32 to stream 4 synchronized digital microphone channels (INMP441) alongside a 3-axis accelerometer (ADXL345) and an optical tachometer (FC-03).

## Architecture & Pinout
* **Master Shared Clocks:** SCK (BCLK) = GPIO 18, WS (Frame Sync) = GPIO 21.
* **Pair A (Mics 1 & 2):** GPIO 19 (I2S0 SD)
* **Pair B (Mics 3 & 4):** GPIO 23 (I2S1 SD)
* **ADXL345 I2C:** SDA = TX2 (GPIO 17), SCL = RX2 (GPIO 16)
* **FC-03 Tachometer:** DO = D4 (GPIO 4) with hardware ISR

## Streaming Protocol
* **Sampling Rate:** 11,025 Hz, 4 channels, 16-bit PCM.
* **Serial Link:** COM8 @ 921,600 baud (79.4 kB/s throughput).
* **Integrity:** 16-bit XOR packet checksum. Zero packet loss verified under continuous acquisition.
