# Hardware Status & Verified Wiring Specification

**Status:** Physical hardware prototype assembled on breadboard, flashed, and verified live on **COM8 @ 921,600 baud** (Type 0x05 Binary Protocol with 10 kHz Audio & 400 Hz ADXL345 Stream FIFO). CP2102 hardware UART bridge confirmed locked with 0 framing errors.

---

## 1. Physical Hardware Components
* **MCU Board:** Classic ESP32 DevKit (ESP32-WROOM-32, 38-pin / 30-pin layout, dual-core @ 240 MHz).
* **Acoustic Array:** 4x INMP441 MEMS digital microphones configured in a Uniform Linear Array (ULA) spaced at $d = 20.32\text{ mm}$ (8 breadboard slots, total aperture $60.96\text{ mm}$).
* **Vibration Sensor:** ADXL345 3-axis digital accelerometer breakout configured in I2C mode with 400 Hz Stream FIFO.
* **Tachometer:** FC-03 slotted optical interrupter module (3.3V supply, digital pulse output with 2500 µs ISR refractory debounce).
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

* **Hardware Clock Synchronization Invariant:** Although the classic ESP32 lacks hardware TDM, configuring I2S0 (Master) and I2S1 (Slave) with shared bit-clock (GPIO 18) and word-select (GPIO 21) guarantees identical sampling frequencies across all 4 channels (0 PPM drift).
* **Historical Evolution of SCK Pinout (GPIO 18 vs GPIO 22):** In early experimental bringup (`dual_i2s_4mic/ESP32_Classic_4Mic_DualI2S.ino`), I2S0 drove GPIO 22 (BCLK) and GPIO 21 (WS), requiring physical loopback jumpers from GPIO 22 to GPIO 18 and GPIO 21 to GPIO 5. In the unified production firmware (`ESP32_Spatial_Acoustic_Radar_Firmware.ino`), the bus is simplified directly: I2S0 drives physical SCK on GPIO 18 and WS on GPIO 21, and the GPIO MUX input buffer (`PIN_INPUT_ENABLE`) is enabled directly on GPIO 18 and 21 pads so I2S1 receives the master clocks without external loopback jumpers.
* **DMA Alignment & Startup Flushing:** At startup, both I2S peripherals drain and discard 4 DMA blocks in lockstep before entering the streaming loop, eliminating initial startup transient skew.

### 2.2 ADXL345 3-Axis Accelerometer (I2C Bus)
* **VCC:** 3.3V (ESP32 3V3 rail)
* **GND:** Ground (Common GND)
* **SDA:** TX2 (GPIO 17)
* **SCL:** RX2 (GPIO 16)
* **CS:** Tied to 3.3V (enables I2C mode)
* **SDO:** Tied to GND (sets I2C address to `0x53`)
* **Verified Sensor State:** Readout confirmed `DEVID = 0xE5`, configured in 400 Hz ODR Stream FIFO mode (`BW_RATE = 0x0C`, `FIFO_CTL = 0x80`).

### 2.3 FC-03 Optical Tachometer
* **VCC:** 3.3V
* **GND:** Ground
* **DO (Digital Out):** D4 (GPIO 4) configured with `FALLING` hardware interrupt service routine (`fc03_isr`) with `INPUT_PULLUP` and 200 µs refractory debounce filter (supporting up to 5,000 pulses/sec = 15,000 RPM at 20 PPR, or 300,000 RPM at 1 PPR), matching active-low optical blade entry. Live pin state is monitored via `digitalRead(FC03_DO_PIN)` in diagnostic mode (`X` key).
* **AO (Analog Out):** Unconnected / Not needed.

---

## 3. Communication Link & Streaming Protocol
* **Serial Port:** COM8
* **Baud Rate:** 921,600 baud (CP2102 hardware limit with zero bit/framing errors, raw channel capacity $92,160\text{ bytes/s}$; receiver also supports 1,152,000 baud).
* **Audio Format:** 10,000 Hz, 4 channels, 16-bit signed PCM (unpacked from INMP441 24-bit MSB-aligned slot via `>> 16`).
* **Frame Chunk Size:** 128 samples per block (12.8 ms packet interval, 78.125 packets/sec).
* **Packet Structure (Type 0x05):** 16-byte `PacketHeader` + $N \times 6$-byte ADXL345 samples (typically 4–5 samples, max 8) + 1024-byte audio chunk (128 frames x 4 ch x 2 bytes) + 2-byte 16-bit XOR checksum (seed `0x55AA`) = 1,072 bytes per packet.
* **Streaming Throughput:** $78.125\text{ pkt/s} \times 1,072\text{ B/pkt} = 83,750\text{ B/s} = 83.75\text{ kB/s}$ ($90.9\%$ link utilization at 921,600 baud, $9.1\%$ headroom; $72.7\%$ link utilization at 1.152M baud).
* **Channel Unpacking (I2S_CHANNEL_FMT_RIGHT_LEFT):**
  * DMA Word 0 (`[2*i]`): Right channel (tied to 3.3V: Mic 2 on GPIO 19, Mic 4 on GPIO 23)
  * DMA Word 1 (`[2*i+1]`): Left channel (tied to GND: Mic 1 on GPIO 19, Mic 3 on GPIO 23)
  * Firmly mapped: `m1 = dma_rx0[2*i+1]`, `m2 = dma_rx0[2*i]`, `m3 = dma_rx1[2*i+1]`, `m4 = dma_rx1[2*i]` to maintain physical left-to-right ordering across breadboard.
* **UART TX Buffer:** Initialized with `Serial.setTxBufferSize(4096)` in `setup()`, allowing non-blocking DMA transmission without stalling the audio loop.
* **Packet Integrity & Validation:** Contiguous 16-bit XOR checksum computed across header and payloads; sequence counter continuity verified in Python receiver; dropped packet count tracked in real time.
* **Orientation-Invariant Order Tracking & ISO Velocity:**
  * Multi-axis power spectrum $P(f) = |X_{\text{dyn}}(f)|^2 + |Y_{\text{dyn}}(f)|^2 + |Z_{\text{dyn}}(f)|^2$ provides orientation-invariant rotational order analysis without Euclidean magnitude rectification doubling.
  * True ISO 10816-1 velocity RMS ($v_{\text{rms}}$ in mm/s) is computed via frequency-domain $\omega$-arithmetic integration in the 10 Hz – 200 Hz band.
* **Phase Alignment & Inter-DMA Calibration:**
  * Master/Slave shared bit-clock (GPIO 18) and word-select (GPIO 21) eliminate sample-clock drift (0 PPM).
  * Empirical inter-DMA phase calibration (`calibrate_i2s_phase.py`) measured mean skew between Pair A (I2S0) and Pair B (I2S1) at $\Delta \tau = -24.69\ \mu\text{s}$ ($-1.09$ samples at 44.1 kHz, or $-0.27$ samples at 11.025 kHz), which is compensated via a 1-sample integer alignment shift.
* **Dynamic Vibration Gravity Decoupling:**
  * Static gravity is decoupled via running per-axis DC bias tracking ($\bar{g}_x, \bar{g}_y, \bar{g}_z$ with exponential moving average $\alpha=0.01$).
  * Dynamic vibration magnitude is computed as $a_{\text{dyn}} = \sqrt{(a_x - \bar{g}_x)^2 + (a_y - \bar{g}_y)^2 + (a_z - \bar{g}_z)^2}$, preventing cancellation of lateral vibration signatures.

---

## 4. Live Verification & Evidence Breakdown
* **Live Hardware Streaming Evidence (COM8 Capture):**
  * Mic 1 (Pair A Left): `RMS = 0.5606`
  * Mic 2 (Pair A Right): `RMS = 0.6617`
  * Mic 3 (Pair B Left): `RMS = 0.5145`
  * Mic 4 (Pair B Right): `RMS = 0.6334`
  * All 4 channels verified active with distinct acoustic responses.
* **Polarity Inversion:**
  * Pressing Key `'I'` in the live application flips channel mapping (`np.fliplr`) so that screen coordinates align with the physical breadboard orientation.
* **Evidence Separation Invariant:**
  1. *Physical Prototype:* Real-time 4-channel audio streaming, ADXL345 vibration reading, FC-03 pulse counting on COM8.
  2. *Machine Fault Benchmark:* MAFAULDA and CWRU open datasets provide the validated ground truth for $1\times$ mass unbalance and $2\times$ misalignment fault physics.
  3. *Acoustic Beamforming Proof:* Stage 2 numerical simulation suite provides 25-point off-grid benchmark ($0.34^\circ$ median error).
