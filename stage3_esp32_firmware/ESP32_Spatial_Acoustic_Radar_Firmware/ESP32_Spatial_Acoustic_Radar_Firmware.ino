#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <soc/io_mux_reg.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS (Verified Breadboard Connections)
// ============================================================================
#define I2C_SDA_PIN      17   // TX2
#define I2C_SCL_PIN      16   // RX2
#define FC03_DO_PIN      4    // D4 (Tachometer ISR)
#define I2S_SCK_PIN      18   // D18 (Master Bit Clock)
#define I2S_WS_PIN       21   // D21 (Word Select / Frame Sync)
#define I2S0_SD_PIN      19   // D19 (Pair A: Mic 1 Left, Mic 2 Right)
#define I2S1_SD_PIN      23   // D23 (Pair B: Mic 3 Left, Mic 4 Right)

// Serial & Acquisition Parameters
#define SERIAL_BAUD      1152000
#define SAMPLE_RATE      11025   // Fits perfectly in 1,152,000 baud with 24.7% headroom
#define CHUNK_FRAMES     128     // 128 frames = 11.6 ms per packet (86.1 packets/sec)
#define MAX_ACCEL_PER_CHUNK 8    // Typically 4-5 samples at 400 Hz ODR per 11.6 ms chunk

// ADXL345 Register Definitions
#define ADXL345_ADDR        0x53
#define ADXL345_DEVID       0x00
#define ADXL345_BW_RATE     0x2C
#define ADXL345_POWER_CTL   0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_FIFO_CTL    0x38
#define ADXL345_FIFO_STATUS 0x39
#define ADXL345_DATAX0      0x32

// Packet Sync Header
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55
#define PKT_TYPE_4CH_LEGACY  0x04
#define PKT_TYPE_4CH_MULTIVIB 0x05

#pragma pack(push, 1)
struct PacketHeader {
    uint8_t  sync1;        // 0xAA
    uint8_t  sync2;        // 0x55
    uint8_t  pkt_type;     // 0x05 (PKT_TYPE_4CH_MULTIVIB)
    uint8_t  channels;     // 4
    uint16_t seq;          // Sequence counter
    uint16_t sample_rate;  // 11025
    uint16_t block_frames; // 128
    uint8_t  num_vib;      // Number of accelerometer samples in payload (typically 4-5, max 8)
    uint8_t  reserved;     // 0x00
    uint32_t tach_pulses;  // Total tachometer pulse count
};

struct AccelSample {
    int16_t x_mg;          // Accelerometer X (milli-g)
    int16_t y_mg;          // Accelerometer Y (milli-g)
    int16_t z_mg;          // Accelerometer Z (milli-g)
};
#pragma pack(pop)

// Struct for 4-channel audio sample
struct __attribute__((packed)) AudioSample4Ch {
    int16_t m1; // Pair A Left  (Mic 1, x = -30.48 mm)
    int16_t m2; // Pair A Right (Mic 2, x = -10.16 mm)
    int16_t m3; // Pair B Left  (Mic 3, x = +10.16 mm)
    int16_t m4; // Pair B Right (Mic 4, x = +30.48 mm)
};

static AudioSample4Ch tx_audio_chunk[CHUNK_FRAMES];
static AccelSample tx_accel_chunk[MAX_ACCEL_PER_CHUNK];
static int32_t dma_rx0[CHUNK_FRAMES * 2];
static int32_t dma_rx1[CHUNK_FRAMES * 2];

// Telemetry state
volatile uint32_t tach_pulse_count = 0;
bool adxl_present = false;
bool continuous_streaming = true;
uint16_t packet_seq = 0;
uint32_t last_ascii_time = 0;

void IRAM_ATTR fc03_isr() {
    tach_pulse_count++;
}

bool init_adxl345() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    delay(20);

    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_DEVID);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)1);
    uint8_t devid = Wire.available() ? Wire.read() : 0;
    adxl_present = (devid == 0xE5);

    if (adxl_present) {
        // 400 Hz ODR (BW_RATE = 0x0C)
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_BW_RATE);
        Wire.write(0x0C);
        Wire.endTransmission();

        // +/- 2g Full Resolution (3.9 mg/LSB, DATA_FORMAT = 0x08)
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_DATA_FORMAT);
        Wire.write(0x08);
        Wire.endTransmission();

        // FIFO Stream Mode (FIFO_CTL = 0x80)
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_FIFO_CTL);
        Wire.write(0x80);
        Wire.endTransmission();

        // Measurement Mode (POWER_CTL = 0x08)
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_POWER_CTL);
        Wire.write(0x08);
        Wire.endTransmission();
    }
    return adxl_present;
}

uint8_t read_adxl_fifo(AccelSample *out_buf, uint8_t max_samples) {
    if (!adxl_present) {
        out_buf[0].x_mg = 0;
        out_buf[0].y_mg = 0;
        out_buf[0].z_mg = -820;
        return 1;
    }

    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_FIFO_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)1);
    uint8_t entries = Wire.available() ? (Wire.read() & 0x3F) : 0;

    // Graceful fallback for clone chip or empty FIFO: read latest single sample directly
    if (entries == 0) {
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_DATAX0);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)6);
        if (Wire.available() == 6) {
            int16_t rx = Wire.read() | (Wire.read() << 8);
            int16_t ry = Wire.read() | (Wire.read() << 8);
            int16_t rz = Wire.read() | (Wire.read() << 8);
            out_buf[0].x_mg = (int16_t)(rx * 3.9f);
            out_buf[0].y_mg = (int16_t)(ry * 3.9f);
            out_buf[0].z_mg = (int16_t)(rz * 3.9f);
            return 1;
        } else {
            out_buf[0].x_mg = 0;
            out_buf[0].y_mg = 0;
            out_buf[0].z_mg = -820;
            return 1;
        }
    }

    if (entries > max_samples) entries = max_samples;
    for (uint8_t i = 0; i < entries; i++) {
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_DATAX0);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)6);
        if (Wire.available() == 6) {
            int16_t rx = Wire.read() | (Wire.read() << 8);
            int16_t ry = Wire.read() | (Wire.read() << 8);
            int16_t rz = Wire.read() | (Wire.read() << 8);
            out_buf[i].x_mg = (int16_t)(rx * 3.9f);
            out_buf[i].y_mg = (int16_t)(ry * 3.9f);
            out_buf[i].z_mg = (int16_t)(rz * 3.9f);
        } else {
            out_buf[i].x_mg = 0;
            out_buf[i].y_mg = 0;
            out_buf[i].z_mg = -820;
        }
    }
    return entries;
}

void init_dual_i2s_4mic() {
    // Hardware Clocking Invariant:
    // I2S0 (Master) drives physical SCK (GPIO 18) and WS (GPIO 21).
    // I2S1 (Slave) samples on the exact same physical clock pins via Schmitt input buffers.
    // This hardware-locks the sampling frequency (0 PPM frequency drift between pairs).
    // Note on Sample Alignment: Both DMA receivers are queued consecutively; while hardware
    // clocks are locked, minor inter-peripheral DMA descriptor startup offsets may exist
    // until verified via simultaneous multi-channel impulse cross-correlation.
    gpio_set_pull_mode((gpio_num_t)I2S0_SD_PIN, GPIO_PULLDOWN_ONLY);
    gpio_set_pull_mode((gpio_num_t)I2S1_SD_PIN, GPIO_PULLDOWN_ONLY);

    // 1. Install I2S1 as SLAVE FIRST
    i2s_config_t cfg1 = {
        .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 16,
        .dma_buf_len = CHUNK_FRAMES,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_pin_config_t pins1 = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = -1,
        .data_in_num = I2S1_SD_PIN
    };
    i2s_driver_install(I2S_NUM_1, &cfg1, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pins1);

    // 2. Install I2S0 as MASTER SECOND (Drives SCK=18 & WS=21)
    i2s_config_t cfg0 = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 16,
        .dma_buf_len = CHUNK_FRAMES,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_pin_config_t pins0 = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = -1,
        .data_in_num = I2S0_SD_PIN
    };
    i2s_driver_install(I2S_NUM_0, &cfg0, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins0);

    // 3. Keep Schmitt trigger input buffer enabled on GPIO 18 and 21 pads so I2S1 receives clock
    PIN_INPUT_ENABLE(PERIPHS_IO_MUX_GPIO18_U);
    PIN_INPUT_ENABLE(PERIPHS_IO_MUX_GPIO21_U);

    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_zero_dma_buffer(I2S_NUM_1);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(400);

    // Initialize Multimodal Hardware
    init_adxl345();
    init_dual_i2s_4mic();

    // FC-03 Tachometer ISR
    pinMode(FC03_DO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FC03_DO_PIN), fc03_isr, RISING);

    Serial.println("\n[SPATIAL_RADAR] ESP32 4-Mic Dual-I2S Multimodal Streamer Online");
    Serial.printf("[SPATIAL_RADAR] fs=%d, channels=4, baud=%d, ADXL345=%s (400Hz Stream FIFO)\n",
                  SAMPLE_RATE, SERIAL_BAUD, adxl_present ? "OK" : "DEFAULT");
}

void loop() {
    // 1. Process incoming commands
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'S' || c == 's') {
            continuous_streaming = true;
        } else if (c == 'X' || c == 'x') {
            continuous_streaming = false;
        } else if (c == 'P' || c == 'p') {
            Serial.printf("[PONG] ESP32 4-Mic Radar Streamer Alive (fs=%d, baud=%d, type=0x05)\n", SAMPLE_RATE, SERIAL_BAUD);
        }
    }

    // 2. Read 128 frames from both I2S0 and I2S1
    size_t r0 = 0, r1 = 0;
    const size_t target_bytes = CHUNK_FRAMES * 2 * sizeof(int32_t); // 1024 bytes
    esp_err_t e0 = i2s_read(I2S_NUM_0, dma_rx0, target_bytes, &r0, pdMS_TO_TICKS(50));
    esp_err_t e1 = i2s_read(I2S_NUM_1, dma_rx1, target_bytes, &r1, pdMS_TO_TICKS(50));

    if (e0 == ESP_OK && e1 == ESP_OK && r0 == target_bytes && r1 == target_bytes) {
        int64_t s1 = 0, s2 = 0, s3 = 0, s4 = 0;

        // Unpack 24-bit audio in 32-bit slot:
        // INMP441 transmits 24-bit audio MSB-aligned in bits [31:8] with bits [7:0] zero-padded.
        // Shifting >> 16 maps the upper 16 MSBs directly into signed int16_t with unity digital gain,
        // preventing peak clipping on loud acoustic transients.
        for (int i = 0; i < CHUNK_FRAMES; i++) {
            int16_t m1 = (int16_t)(dma_rx0[2 * i] >> 16);     // Pair A Left  (Mic 1, Row 20)
            int16_t m2 = (int16_t)(dma_rx0[2 * i + 1] >> 16); // Pair A Right (Mic 2, Row 28)
            int16_t m3 = (int16_t)(dma_rx1[2 * i] >> 16);     // Pair B Left  (Mic 3, Row 36)
            int16_t m4 = (int16_t)(dma_rx1[2 * i + 1] >> 16); // Pair B Right (Mic 4, Row 44)

            tx_audio_chunk[i].m1 = m1;
            tx_audio_chunk[i].m2 = m2;
            tx_audio_chunk[i].m3 = m3;
            tx_audio_chunk[i].m4 = m4;

            s1 += (int32_t)m1 * m1;
            s2 += (int32_t)m2 * m2;
            s3 += (int32_t)m3 * m3;
            s4 += (int32_t)m4 * m4;
        }

        // Read continuous accelerometer samples from ADXL345 FIFO & tachometer
        uint8_t num_vib = read_adxl_fifo(tx_accel_chunk, MAX_ACCEL_PER_CHUNK);
        uint32_t pulses = tach_pulse_count;

        if (continuous_streaming) {
            PacketHeader hdr;
            hdr.sync1 = SYNC_BYTE_1;
            hdr.sync2 = SYNC_BYTE_2;
            hdr.pkt_type = PKT_TYPE_4CH_MULTIVIB;
            hdr.channels = 4;
            hdr.seq = packet_seq++;
            hdr.sample_rate = SAMPLE_RATE;
            hdr.block_frames = CHUNK_FRAMES;
            hdr.num_vib = num_vib;
            hdr.reserved = 0x00;
            hdr.tach_pulses = pulses;

            // 16-bit XOR Checksum across header, vibration payload, and audio chunk
            uint16_t checksum = 0x55AA;
            size_t byte_idx = 0;

            const uint8_t *h_bytes = (const uint8_t *)&hdr;
            for (size_t i = 0; i < sizeof(hdr); i++, byte_idx++) {
                checksum ^= (uint16_t)(h_bytes[i] << (byte_idx % 8));
            }

            const uint8_t *v_bytes = (const uint8_t *)tx_accel_chunk;
            for (size_t i = 0; i < num_vib * sizeof(AccelSample); i++, byte_idx++) {
                checksum ^= (uint16_t)(v_bytes[i] << (byte_idx % 8));
            }

            const uint8_t *a_bytes = (const uint8_t *)tx_audio_chunk;
            for (size_t i = 0; i < sizeof(tx_audio_chunk); i++, byte_idx++) {
                checksum ^= (uint16_t)(a_bytes[i] << (byte_idx % 8));
            }

            // Transmit binary packet: Header (16B) + Vibration (num_vib * 6B) + Audio (1024B) + Checksum (2B)
            Serial.write((const uint8_t *)&hdr, sizeof(hdr));
            if (num_vib > 0) {
                Serial.write((const uint8_t *)tx_accel_chunk, num_vib * sizeof(AccelSample));
            }
            Serial.write((const uint8_t *)tx_audio_chunk, sizeof(tx_audio_chunk));
            Serial.write((const uint8_t *)&checksum, sizeof(checksum));
        } else {
            // Periodic ASCII telemetry mode (every 200ms)
            if (millis() - last_ascii_time >= 200) {
                last_ascii_time = millis();
                float rms1 = sqrt(s1 / CHUNK_FRAMES);
                float rms2 = sqrt(s2 / CHUNK_FRAMES);
                float rms3 = sqrt(s3 / CHUNK_FRAMES);
                float rms4 = sqrt(s4 / CHUNK_FRAMES);
                int16_t ax = num_vib > 0 ? tx_accel_chunk[num_vib - 1].x_mg : 0;
                int16_t ay = num_vib > 0 ? tx_accel_chunk[num_vib - 1].y_mg : 0;
                int16_t az = num_vib > 0 ? tx_accel_chunk[num_vib - 1].z_mg : -820;
                Serial.printf("[4-MIC LIVE] Mic1(D19_L):%6.1f | Mic2(D19_R):%6.1f | Mic3(D23_L):%6.1f | Mic4(D23_R):%6.1f | Accel(samples=%d): X=%+0.2fg Y=%+0.2fg Z=%+0.2fg | Tach:%u\n",
                              rms1, rms2, rms3, rms4, num_vib, ax / 1000.0f, ay / 1000.0f, az / 1000.0f, pulses);
            }
        }
    }
}
