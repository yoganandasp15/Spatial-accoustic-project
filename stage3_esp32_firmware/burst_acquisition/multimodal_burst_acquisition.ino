#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <esp_timer.h>

// ============================================================================
// HARDWARE PIN ASSIGNMENTS (Exact Verified Silk-Screen PCB Names)
// ============================================================================
#define I2C_SDA_PIN    17   // TX2
#define I2C_SCL_PIN    16   // RX2
#define FC03_DO_PIN    4    // D4
#define I2S_SCK_PIN    22   // D22
#define I2S_WS_PIN     21   // D21
#define I2S_SD_PIN     19   // D19

// ADXL345 Register Definitions
#define ADXL345_ADDR          0x53
#define ADXL345_DEVID         0x00
#define ADXL345_BW_RATE       0x2C
#define ADXL345_POWER_CTL     0x2D
#define ADXL345_DATA_FORMAT   0x31
#define ADXL345_FIFO_CTL      0x38
#define ADXL345_DATAX0        0x32

// Burst Acquisition Parameters
#define BURST_DURATION_MS     200
#define I2S_SAMPLE_RATE       44100
#define AUDIO_BURST_FRAMES    ((I2S_SAMPLE_RATE * BURST_DURATION_MS) / 1000) // 8820 frames
#define VIB_SAMPLE_RATE       400
#define VIB_BURST_SAMPLES     ((VIB_SAMPLE_RATE * BURST_DURATION_MS) / 1000) // 80 samples
#define MAX_TACH_PULSES       64

// Structs for RAM Buffering
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t timestamp_us;
} __attribute__((packed)) vib_sample_t;

typedef struct {
    int16_t left;   // 16-bit packed PCM (scaled from 24-bit INMP441)
    int16_t right;  // 16-bit packed PCM
} __attribute__((packed)) audio_frame_16_t;

// SRAM Allocation
static audio_frame_16_t audio_buffer[AUDIO_BURST_FRAMES]; // 8820 * 4 = 35,280 bytes
static vib_sample_t     vib_buffer[VIB_BURST_SAMPLES];     // 80 * 10 = 800 bytes
static uint32_t         tach_timestamps[MAX_TACH_PULSES];  // 64 * 4 = 256 bytes

volatile uint32_t tach_pulse_count = 0;
volatile uint32_t burst_start_us = 0;
volatile bool burst_active = false;

void IRAM_ATTR fc03_isr() {
    if (burst_active && tach_pulse_count < MAX_TACH_PULSES) {
        tach_timestamps[tach_pulse_count++] = (uint32_t)(esp_timer_get_time() - burst_start_us);
    }
}

void init_adxl345_400hz() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    delay(20);

    // 1. Verify DEVID = 0xE5
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_DEVID);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)1);
    uint8_t devid = Wire.available() ? Wire.read() : 0;
    Serial.printf("[ADXL345] DEVID Register Read: 0x%02X (%s)\n", devid, devid == 0xE5 ? "CONFIRMED 0xE5" : "MISMATCH!");

    // 2. Configure BW_RATE to 400 Hz ODR (0x0C = 400 Hz ODR, Bandwidth = 200 Hz)
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_BW_RATE);
    Wire.write(0x0C);
    Wire.endTransmission();

    // 3. Configure DATA_FORMAT: Full resolution, +/- 2g (0x08)
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_DATA_FORMAT);
    Wire.write(0x08);
    Wire.endTransmission();

    // 4. Configure FIFO: Stream mode (0x80)
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_FIFO_CTL);
    Wire.write(0x80);
    Wire.endTransmission();

    // 5. Measurement mode (0x08)
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_POWER_CTL);
    Wire.write(0x08);
    Wire.endTransmission();
    delay(20);
}

void init_i2s_44k1() {
    gpio_set_pull_mode((gpio_num_t)I2S_SD_PIN, GPIO_PULLDOWN_ONLY);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=======================================================");
    Serial.println(">>> ESP32 SYNCHRONIZED MULTIMODAL BURST ACQUISITION <<<");
    Serial.println("=======================================================");
    Serial.printf("Free Heap: %u bytes (Audio Buffer: %u bytes, Vib: %u bytes)\n",
                  esp_get_free_heap_size(), sizeof(audio_buffer), sizeof(vib_buffer));

    init_adxl345_400hz();
    init_i2s_44k1();

    pinMode(FC03_DO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FC03_DO_PIN), fc03_isr, RISING);

    Serial.println("[*] System Ready.");
    Serial.println("    Send 'B' to trigger a 200ms Synchronized Burst (ASCII telemetry).");
    Serial.println("    Send 'W' to trigger a 200ms Synchronized Burst + Raw Binary Audio Stream.");
}

void execute_synchronized_burst(bool send_binary_audio) {
    Serial.println("\n--- [STARTING 200ms SYNCHRONIZED BURST] ---");
    tach_pulse_count = 0;
    i2s_zero_dma_buffer(I2S_NUM_0);

    // Shared Hardware Reference Timestamp t0
    int64_t t0 = esp_timer_get_time();
    burst_start_us = (uint32_t)t0;
    burst_active = true;

    size_t audio_frames_acquired = 0;
    size_t vib_samples_acquired = 0;
    int32_t dma_raw_buf[64]; // 32 stereo pairs
    size_t bytes_read = 0;

    int64_t next_vib_time = t0;
    const int64_t vib_period_us = 1000000 / VIB_SAMPLE_RATE; // 2500 us (400 Hz)

    int64_t t_end = t0 + (BURST_DURATION_MS * 1000LL);

    while (esp_timer_get_time() < t_end) {
        int64_t now = esp_timer_get_time();

        // 1. Drain I2S Audio DMA Buffer into RAM
        if (audio_frames_acquired < AUDIO_BURST_FRAMES) {
            i2s_read(I2S_NUM_0, dma_raw_buf, sizeof(dma_raw_buf), &bytes_read, 0);
            int frames_in_chunk = bytes_read / 8;
            for (int i = 0; i < frames_in_chunk && audio_frames_acquired < AUDIO_BURST_FRAMES; i++) {
                // INMP441 24-bit data in MSB of 32-bit slot: shift down to 16-bit packed PCM
                audio_buffer[audio_frames_acquired].left = (int16_t)(dma_raw_buf[2 * i] >> 14);
                audio_buffer[audio_frames_acquired].right = (int16_t)(dma_raw_buf[2 * i + 1] >> 14);
                audio_frames_acquired++;
            }
        }

        // 2. Poll ADXL345 at calibrated 400 Hz (every 2500 us)
        if (now >= next_vib_time && vib_samples_acquired < VIB_BURST_SAMPLES) {
            Wire.beginTransmission(ADXL345_ADDR);
            Wire.write(ADXL345_DATAX0);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)6);
            if (Wire.available() == 6) {
                vib_buffer[vib_samples_acquired].x = Wire.read() | (Wire.read() << 8);
                vib_buffer[vib_samples_acquired].y = Wire.read() | (Wire.read() << 8);
                vib_buffer[vib_samples_acquired].z = Wire.read() | (Wire.read() << 8);
                vib_buffer[vib_samples_acquired].timestamp_us = (uint32_t)(now - t0);
                vib_samples_acquired++;
            }
            next_vib_time += vib_period_us;
        }
    }

    burst_active = false;
    int64_t t_actual_end = esp_timer_get_time();
    uint32_t elapsed_us = (uint32_t)(t_actual_end - t0);

    // ------------------------------------------------------------------------
    // TRANSMIT STRUCTURED BURST PAYLOAD OVER UART
    // ------------------------------------------------------------------------
    Serial.println("--- [BURST COMPLETE - TRANSMITTING PAYLOAD] ---");
    Serial.printf("[HEADER] t0_us=%lld, duration_us=%u, audio_frames=%u/%u, vib_samples=%u/%u, tach_pulses=%u\n",
                  t0, elapsed_us, audio_frames_acquired, AUDIO_BURST_FRAMES, vib_samples_acquired, VIB_BURST_SAMPLES, tach_pulse_count);

    // 1. Tachometer Pulse Timestamps
    Serial.print("[TACH_TIMESTAMPS_US] ");
    for (uint32_t p = 0; p < tach_pulse_count; p++) {
        Serial.printf("%u%s", tach_timestamps[p], p < tach_pulse_count - 1 ? "," : "");
    }
    Serial.println();

    // 2. Accelerometer 400 Hz Samples
    Serial.println("[VIB_DATA_START]");
    for (size_t v = 0; v < vib_samples_acquired; v++) {
        Serial.printf("%u,%d,%d,%d,%+0.3f,%+0.3f,%+0.3f\n",
                      vib_buffer[v].timestamp_us,
                      vib_buffer[v].x, vib_buffer[v].y, vib_buffer[v].z,
                      vib_buffer[v].x * 0.0039, vib_buffer[v].y * 0.0039, vib_buffer[v].z * 0.0039);
    }
    Serial.println("[VIB_DATA_END]");

    // 3. Audio Metrics Calculation
    int64_t sum_sq_l = 0, sum_sq_r = 0;
    int16_t min_l = 32767, max_l = -32768;
    int16_t min_r = 32767, max_r = -32768;

    for (size_t a = 0; a < audio_frames_acquired; a++) {
        int16_t l = audio_buffer[a].left;
        int16_t r = audio_buffer[a].right;
        sum_sq_l += (int32_t)l * l;
        sum_sq_r += (int32_t)r * r;
        if (l < min_l) min_l = l;
        if (l > max_l) max_l = l;
        if (r < min_r) min_r = r;
        if (r > max_r) max_r = r;
    }
    float rms_l = audio_frames_acquired > 0 ? sqrt(sum_sq_l / audio_frames_acquired) : 0;
    float rms_r = audio_frames_acquired > 0 ? sqrt(sum_sq_r / audio_frames_acquired) : 0;
    Serial.printf("[AUDIO_METRICS] RMS_L=%0.1f, Min_L=%d, Max_L=%d | RMS_R=%0.1f, Min_R=%d, Max_R=%d | TotalFrames=%u\n",
                  rms_l, min_l, max_l, rms_r, min_r, max_r, audio_frames_acquired);

    // Stream 50 representative non-zero frames (from frame index 500 to 550)
    Serial.println("[AUDIO_EXEMPLARS_500_550_START]");
    for (size_t a = 500; a < 550 && a < audio_frames_acquired; a++) {
        Serial.printf("%d,%d\n", audio_buffer[a].left, audio_buffer[a].right);
    }
    Serial.println("[AUDIO_EXEMPLARS_500_550_END]");

    // If requested, transmit raw binary PCM audio payload
    if (send_binary_audio && audio_frames_acquired > 0) {
        Serial.println("[RAW_AUDIO_BINARY_START]");
        Serial.flush();
        Serial.write((const uint8_t*)audio_buffer, audio_frames_acquired * sizeof(audio_frame_16_t));
        Serial.flush();
        Serial.println("\n[RAW_AUDIO_BINARY_END]");
    }

    Serial.println("--- [END OF BURST TRANSMISSION] ---\n");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'B' || c == 'b') {
            execute_synchronized_burst(false);
        } else if (c == 'W' || c == 'w') {
            execute_synchronized_burst(true);
        }
    }
    delay(10);
}
