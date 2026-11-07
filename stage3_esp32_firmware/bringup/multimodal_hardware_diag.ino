#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <driver/gpio.h>

// Pin Definitions according to the verified layout
#define I2C_SDA_PIN    17
#define I2C_SCL_PIN    16
#define FC03_DO_PIN    4
#define I2S_SCK_PIN    22
#define I2S_WS_PIN     21
#define I2S_SD_PIN     19

// ADXL345 Registers
#define ADXL345_ADDR          0x53
#define ADXL345_POWER_CTL     0x2D
#define ADXL345_DATA_FORMAT   0x31
#define ADXL345_DATAX0        0x32

volatile uint32_t fc03_pulse_count = 0;
void IRAM_ATTR fc03_isr() {
    fc03_pulse_count++;
}

bool adxl_found = false;

void init_i2s() {
    // Enable internal pull-down on GPIO19
    gpio_set_pull_mode((gpio_num_t)I2S_SD_PIN, GPIO_PULLDOWN_ONLY);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
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

    // Discard initial 150ms settling time
    delay(150);
    size_t bytes_read = 0;
    int32_t dummy_buf[128];
    for (int i = 0; i < 10; i++) {
        i2s_read(I2S_NUM_0, dummy_buf, sizeof(dummy_buf), &bytes_read, portMAX_DELAY);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n============================================================");
    Serial.println(">>> ESP32 MULTIMODAL HARDWARE & SENSOR SELF-TEST ENGINE <<<");
    Serial.println("============================================================");

    // 1. I2C Initialization & Bus Scan
    Serial.printf("[1/3] Scanning I2C Bus on SDA=%d, SCL=%d (100kHz standard mode)...\n", I2C_SDA_PIN, I2C_SCL_PIN);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    delay(10);
    int sda_state = digitalRead(I2C_SDA_PIN);
    int scl_state = digitalRead(I2C_SCL_PIN);
    Serial.printf("      Bus line idle states: SDA(17)=%s, SCL(16)=%s\n", 
                  sda_state ? "HIGH (OK)" : "LOW (STUCK / GROUNDED!)", 
                  scl_state ? "HIGH (OK)" : "LOW (STUCK / GROUNDED!)");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
    delay(50);

    int nDevices = 0;
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("      [+] Found I2C Device at 0x%02X", address);
            if (address == ADXL345_ADDR) {
                Serial.println(" -> CONFIRMED: ADXL345 Accelerometer!");
                adxl_found = true;
            } else if (address == 0x1D) {
                Serial.println(" -> ADXL345 (Alt Address SDO=3.3V)!");
                adxl_found = true;
            } else {
                Serial.println();
            }
            nDevices++;
        }
    }

    if (!adxl_found) {
        Serial.println("      [-] WARNING: ADXL345 not responding at 0x53.");
        Serial.println("          Check: CS -> 3.3V, SDO -> GND, SDA -> Pin 17, SCL -> Pin 16.");
    } else {
        // Configure ADXL345: Wake up, measure mode (+/- 2g full res)
        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_POWER_CTL);
        Wire.write(0x08); // Measurement mode
        Wire.endTransmission();

        Wire.beginTransmission(ADXL345_ADDR);
        Wire.write(ADXL345_DATA_FORMAT);
        Wire.write(0x08); // Full resolution, +/- 2g
        Wire.endTransmission();
    }

    // 2. FC-03 Optical RPM Sensor
    Serial.printf("[2/3] Initializing FC-03 Tachometer on GPIO%d...\n", FC03_DO_PIN);
    pinMode(FC03_DO_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FC03_DO_PIN), fc03_isr, RISING);
    Serial.printf("      [+] FC-03 Optical Sensor Interrupt Armed! Current Beam State: %s\n",
                  digitalRead(FC03_DO_PIN) ? "BLOCKED (DO=HIGH)" : "OPEN (DO=LOW)");

    // 3. INMP441 Dual-Microphone I2S
    Serial.printf("[3/3] Initializing INMP441 Stereo I2S (SCK=%d, WS=%d, SD=%d)...\n",
                  I2S_SCK_PIN, I2S_WS_PIN, I2S_SD_PIN);
    init_i2s();
    Serial.println("      [+] I2S DMA Engine Active (Sample Rate: 44.1 kHz, 32-bit slot)!");
    Serial.println("============================================================\n");
    Serial.println("STARTING CONTINUOUS STREAM: [ADXL345 X,Y,Z] | [FC-03 PULSES] | [MIC1 (L) / MIC2 (R) AUDIO LEVEL]\n");
}

uint32_t last_report_time = 0;

void loop() {
    // Read Audio Samples from I2S
    int32_t audio_buffer[64]; // 32 samples L, 32 samples R
    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, audio_buffer, sizeof(audio_buffer), &bytes_read, 20);

    int64_t sum_l = 0;
    int64_t sum_r = 0;
    int sample_count = bytes_read / 8; // 8 bytes per stereo pair (32-bit L + 32-bit R)

    for (int i = 0; i < sample_count; i++) {
        int32_t left_sample = audio_buffer[2 * i] >> 14;     // Scale 24-bit data
        int32_t right_sample = audio_buffer[2 * i + 1] >> 14;
        sum_l += (int64_t)left_sample * left_sample;
        sum_r += (int64_t)right_sample * right_sample;
    }

    float rms_l = sample_count > 0 ? sqrt(sum_l / sample_count) : 0;
    float rms_r = sample_count > 0 ? sqrt(sum_r / sample_count) : 0;

    // Report every 250ms
    if (millis() - last_report_time >= 250) {
        last_report_time = millis();

        // 1. Read ADXL345
        float ax = 0, ay = 0, az = 0;
        if (adxl_found) {
            Wire.beginTransmission(ADXL345_ADDR);
            Wire.write(ADXL345_DATAX0);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)ADXL345_ADDR, (uint8_t)6);

            if (Wire.available() == 6) {
                int16_t x = Wire.read() | (Wire.read() << 8);
                int16_t y = Wire.read() | (Wire.read() << 8);
                int16_t z = Wire.read() | (Wire.read() << 8);
                // In full-res mode, scale factor is ~3.9 mg/LSB (0.0039 * 9.80665 m/s^2)
                ax = x * 0.0039;
                ay = y * 0.0039;
                az = z * 0.0039;
            }
        }

        // 2. Read FC-03 Pulse count
        uint32_t pulses = fc03_pulse_count;
        bool beam_state = digitalRead(FC03_DO_PIN);

        // 3. Print Unified Telemetry Line
        Serial.printf("[VIB g] X:%+5.2f Y:%+5.2f Z:%+5.2f | [RPM BEAM: %s (Total:%u)] | [MIC RMS] L:%6.0f | R:%6.0f\n",
                      ax, ay, az,
                      beam_state ? "BLOCK" : "OPEN ",
                      pulses,
                      rms_l, rms_r);
    }
}
