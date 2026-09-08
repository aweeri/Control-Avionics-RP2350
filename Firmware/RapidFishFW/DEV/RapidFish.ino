#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <FastLED.h>
#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

/*
 * RapidFish - The CARP Rocket Avionics firmware
 * Core functionality: High-frequency sensor data acquisition, flight state estimation, 
 * and persistent flash storage for telemetry logging.
 */

// ============================================================================
// [1] CONFIGURATION SECTION
// ============================================================================

// --- Hardware Pins ---
#define LED_PIN 25
#define NUM_LEDS 1
#define LED_BRIGHTNESS 255

#define PYRO1_PIN 20
#define PYRO2_PIN 15

#define LSM_CS_PIN 7
#define LSM_SCK_PIN 6
#define LSM_MOSI_PIN 5
#define LSM_MISO_PIN 4

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

// --- Flight Physics Thresholds ---
const float LAUNCH_G_THRESHOLD   = 2.0f;  // Acceleration trigger for liftoff
const float BURNOUT_G_THRESHOLD  = 0.5f;  // Acceleration drop indicating motor burnout
const float APOGEE_DIP_METERS    = 6.0f; // Altitude threshold for apogee event
const float GROUND_G_TOLERANCE   = 0.2f;  // G-load variance allowed in stationary state

// --- Timing Thresholds (Milliseconds) ---
const uint32_t MIN_MOTOR_BURN_MS = 500;   // Minimum ignition time required
const uint32_t MAX_MOTOR_BURN_MS = 3000;  // Failsafe timeout for motor burnout
const uint32_t RECOVERY_DELAY_MS = 0;     // Post-apogee wait before deployment
const uint32_t GROUND_WAIT_MS    = 5000;  // Duration of stillness to confirm landing

// --- Flash Storage ---
const uint32_t FLIGHT_DATA_FLASH_SIZE = 2 * 1024 * 1024; // 16MB allocation
const uint32_t SYNC_WORD = 0x1ACFFC1D;                    // Identifier for log start

// ============================================================================
// [2] DATA STRUCTURES & GLOBALS
// ============================================================================

enum FlightState {
    STATE_BOOTING,
    STATE_ARMED,
    STATE_ACCELERATING,
    STATE_COAST,
    STATE_RECOVERY,
    STATE_CHUTE,
    STATE_GROUND
};

// Data frame aligned to 32-byte boundary for flash write efficiency
struct __attribute__((packed)) LogFrame {
    uint32_t sync_word;
    uint32_t timestamp;
    int16_t ax, ay, az;      // Accel: (m/s²) * 100
    int16_t gx, gy, gz;      // Gyro: (rad/s) * 1000
    uint32_t pressure;       // Pascal
    uint8_t flight_state;    // Current state machine stage
    int8_t temperature;      // Barometer C
    int8_t core_temp;        // RP2040 internal temp
    uint8_t _pad[5];         // Maintains 32-byte alignment
};

CRGB leds[NUM_LEDS];
Adafruit_LSM6DSOX lsm;
Adafruit_BMP3XX bmp;

// Flash memory state tracking
uint32_t flight_flash_offset;
uint32_t current_flash_addr = 0;
const int FRAMES_PER_PAGE = FLASH_PAGE_SIZE / sizeof(LogFrame); 
LogFrame page_buffer[8]; // 256 bytes total
int buffer_index = 0;

// Shared volatile variables for inter-core communication
volatile FlightState current_state = STATE_BOOTING;
volatile float current_gforce = 0.0f;
volatile float current_accel_x = 0.0f;
volatile float current_gyro_mag = 0.0f;
volatile float current_altitude = 0.0f;
volatile float max_altitude = 0.0f;
volatile float reference_pressure_hpa = 1013.25f;
volatile uint32_t state_start_time = 0;

uint32_t pyro1_fire_start = 0;
uint32_t pyro2_fire_start = 0;
bool pyro1_active = false;
bool pyro2_active = false;

// Function Prototypes
void scanForAppendAddress();
void handleSerialCommands();

// ============================================================================
// [3] CORE 0: STATE MACHINE & SYSTEM MANAGEMENT
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    pinMode(PYRO1_PIN, OUTPUT);
    digitalWrite(PYRO1_PIN, LOW);
    pinMode(PYRO2_PIN, OUTPUT);
    digitalWrite(PYRO2_PIN, LOW);
    
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Blue;
    FastLED.show();

    // Allow time for USB enumeration
    uint32_t boot_timer = millis();
    while (!Serial && millis() - boot_timer < 5000) { delay(10); }

    Serial.println("\n--- RapidFish Avionics Initializing ---");

    flight_flash_offset = PICO_FLASH_SIZE_BYTES; 
    gpio_set_function(0, GPIO_FUNC_XIP_CS1);
    flash_devinfo_set_cs_size(1, flash_devinfo_bytes_to_size(FLIGHT_DATA_FLASH_SIZE));
    flash_devinfo_set_cs_gpio(1, 0);

    scanForAppendAddress();

    state_start_time = millis();
    current_state = STATE_ARMED;
    Serial.println("System Armed.");
}

void loop() {
    handleSerialCommands();
    uint32_t current_time = millis();

    if (pyro1_active && (current_time - pyro1_fire_start >= 3000)) {
        digitalWrite(PYRO1_PIN, LOW);
        pyro1_active = false;
    }
    if (pyro2_active && (current_time - pyro2_fire_start >= 3000)) {
        digitalWrite(PYRO2_PIN, LOW);
        pyro2_active = false;
    }

    // LED State Feedback
    static bool last_blink_state = false;
    static FlightState last_indicated_state = STATE_BOOTING;
    bool current_blink_state = ((current_time / 500) % 2 == 0);

    if (current_blink_state != last_blink_state || current_state != last_indicated_state) {
        if (current_blink_state) {
            switch(current_state) {
                case STATE_ARMED:        leds[0] = CRGB::White; break;
                case STATE_ACCELERATING: leds[0] = CRGB::Red;   break;
                case STATE_COAST:        leds[0] = CRGB::Blue;  break;
                case STATE_RECOVERY:     leds[0] = CRGB::Yellow; break;
                case STATE_CHUTE:        leds[0] = CRGB::Purple; break;
                case STATE_GROUND:       leds[0] = CRGB::Green;  break;
                default:                 leds[0] = CRGB::Black;  break;
            }
        } else {
            leds[0] = CRGB::Black;
        }
        FastLED.show();
        last_blink_state = current_blink_state;
        last_indicated_state = current_state;
    }

    // Flight Logic
    switch (current_state) {
        case STATE_ARMED: {
            static uint32_t launch_detect_start = 0;
            // Rely on positive X-axis jolt for launch detection. Requires fixed mounting orientation.
            if (current_accel_x > LAUNCH_G_THRESHOLD) {
                if (launch_detect_start == 0) launch_detect_start = current_time;
                else if (current_time - launch_detect_start > 50) {
                    current_state = STATE_ACCELERATING;
                    state_start_time = current_time;
                    Serial.println("Liftoff detected.");
                }
            } else {
                launch_detect_start = 0;
            }
            break;
        }

        case STATE_ACCELERATING:
            // Burnout means vertical thrust drops. High aerodynamic drag will result in negative X-axis acceleration.
            if ((current_time - state_start_time > MIN_MOTOR_BURN_MS) && 
                (current_accel_x < BURNOUT_G_THRESHOLD)) {
                current_state = STATE_COAST;
                state_start_time = current_time;
                Serial.println("Burnout detected.");
            }
            // Failsafe: Force coast if max expected motor burn time is exceeded (or imu has issues)
            else if (current_time - state_start_time > MAX_MOTOR_BURN_MS) {
                current_state = STATE_COAST;
                state_start_time = current_time;
                Serial.println("Burnout detected (Timeout Failsafe).");
            }
            break;

        case STATE_COAST:
            if (current_altitude < (max_altitude - APOGEE_DIP_METERS)) {
                current_state = STATE_RECOVERY;
                state_start_time = current_time;
                Serial.println("Apogee reached.");
            }
            break;

        case STATE_RECOVERY:
            if (current_time - state_start_time > RECOVERY_DELAY_MS) {
                current_state = STATE_CHUTE;
                state_start_time = current_time;
                Serial.println("Chute deployment initiated.");
            }
            break;

        case STATE_CHUTE: {
            // Tolerance increased to 0.4 rad/s (~23 deg/s) to clear the uncalibrated zero-offset noise floor
            const float GYRO_STILL_TOLERANCE = 0.4f; 
            const float GROUND_ALTITUDE_TOLERANCE = 50.0f; // Prevent mid-air ground triggers
            
            // If the airframe is rotating OR we are too high, it is still flying.
            if (current_gyro_mag > GYRO_STILL_TOLERANCE || current_altitude > GROUND_ALTITUDE_TOLERANCE) {
                state_start_time = current_time; // Reset wait timer
            } else if (current_time - state_start_time > GROUND_WAIT_MS) {
                current_state = STATE_GROUND;
                Serial.println("Flight complete.");
            }
            break;
        }

        case STATE_GROUND:
        case STATE_BOOTING:
            break;
    }
}

// ============================================================================
// [4] CORE 1: HIGH-FREQUENCY LOGGING & SENSOR ACQUISITION
// ============================================================================

void setup1() {
    delay(5000); 

    pinMode(LSM_CS_PIN, OUTPUT);
    digitalWrite(LSM_CS_PIN, HIGH);
    delay(10);

    if (!lsm.begin_SPI(LSM_CS_PIN, LSM_SCK_PIN, LSM_MISO_PIN, LSM_MOSI_PIN)) {
        while (1) { delay(100); }
    }

    lsm.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    Wire1.setSDA(I2C_SDA_PIN);
    Wire1.setSCL(I2C_SCL_PIN);
    Wire1.begin();
    Wire1.setClock(400000); 
    
    if (bmp.begin_I2C(0x76, &Wire1)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ); 
    }
}

void loop1() {
    static uint32_t last_sample_micros = 0;
    static int8_t cached_core_temp = 0;

    if (micros() - last_sample_micros >= 1000) {
        
        if (micros() - last_sample_micros > 10000) last_sample_micros = micros();
        else last_sample_micros += 1000; 

        // 1. IMU Sample
        sensors_event_t accel, gyro, temp;
        lsm.getEvent(&accel, &gyro, &temp);

        current_accel_x = accel.acceleration.x / 9.81f;
        current_gforce = sqrtf((accel.acceleration.x * accel.acceleration.x) + 
                               (accel.acceleration.y * accel.acceleration.y) + 
                               (accel.acceleration.z * accel.acceleration.z)) / 9.81f;
        
        current_gyro_mag = sqrtf((gyro.gyro.x * gyro.gyro.x) + 
                                 (gyro.gyro.y * gyro.gyro.y) + 
                                 (gyro.gyro.z * gyro.gyro.z));

        // 2. Baro Sample (Decimated)
        static uint8_t bmp_divider = 0;
        if (++bmp_divider >= 20) {
            bmp.performReading();
            cached_core_temp = (int8_t)analogReadTemp();
            bmp_divider = 0;
            
            if (current_state == STATE_BOOTING || current_state == STATE_ARMED) {
                float current_hpa = bmp.pressure / 100.0f;
                reference_pressure_hpa = (reference_pressure_hpa * 0.9f) + (current_hpa * 0.1f);
            }

            float alt = bmp.readAltitude(reference_pressure_hpa);
            if (current_altitude == 0.0f) current_altitude = alt;
            else current_altitude = (current_altitude * 0.8f) + (alt * 0.2f);
            
            if (current_altitude > max_altitude && current_state >= STATE_ACCELERATING) {
                max_altitude = current_altitude;
            }
        }

        // 3. Logger
        if (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE) {
            page_buffer[buffer_index].sync_word = SYNC_WORD;
            page_buffer[buffer_index].timestamp = millis();
            page_buffer[buffer_index].ax = (int16_t)(accel.acceleration.x * 100);
            page_buffer[buffer_index].ay = (int16_t)(accel.acceleration.y * 100);
            page_buffer[buffer_index].az = (int16_t)(accel.acceleration.z * 100);
            page_buffer[buffer_index].gx = (int16_t)(gyro.gyro.x * 1000);
            page_buffer[buffer_index].gy = (int16_t)(gyro.gyro.y * 1000);
            page_buffer[buffer_index].gz = (int16_t)(gyro.gyro.z * 1000);
            page_buffer[buffer_index].pressure = (uint32_t)bmp.pressure;
            page_buffer[buffer_index].flight_state = (uint8_t)current_state;
            page_buffer[buffer_index].temperature = (int8_t)bmp.temperature;
            page_buffer[buffer_index].core_temp = cached_core_temp;
            for (int i = 0; i < 5; i++) page_buffer[buffer_index]._pad[i] = 0;

            buffer_index++;

            if (buffer_index >= FRAMES_PER_PAGE) {
                if (current_flash_addr < FLIGHT_DATA_FLASH_SIZE) {
                    rp2040.idleOtherCore(); 
                    uint32_t ints = save_and_disable_interrupts();
                    flash_range_program(flight_flash_offset + current_flash_addr, (uint8_t*)page_buffer, FLASH_PAGE_SIZE);
                    restore_interrupts(ints);
                    rp2040.resumeOtherCore();
                    current_flash_addr += FLASH_PAGE_SIZE;
                }
                buffer_index = 0;
            }
        }
    }
}

// ============================================================================
// [5] UTILITY FUNCTIONS
// ============================================================================

void scanForAppendAddress() {
    current_flash_addr = 0;
    while (current_flash_addr < FLIGHT_DATA_FLASH_SIZE) {
        uint32_t* ptr = (uint32_t*)(XIP_BASE + flight_flash_offset + current_flash_addr);
        if (*ptr == 0xFFFFFFFF) break; 
        current_flash_addr += FLASH_PAGE_SIZE;
    }
    Serial.printf("Data append pointer at: 0x%08X\n", current_flash_addr);
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        bool flight_active = (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE);

        if (cmd == "WIPE_FLASH" && !flight_active) {
            Serial.println("Flash erase initialized...");
            leds[0] = CRGB::Red; FastLED.show();

            uint32_t total_sectors = FLIGHT_DATA_FLASH_SIZE / FLASH_SECTOR_SIZE;
            uint32_t sector_count = 0;

            for (uint32_t i = 0; i < FLIGHT_DATA_FLASH_SIZE; i += FLASH_SECTOR_SIZE) {
                rp2040.idleOtherCore(); 
                uint32_t ints = save_and_disable_interrupts();
                flash_range_erase(flight_flash_offset + i, FLASH_SECTOR_SIZE);
                restore_interrupts(ints);
                rp2040.resumeOtherCore();
                
                sector_count++;
                if (sector_count % 16 == 0 || i + FLASH_SECTOR_SIZE >= FLIGHT_DATA_FLASH_SIZE) {
                    Serial.printf("Progress: %d%%\n", (sector_count * 100) / total_sectors);
                }
                delay(5);
            }
            current_flash_addr = 0;
            Serial.println("Erase complete.");
        }
        else if (cmd == "DUMP_FLASH" && !flight_active) {
            Serial.println("DUMP_START");
            uint8_t* flash_ptr = (uint8_t *)(XIP_BASE + flight_flash_offset);
            for (uint32_t i = 0; i < current_flash_addr; i += 256) {
                Serial.write(flash_ptr + i, 256);
            }
            Serial.println("\nDUMP_END");
        }
        else if (cmd == "STATUS") {
            Serial.println("\n--- SYSTEM STATUS ---");
            Serial.printf("State: %d\n", current_state);
            Serial.printf("Altitude: %.2f m (Max: %.2f m)\n", current_altitude, max_altitude);
            Serial.printf("Accel X: %.2f G\n", current_accel_x);
            Serial.printf("Gyro Mag: %.2f rad/s\n", current_gyro_mag);
            Serial.printf("Flash Usage: %u / %u bytes\n", current_flash_addr, FLIGHT_DATA_FLASH_SIZE);
            Serial.println("---------------------");
        }
        else if (cmd == "SIM_LAUNCH" && !flight_active) {
            current_state = STATE_ACCELERATING;
            state_start_time = millis();
            Serial.println("SIMULATION: State -> ACCELERATING (Logging started)");
        }
        else if (cmd == "SIM_BURNOUT" && current_state == STATE_ACCELERATING) {
            current_state = STATE_COAST;
            state_start_time = millis();
            Serial.println("SIMULATION: State -> COAST");
        }
        else if (cmd == "SIM_APOGEE" && current_state == STATE_COAST) {
            current_state = STATE_RECOVERY;
            state_start_time = millis();
            Serial.println("SIMULATION: State -> RECOVERY");
        }
        else if (cmd == "P1_FIRE") {
            digitalWrite(PYRO1_PIN, HIGH);
            pyro1_fire_start = millis();
            pyro1_active = true;
            Serial.println("PYRO1 executing 3s burn");
        }
        else if (cmd == "P2_FIRE") {
            digitalWrite(PYRO2_PIN, HIGH);
            pyro2_fire_start = millis();
            pyro2_active = true;
            Serial.println("PYRO2 executing 3s burn");
        }
        else if (cmd == "RESET_ARMED") {
            current_state = STATE_ARMED;
            max_altitude = 0.0f;
            current_altitude = 0.0f;
            Serial.println("SYSTEM: State reset to ARMED");
        }
    }
}