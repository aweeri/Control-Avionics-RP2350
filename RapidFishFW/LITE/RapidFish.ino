#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <FastLED.h>
#include <Adafruit_LSM6DSOX.h>
#include "SparkFun_LSM6DSV16X.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <RadioLib.h>
#include <SPI.h>

/*
 * RapidFish - The CARP Rocket Avionics firmware (Single Flash 16MB)
 * - 0 to 2 MB: Quarantined for Firmware/Sketch
 * - 2 MB to 16 MB: Hard-clamped Flight Data Logging
 */

// ============================================================================
// [1] CONFIGURATION SECTION
// ============================================================================

// --- Hardware Pins (LITE board) ---
#define LED_PIN 25
#define NUM_LEDS 1
#define LED_BRIGHTNESS 255

#define PYRO1_PIN 21
#define PYRO2_PIN 22

#define LSM_CS_PIN  17
#define BMP_SDA_PIN 4
#define BMP_SCL_PIN 5

#define BAT_ADC_PIN   26
#define PYRO1_ADC_PIN 28
#define PYRO2_ADC_PIN 29

#define BUZZER_PIN 24

// --- Flight Physics Thresholds ---
const float LAUNCH_G_THRESHOLD   = 2.0f; 
const float BURNOUT_G_THRESHOLD  = 0.5f; 
const float APOGEE_DIP_METERS    = 6.0f;
const float GROUND_G_TOLERANCE   = 0.2f;  
const bool  IS_UPSIDE_DOWN       = false; 

// --- Timing Thresholds (Milliseconds) ---
const uint32_t MIN_MOTOR_BURN_MS = 500;   
const uint32_t MAX_MOTOR_BURN_MS = 3000;  
const uint32_t RECOVERY_DELAY_MS = 0;     
const uint32_t GROUND_WAIT_MS    = 5000;  

// --- Pyro Deployment Thresholds ---
const uint8_t  PYRO_CONTINUITY_THRESHOLD  = 10;    
const uint32_t PYRO_REDEPLOY_TIMEOUT_MS   = 3000;  
const float    CHUTE_DESCENT_RATE_THRESHOLD = 5.0f;

// --- Flash Storage Constraints (Single 16MB Chip) ---
const uint32_t FIRMWARE_RESERVED_SIZE = 2 * 1024 * 1024;  // 2MB safe zone
const uint32_t FLIGHT_DATA_FLASH_SIZE = 14 * 1024 * 1024; // 14MB log capacity
const uint32_t SYNC_WORD_APID0 = 0x1ACFFC1D;              
const uint32_t SYNC_WORD_APID1 = 0x5ACFFC1D;              

// --- Radio Telemetry (LR2021 GFSK on SPI1) ---
const bool RADIO_ENABLED = true; 
#define RADIO_CS_PIN    13
#define RADIO_IRQ_PIN   6    
#define RADIO_RST_PIN   11
#define RADIO_BUSY_PIN  10

#define RADIO_FREQUENCY_MHZ 434.0f
#define RADIO_TX_INTERVAL_MS       100
#define RADIO_BEACON_INTERVAL_MS   1000

// ============================================================================
// [2] DATA STRUCTURES & GLOBALS
// ============================================================================

enum FlightState {
    STATE_BOOTING,
    STATE_ERROR,
    STATE_ARMED,
    STATE_ACCELERATING,
    STATE_COAST,
    STATE_RECOVERY,
    STATE_CHUTE,
    STATE_GROUND
};

// --- APID 0: Core Sensor Frame (32 bytes) ---
struct __attribute__((packed)) LogFrameCore {
    uint32_t sync_word;       // +0  (4)
    uint32_t timestamp;       // +4  (4)
    uint8_t  apid;            // +8  (1)
    uint8_t  flight_state;    // +9  (1)
    uint8_t  flash_used;      // +10 (1)
    int8_t   core_temp;       // +11 (1)
    int16_t  ax, ay, az;      // +12 (6)
    int16_t  gx, gy, gz;      // +18 (6)
    uint16_t altitude;        // +24 (2)
    int8_t   temperature;     // +26 (1)
    uint8_t  bat_voltage;     // +27 (1)
    uint8_t  p1_voltage;      // +28 (1)
    uint8_t  p2_voltage;      // +29 (1)
    uint8_t  _pad[2];         // +30 (2)
};
static_assert(sizeof(LogFrameCore) == 32, "LogFrameCore must be 32 bytes");

struct __attribute__((packed)) LogFrameGPS {
    uint32_t sync_word;       
    uint32_t timestamp;       
    uint8_t  apid;            
    uint32_t lat;             
    uint32_t lon;             
    uint16_t gps_alt;         
    uint8_t  state;           
    uint8_t  sats;            
    int16_t  mx, my, mz;      
    uint8_t  _pad[5];         
};
static_assert(sizeof(LogFrameGPS) == 32, "LogFrameGPS must be 32 bytes");

union LogFrame {
    LogFrameCore core;
    LogFrameGPS  gps;
    uint8_t      bytes[32];
};
static_assert(sizeof(union LogFrame) == 32, "LogFrame union must be 32 bytes");

CRGB leds[NUM_LEDS];
Adafruit_LSM6DSOX lsm_dsox;
SparkFun_LSM6DSV16X_SPI lsm_dsv;
bool use_dsox = false;
Adafruit_BMP3XX bmp;

LR2021 radio = new Module(RADIO_CS_PIN, RADIO_IRQ_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI1);
bool radio_ready = false;
uint32_t last_radio_tx = 0;

// Flash memory state tracking
uint32_t flight_flash_offset = FIRMWARE_RESERVED_SIZE; 
uint32_t current_flash_addr = 0;
const int FRAMES_PER_PAGE = FLASH_PAGE_SIZE / sizeof(union LogFrame); 
union LogFrame page_buffer[8]; 
int buffer_index = 0;

// Shared volatile variables
volatile FlightState current_state = STATE_BOOTING;
volatile float current_gforce = 0.0f;
volatile float current_accel_x = 0.0f;
volatile float current_accel_y = 0.0f;
volatile float current_accel_z = 0.0f;
volatile float current_gyro_x = 0.0f;
volatile float current_gyro_y = 0.0f;
volatile float current_gyro_z = 0.0f;
volatile float current_gyro_mag = 0.0f;
volatile float current_altitude = 0.0f;
volatile float max_altitude = 0.0f;
volatile float reference_pressure_hpa = 1013.25f;
volatile uint32_t state_start_time = 0;
volatile uint8_t system_errors = 0; 
volatile bool core1_init_complete = false;
volatile int radio_error_code = 0;

uint32_t pyro1_fire_start = 0;
uint32_t pyro2_fire_start = 0;
bool pyro1_active = false;
bool pyro2_active = false;
bool pyro1_fired = false;   
bool pyro2_fired = false;   
volatile float current_descent_rate = 0.0f; 
volatile uint8_t current_bat_voltage = 0;
volatile uint8_t current_p1_voltage = 0;
volatile uint8_t current_p2_voltage = 0;
volatile int8_t current_core_temp = 0;
volatile int8_t current_baro_temp = 0;

enum BuzzerPattern {
    BUZZER_IDLE, BUZZER_ERROR, BUZZER_BOOT_BEEPS, BUZZER_LIFTOFF_SPAM, BUZZER_SOS
};

struct BuzzerState {
    BuzzerPattern pattern;
    uint32_t      step_start;
    uint8_t       step_index;
    bool          on_state;
} buzzer = { BUZZER_IDLE, 0, 0, false };

void scanForAppendAddress();
void handleSerialCommands();
void updateBuzzer();
void startBuzzerPattern(BuzzerPattern p);
void radioInit();
void radioTransmitFrame();
void radioSetFrequency(float mhz);

// ============================================================================
// [3] CORE 0: STATE MACHINE & SYSTEM MANAGEMENT
// ============================================================================

void setup() {
    Serial.begin(115200);
    
    pinMode(PYRO1_PIN, OUTPUT);
    digitalWrite(PYRO1_PIN, LOW);
    pinMode(PYRO2_PIN, OUTPUT);
    digitalWrite(PYRO2_PIN, LOW);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Blue;
    FastLED.show();

    analogReadResolution(12);

    uint32_t boot_timer = millis();
    while (!Serial && millis() - boot_timer < 5000) { delay(10); }

    Serial.println("\n--- RapidFish Avionics (Single Flash) Initializing ---");

    scanForAppendAddress();
    radioInit();

    Serial.println("Waiting for sensors to initialize...");
    while (!core1_init_complete) { delay(10); }

    if (system_errors > 0) {
        current_state = STATE_ERROR;
        Serial.println("\n*** BOOT FAILURE: SENSOR/RADIO ERROR ***");
        if (system_errors & 1) Serial.println("- IMU (LSM6DSOX/LSM6DSV16X) failed to initialize.");
        if (system_errors & 2) Serial.println("- Barometer (BMP390) failed to initialize.");
        if (system_errors & 4) Serial.printf("- Radio (LR2021) failed to initialize. Code: %d\n", radio_error_code);
        Serial.println("System halted in STATE_ERROR. Waiting for reboot.");
        startBuzzerPattern(BUZZER_ERROR);
    } else {
        pyro1_fired = false;
        pyro2_fired = false;
        state_start_time = millis();
        current_state = STATE_ARMED;
        Serial.println("System Armed.");
        startBuzzerPattern(BUZZER_BOOT_BEEPS);
    }
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

    static bool last_blink_state = false;
    static FlightState last_indicated_state = STATE_BOOTING;
    bool current_blink_state = false;
    
    if (current_state == STATE_ERROR) {
        current_blink_state = ((current_time / 100) % 2 == 0); 
    } else {
        current_blink_state = ((current_time / 500) % 2 == 0); 
    }

    if (current_blink_state != last_blink_state || current_state != last_indicated_state) {
        if (current_blink_state) {
            switch(current_state) {
                case STATE_ERROR:        leds[0] = CRGB::Red;    break;
                case STATE_ARMED:        leds[0] = CRGB::White;  break;
                case STATE_ACCELERATING: leds[0] = CRGB::Red;    break;
                case STATE_COAST:        leds[0] = CRGB::Blue;   break;
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

    updateBuzzer();

    if (radio_ready) {
        bool flight_active = (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE);
        uint32_t interval = flight_active ? RADIO_TX_INTERVAL_MS : RADIO_BEACON_INTERVAL_MS;
        if (current_time - last_radio_tx >= interval) {
            last_radio_tx = current_time;
            radioTransmitFrame();
        }
    }

    switch (current_state) {
        case STATE_ERROR: break;
            
        case STATE_ARMED: {
            static uint32_t launch_detect_start = 0;
            if (current_accel_x > LAUNCH_G_THRESHOLD) {
                if (launch_detect_start == 0) launch_detect_start = current_time;
                else if (current_time - launch_detect_start > 50) {
                    current_state = STATE_ACCELERATING;
                    state_start_time = current_time;
                    Serial.println("Liftoff detected.");
                    startBuzzerPattern(BUZZER_LIFTOFF_SPAM);
                }
            } else {
                launch_detect_start = 0;
            }
            break;
        }

        case STATE_ACCELERATING:
            if ((current_time - state_start_time > MIN_MOTOR_BURN_MS) && 
                (current_accel_x < BURNOUT_G_THRESHOLD)) {
                current_state = STATE_COAST;
                state_start_time = current_time;
                Serial.println("Burnout detected.");
            }
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

        case STATE_RECOVERY: {
            bool p1_has_cont = (current_p1_voltage >= PYRO_CONTINUITY_THRESHOLD);
            bool p2_has_cont = (current_p2_voltage >= PYRO_CONTINUITY_THRESHOLD);
            bool both_available = (p1_has_cont && p2_has_cont);
            bool one_available  = (p1_has_cont || p2_has_cont);

            if (!pyro1_fired && !pyro2_fired && one_available) {
                if (p1_has_cont) {
                    digitalWrite(PYRO1_PIN, HIGH);
                    pyro1_fire_start = current_time;
                    pyro1_active = true;
                    pyro1_fired = true;
                    Serial.println("PYRO1 fired (primary deployment).");
                } else {
                    digitalWrite(PYRO2_PIN, HIGH);
                    pyro2_fire_start = current_time;
                    pyro2_active = true;
                    pyro2_fired = true;
                    Serial.println("PYRO2 fired (primary deployment, P1 no continuity).");
                }
                state_start_time = current_time; 
                break;
            }

            if ((pyro1_fired || pyro2_fired) &&
                (current_time - state_start_time > PYRO_REDEPLOY_TIMEOUT_MS)) {
                
                if (both_available && !pyro2_fired) {
                    digitalWrite(PYRO2_PIN, HIGH);
                    pyro2_fire_start = current_time;
                    pyro2_active = true;
                    pyro2_fired = true;
                    Serial.println("PYRO2 fired (backup deployment, no chute detected).");
                    state_start_time = current_time;
                    break;
                } else if (!both_available && !pyro1_fired && !pyro2_fired) {
                    Serial.println("WARN: No backup pyro available, awaiting chute/ground passively.");
                }
            }

            if (current_time - state_start_time > RECOVERY_DELAY_MS) {
                current_state = STATE_CHUTE;
                state_start_time = current_time;
                Serial.println("State -> CHUTE (awaiting chute detection).");
            }
            break;
        }

        case STATE_CHUTE: {
            const float GYRO_STILL_TOLERANCE = 0.4f;
            const float GROUND_ALTITUDE_TOLERANCE = 50.0f; 
            
            bool chute_deployed = (current_descent_rate < CHUTE_DESCENT_RATE_THRESHOLD);
            bool gyro_still     = (current_gyro_mag <= GYRO_STILL_TOLERANCE);
            bool low_altitude   = (current_altitude <= GROUND_ALTITUDE_TOLERANCE);

            if (chute_deployed && gyro_still && low_altitude) {
                if (current_time - state_start_time > GROUND_WAIT_MS) {
                    current_state = STATE_GROUND;
                    Serial.println("Flight complete (chute descent confirmed).");
                }
            } else {
                state_start_time = current_time;
            }
            break;
        }

        case STATE_GROUND:
            if (buzzer.pattern != BUZZER_SOS) {
                startBuzzerPattern(BUZZER_SOS);
            }
            break;
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
    
    // Explicitly route SPI0 to the designated pins for the IMU
    // This prevents failures on generic RP2040/RP2350 target profiles
    SPI.setRX(16);
    SPI.setSCK(18);
    SPI.setTX(19);
    SPI.begin();

    // Toggle CS low then high to ensure the IMU switches from I2C to SPI mode
    digitalWrite(LSM_CS_PIN, LOW);
    delay(5);
    digitalWrite(LSM_CS_PIN, HIGH);
    delay(5);

    if (lsm_dsox.begin_SPI(LSM_CS_PIN)) {
        use_dsox = true;
        lsm_dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
        lsm_dsox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
        lsm_dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
        lsm_dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    } 
    else {
        // Manually pull CS back up in case the Adafruit library left it dangling after a failed WHOAMI check
        digitalWrite(LSM_CS_PIN, HIGH);
        delay(10);
        
        if (lsm_dsv.begin(LSM_CS_PIN)) {
            use_dsox = false;
            lsm_dsv.deviceReset();
            while (!lsm_dsv.getDeviceReset()) {
                delay(1);
            }
            lsm_dsv.enableBlockDataUpdate();
            lsm_dsv.setAccelDataRate(LSM6DSV16X_ODR_AT_1920Hz);
            lsm_dsv.setAccelFullScale(LSM6DSV16X_16g);
            lsm_dsv.setGyroDataRate(LSM6DSV16X_ODR_AT_1920Hz);
            lsm_dsv.setGyroFullScale(LSM6DSV16X_2000dps);
        } 
        else {
            system_errors |= 1; 
        }
    }

    Wire.setSDA(BMP_SDA_PIN);
    Wire.setSCL(BMP_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
    
    if (!bmp.begin_I2C(0x76, &Wire)) {
        system_errors |= 2; 
    } else {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ); 
    }

    core1_init_complete = true; 
}

void loop1() {
    static uint32_t last_sample_micros = 0;
    static int8_t cached_core_temp = 0;

    if (system_errors > 0) return;

    if (micros() - last_sample_micros >= 1000) {
         
        if (micros() - last_sample_micros > 10000) last_sample_micros = micros();
        else last_sample_micros += 1000; 
        
        float ax_val = 0.0f;
        float ay_val = 0.0f;
        float az_val = 0.0f;
        float gx_val = 0.0f;
        float gy_val = 0.0f;
        float gz_val = 0.0f;

        if (use_dsox) {
            sensors_event_t accel, gyro, temp;
            lsm_dsox.getEvent(&accel, &gyro, &temp);
            
            ax_val = accel.acceleration.x; 
            ay_val = accel.acceleration.y;
            az_val = accel.acceleration.z;
            
            gx_val = gyro.gyro.x;
            gy_val = gyro.gyro.y;
            gz_val = gyro.gyro.z;
        } else {
            sfe_lsm_data_t accelData;
            sfe_lsm_data_t gyroData;

            if (lsm_dsv.checkStatus()) {
                lsm_dsv.getAccel(&accelData);
                lsm_dsv.getGyro(&gyroData);

                ax_val = (accelData.xData / 1000.0f) * 9.81f; 
                ay_val = (accelData.yData / 1000.0f) * 9.81f;
                az_val = (accelData.zData / 1000.0f) * 9.81f;

                gx_val = (gyroData.xData / 1000.0f) * 0.0174533f; 
                gy_val = (gyroData.yData / 1000.0f) * 0.0174533f;
                gz_val = (gyroData.zData / 1000.0f) * 0.0174533f;
            }
        }

        float x_mod = IS_UPSIDE_DOWN ? 1.0f : -1.0f;

        current_accel_x = (ax_val * x_mod) / 9.81f;
        current_accel_y = ay_val / 9.81f;
        current_accel_z = az_val / 9.81f;
        current_gforce = sqrtf((ax_val * ax_val) +
                               (ay_val * ay_val) +
                               (az_val * az_val)) / 9.81f;
        
        current_gyro_x = gx_val * x_mod;
        current_gyro_y = gy_val;
        current_gyro_z = gz_val;
        current_gyro_mag = sqrtf((gx_val * gx_val) +
                                 (gy_val * gy_val) +
                                 (gz_val * gz_val));

        static uint8_t decimator = 0;
        if (++decimator >= 20) {
            bmp.performReading();
            cached_core_temp = (int8_t)analogReadTemp();
            current_core_temp = cached_core_temp;
            current_baro_temp = (int8_t)bmp.temperature;
            decimator = 0;
            
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

            {
                static float prev_altitude = 0.0f;
                float delta = prev_altitude - current_altitude; 
                current_descent_rate = (current_descent_rate * 0.7f) + (delta * 50.0f * 0.3f);
                prev_altitude = current_altitude;
            }

            current_bat_voltage = (uint8_t)(analogRead(BAT_ADC_PIN) >> 4);
            current_p1_voltage  = (uint8_t)(analogRead(PYRO1_ADC_PIN) >> 4);
            current_p2_voltage  = (uint8_t)(analogRead(PYRO2_ADC_PIN) >> 4);
        }

        if (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE) {
            LogFrameCore* f = &page_buffer[buffer_index].core;
            
            f->sync_word    = SYNC_WORD_APID0;
            f->timestamp    = millis();
            f->apid         = 0;  
            f->flight_state = (uint8_t)current_state;
            f->flash_used   = (uint8_t)((current_flash_addr * 255) / FLIGHT_DATA_FLASH_SIZE);
            f->core_temp    = cached_core_temp;
            f->ax           = (int16_t)(ax_val * 100);
            f->ay           = (int16_t)(ay_val * 100);
            f->az           = (int16_t)(az_val * 100);
            f->gx           = (int16_t)(gx_val * 1000);
            f->gy           = (int16_t)(gy_val * 1000);
            f->gz           = (int16_t)(gz_val * 1000);
            f->altitude     = (uint16_t)(current_altitude * 2.0f);
            f->temperature  = (int8_t)bmp.temperature;
            f->bat_voltage  = current_bat_voltage;
            f->p1_voltage   = current_p1_voltage;
            f->p2_voltage   = current_p2_voltage;

            buffer_index++;

            if (buffer_index >= FRAMES_PER_PAGE) {
                // HARD CLAMP: Drop data entirely if we hit the 14MB log boundary
                if (current_flash_addr + FLASH_PAGE_SIZE <= FLIGHT_DATA_FLASH_SIZE) {
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
    Serial.printf("Data append pointer at: 0x%08X (Absolute: 0x%08X)\n", 
                  current_flash_addr, flight_flash_offset + current_flash_addr);
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        bool flight_active = (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE);

        if (cmd.startsWith("RADIO_FREQ")) {
            float freq = cmd.substring(10).toFloat();
            if (freq >= 400.0f && freq <= 510.0f) {
                radioSetFrequency(freq);
            } else {
                Serial.println("RADIO_FREQ: out of range (400-510 MHz)");
            }
        }
        else if (cmd == "RADIO_TEST") {
            if (radio_ready) {
                Serial.println("RADIO_TEST: transmitting test frame...");
                radioTransmitFrame();
            } else {
                Serial.println("RADIO_TEST: radio not ready");
            }
        }
        else if (cmd == "WIPE_FLASH" && !flight_active) {
            Serial.println("Flash erase initialized. Quarantined 2MB boot partition protected.");
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
        // THIS DUMP INTERFACES CLOSELY WITH TOOL.PY. CHANGING ANYTHING REQUIRES RE-VERIFICATION OF TOOL.PY.
        else if (cmd == "DUMP_FLASH" && !flight_active) {
            Serial.println("DUMP_START");
            uint8_t* flash_ptr = (uint8_t *)(XIP_BASE + flight_flash_offset);
            for (uint32_t i = 0; i < current_flash_addr; i += 256) {
                Serial.write(flash_ptr + i, 256);
            }
            Serial.println("\nDUMP_END");
        }
        else if (cmd == "STATUS") {
            const char* state_names[] = {"BOOTING", "ERROR", "ARMED", "ACCELERATING", "COAST", "RECOVERY", "CHUTE", "GROUND"};
            float bat_v = (current_bat_voltage * 9.9f) / 255.0f;
            float p1_v  = (current_p1_voltage * 9.9f) / 255.0f;
            float p2_v  = (current_p2_voltage * 9.9f) / 255.0f;

            Serial.println("\n=================================");
            Serial.println("       SYSTEM STATUS REPORT      ");
            Serial.println("=================================");
            Serial.printf("Flight State : %s\n", state_names[current_state]);
            Serial.printf("Uptime       : %lu ms\n", millis());
            Serial.printf("Core Temp    : %d C\n", current_core_temp);
            Serial.printf("Baro Temp    : %d C\n", current_baro_temp);
            Serial.println("---------------------------------");
            Serial.printf("Altitude     : %.2f m (Max: %.2f m)\n", current_altitude, max_altitude);
            Serial.printf("Descent Rate : %.2f m/s\n", current_descent_rate);
            Serial.printf("Accel Vector : X:%.2f G | Y:%.2f G | Z:%.2f G\n", current_accel_x, current_accel_y, current_accel_z);
            Serial.printf("Gyro Mag     : %.2f rad/s\n", current_gyro_mag);
            Serial.println("---------------------------------");
            Serial.printf("Main Battery : %.2f V\n", bat_v);
            Serial.printf("Pyro1 Voltage: %.2f V\n", p1_v);
            Serial.printf("Pyro2 Voltage: %.2f V\n", p2_v);
            Serial.println("---------------------------------");
            Serial.printf("Flash Usage  : %u / %u bytes (%u%%)\n",
                          current_flash_addr, FLIGHT_DATA_FLASH_SIZE,
                          (current_flash_addr * 100) / FLIGHT_DATA_FLASH_SIZE);
            Serial.println("=================================");
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
            pyro1_fired = false;
            pyro2_fired = false;
            Serial.println("SYSTEM: State reset to ARMED");
        }
        else if (cmd == "BUZZER_ON") {
            digitalWrite(BUZZER_PIN, HIGH);
            Serial.println("BUZZER ON");
        }
        else if (cmd == "BUZZER_OFF") {
            digitalWrite(BUZZER_PIN, LOW);
            Serial.println("BUZZER OFF");
        }
        else if (cmd == "BUZZER_TEST") {
            Serial.println("BUZZER TEST EXECUTING");
            startBuzzerPattern(BUZZER_BOOT_BEEPS);
        }
    }
}

// ============================================================================
// [5b] RADIO TELEMETRY (LR2021 GFSK on SPI1, core 0)
// ============================================================================

void radioInit() {
    if (!RADIO_ENABLED) {
        Serial.println(F("[LR2021] Radio disabled via configuration. Skipping init."));
        return;
    }

    SPI1.setRX(12);
    SPI1.setSCK(14);
    SPI1.setTX(15);
    SPI1.begin();

    radio.tcxoVoltage = 2.7;
    radio.irqDioNum = 5;

    Serial.print(F("[LR2021] Initializing ... "));
    ConfigFSK_t config;
    config.frequency = RADIO_FREQUENCY_MHZ;
    int state = radio.beginGFSK(config);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("failed, code "));
        Serial.println(state);
        radio_ready = false;
        radio_error_code = state;
        system_errors |= 4; 
        return;
    }
    Serial.println(F("success!"));

    radio.setFrequency(RADIO_FREQUENCY_MHZ);
    radio.setBitRate(32.0);
    radio.setFrequencyDeviation(16.0);
    radio.setRxBandwidth(250.0);
    radio.setOutputPower(10.0);
    radio.setDataShaping(RADIOLIB_SHAPING_1_0);
    radio.setPreambleLength(64);
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);

    uint8_t syncWord[] = {0x1A, 0xCF, 0xFC, 0x1D};
    radio.setSyncWord(syncWord, 4);
    radio.setCRC(0);

    radio_ready = true;
    Serial.printf("[LR2021] Radio ready @ %.1f MHz\n", RADIO_FREQUENCY_MHZ);
}

void radioTransmitFrame() {
    LogFrameCore f;
    memset(&f, 0, sizeof(f));

    f.sync_word    = SYNC_WORD_APID0;
    f.timestamp    = millis();
    f.apid         = 0;
    f.flight_state = (uint8_t)current_state;
    f.flash_used   = (uint8_t)((current_flash_addr * 255) / FLIGHT_DATA_FLASH_SIZE);
    f.core_temp    = current_core_temp;
    f.ax           = (int16_t)(current_accel_x * 9.81f * 100.0f);
    f.ay           = (int16_t)(current_accel_y * 9.81f * 100.0f);
    f.az           = (int16_t)(current_accel_z * 9.81f * 100.0f);
    f.gx           = (int16_t)(current_gyro_x * 1000.0f);
    f.gy           = (int16_t)(current_gyro_y * 1000.0f);
    f.gz           = (int16_t)(current_gyro_z * 1000.0f);
    f.altitude     = (uint16_t)(current_altitude * 2.0f);
    f.temperature  = current_baro_temp;
    f.bat_voltage  = current_bat_voltage;
    f.p1_voltage   = current_p1_voltage;
    f.p2_voltage   = current_p2_voltage;

    int state = radio.transmit((uint8_t*)&f, sizeof(LogFrameCore), 100);
    if (state != RADIOLIB_ERR_NONE) {
        static uint32_t last_err_print = 0;
        if (millis() - last_err_print > 1000) {
            last_err_print = millis();
            Serial.printf("[LR2021] TX failed, code %d\n", state);
        }
    }
}

void radioSetFrequency(float mhz) {
    if (!radio_ready) return;
    int state = radio.setFrequency(mhz);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("[LR2021] Frequency set to %.3f MHz\n", mhz);
    } else {
        Serial.printf("[LR2021] setFrequency failed, code %d\n", state);
    }
}

// ============================================================================
// [6] BUZZER PATTERN GENERATOR (non-blocking)
// ============================================================================

void startBuzzerPattern(BuzzerPattern p) {
    buzzer.pattern    = p;
    buzzer.step_start = millis();
    buzzer.step_index = 0;
    buzzer.on_state   = false;
    digitalWrite(BUZZER_PIN, LOW);
}

void updateBuzzer() {
    if (buzzer.pattern == BUZZER_IDLE) return;

    uint32_t now = millis();
    uint32_t elapsed = now - buzzer.step_start;

    switch (buzzer.pattern) {
        case BUZZER_ERROR: {
            uint32_t cycle = elapsed % 100;
            digitalWrite(BUZZER_PIN, (cycle < 50) ? HIGH : LOW);
            break;
        }

        case BUZZER_BOOT_BEEPS: {
            const uint32_t CYCLE_MS = 200; 
            const uint8_t  TOTAL_CYCLES = 6; 
            uint8_t cycle = elapsed / CYCLE_MS;
            if (cycle >= TOTAL_CYCLES) {
                buzzer.pattern = BUZZER_IDLE;
                digitalWrite(BUZZER_PIN, LOW);
                return;
            }
            digitalWrite(BUZZER_PIN, (cycle % 2 == 0) ? HIGH : LOW);
            break;
        }

        case BUZZER_LIFTOFF_SPAM: {
            const uint32_t CYCLE_MS = 100; 
            const uint8_t  TOTAL_CYCLES = 6; 
            uint8_t cycle = elapsed / CYCLE_MS;
            if (cycle >= TOTAL_CYCLES) {
                buzzer.pattern = BUZZER_IDLE;
                digitalWrite(BUZZER_PIN, LOW);
                return;
            }
            digitalWrite(BUZZER_PIN, (cycle % 2 == 0) ? HIGH : LOW);
            break;
        }

        case BUZZER_SOS: {
            static const uint16_t sos_pattern[] = {
                60, 60, 60, 60, 60, 120,   
                180, 60, 180, 60, 180, 120, 
                60, 60, 60, 60, 60, 400    
            };
            const uint8_t NUM_STEPS = sizeof(sos_pattern) / sizeof(sos_pattern[0]);
            
            uint32_t accum = 0;
            uint8_t step = 0;
            while (step < NUM_STEPS && accum + sos_pattern[step] <= elapsed) {
                accum += sos_pattern[step];
                step++;
            }
            
            if (step >= NUM_STEPS) {
                buzzer.step_start = now;
                digitalWrite(BUZZER_PIN, LOW);
                return;
            }
            
            digitalWrite(BUZZER_PIN, (step % 2 == 0) ? HIGH : LOW);
            break;
        }

        default:
            buzzer.pattern = BUZZER_IDLE;
            digitalWrite(BUZZER_PIN, LOW);
            break;
    }
}
