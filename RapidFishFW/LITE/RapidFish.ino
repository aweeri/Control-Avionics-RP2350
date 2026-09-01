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
 * RapidFish - The CARP Rocket Avionics firmware (LITE board variant)
 * Core functionality: High-frequency sensor data acquisition, flight state estimation, 
 * and persistent flash storage for telemetry logging.
 *
 * LITE board pinout (rev 2.0):
 *   IMU  = SPI0  (GPIO 16-19, hardware SPI)
 *   BMP  = I2C0  (GPIO 4-5, Wire)
 *   I2C1 = GPIO 2-3 (user expansion, uninitialized)
 *   GNSS = UART1 (GPIO 8-9, reserved)
 *   Radio= SPI1  (GPIO 12-15, reserved)
 *   LED  = GPIO 25 (WS2812B)
 *   Pyro = GPIO 21, 22
 *   Buzzer= GPIO 24
 *   ADC  = GPIO 26 (batt), 28 (pyro1 cont), 29 (pyro2 cont)
 *   Flash CS = GPIO 0
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

// IMU on SPI0 — hardware pins (CS=17, SCK=18, MOSI=19, MISO=16)
#define LSM_CS_PIN  17

// BMP390 on I2C0 (Wire)
#define BMP_SDA_PIN 4
#define BMP_SCL_PIN 5

// I2C1 is on GPIO 2/3 — left uninitialized for user expansion

// ADC channels (all use 200k+100k voltage divider = 1/3 ratio)
// 2S battery: 6.0-8.4V → ADC pin 2.0-2.8V → 12-bit 2482-3475 → 8-bit 155-217
#define BAT_ADC_PIN   26
#define PYRO1_ADC_PIN 28
#define PYRO2_ADC_PIN 29

// Buzzer
#define BUZZER_PIN 24

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

// --- Pyro Deployment Thresholds ---
const uint8_t  PYRO_CONTINUITY_THRESHOLD  = 10;    // ADC value below which pyro is considered open-circuit
const uint32_t PYRO_REDEPLOY_TIMEOUT_MS   = 3000;  // Wait after first pyro fire before firing backup
const float    CHUTE_DESCENT_RATE_THRESHOLD = 5.0f; // m/s — below this indicates chute has deployed

// --- Flash Storage ---
const uint32_t FLIGHT_DATA_FLASH_SIZE = 2 * 1024 * 1024; // 2MB allocation on secondary flash
const uint32_t SYNC_WORD = 0x1ACFFC1D;              // 4 byte SyncWord

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

// --- APID 0: Core Sensor Frame (32 bytes) ---
// Logged at 1 kHz during flight
struct __attribute__((packed)) LogFrameCore {
    uint32_t sync_word;       // +0  (4)  APID 0 identifier
    uint32_t timestamp;       // +4  (4)  millis()
    uint8_t  apid;            // +8  (1)  Application Process ID (0=core)
    uint8_t  flight_state;    // +9  (1)  Current state machine stage
    uint8_t  flash_used;      // +10 (1)  0-255 = 0-100% flash usage
    int8_t   core_temp;       // +11 (1)  RP2350 internal temp °C
    int16_t  ax, ay, az;      // +12 (6)  Accel: (m/s²) * 100
    int16_t  gx, gy, gz;      // +18 (6)  Gyro: (rad/s) * 1000
    uint16_t altitude;        // +24 (2)  Baro: meters MSL * 2 (0-131070m)
    int8_t   temperature;     // +26 (1)  Barometer temp °C
    uint8_t  bat_voltage;     // +27 (1)  Battery ADC (0-255 scaled)
    uint8_t  p1_voltage;      // +28 (1)  Pyro 1 continuity ADC
    uint8_t  p2_voltage;      // +29 (1)  Pyro 2 continuity ADC
    uint8_t  _pad[2];         // +30 (2)  Padding to 32 bytes
};
static_assert(sizeof(LogFrameCore) == 32, "LogFrameCore must be 32 bytes");

// --- APID 1: GPS / Magneto Frame (32 bytes) ---
// Logged at low rate (e.g. 10 Hz) during flight, if GPS and/or magnetometer are available
struct __attribute__((packed)) LogFrameGPS {
    uint32_t sync_word;       // +0  (4)  Standard SyncWord (0x1ACFFC1D)
    uint32_t timestamp;       // +4  (4)  millis()
    uint8_t  apid;            // +8  (1)  Application Process ID (1=gps)
    uint32_t lat;             // +9  (4)  Latitude  * 1e7
    uint32_t lon;             // +13 (4)  Longitude * 1e7
    uint16_t gps_alt;         // +17 (2)  GPS altitude (meters)
    uint8_t  state;           // +19 (1)  GPS fix state
    uint8_t  sats;            // +20 (1)  Satellite count
    uint32_t gps_time;        // +21 (4)  Unix timestamp (seconds since epoch)
    uint8_t  hdop;            // +25 (1)  Horizontal dilution of precision * 10
    int16_t  mx, my, mz;      // +26 (6)  Magnetometer raw
};
static_assert(sizeof(LogFrameGPS) == 32, "LogFrameGPS must be 32 bytes");

// Union for type-safe page buffer access
union LogFrame {
    LogFrameCore core;
    LogFrameGPS  gps;
    uint8_t      bytes[32];
};
static_assert(sizeof(union LogFrame) == 32, "LogFrame union must be 32 bytes");

CRGB leds[NUM_LEDS];
Adafruit_LSM6DSOX lsm;
Adafruit_BMP3XX bmp;

// Flash memory state tracking
uint32_t flight_flash_offset;
uint32_t current_flash_addr = 0;
const int FRAMES_PER_PAGE = FLASH_PAGE_SIZE / sizeof(union LogFrame); // 256/32 = 8
union LogFrame page_buffer[8]; // 256 bytes total
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
bool pyro1_fired = false;   // Tracks whether pyro 1 has been fired this flight
bool pyro2_fired = false;   // Tracks whether pyro 2 has been fired this flight

// Descent rate tracking (computed in loop0 from baro altitude)
volatile float current_descent_rate = 0.0f; // m/s, positive = descending

// ADC sample storage (updated in loop1, read in loop0 for STATUS)
volatile uint8_t current_bat_voltage = 0;
volatile uint8_t current_p1_voltage = 0;
volatile uint8_t current_p2_voltage = 0;

// Buzzer state machine
enum BuzzerPattern {
    BUZZER_IDLE,
    BUZZER_BOOT_BEEPS,
    BUZZER_LIFTOFF_SPAM,
    BUZZER_SOS
};

struct BuzzerState {
    BuzzerPattern pattern;
    uint32_t      step_start;
    uint8_t       step_index;
    bool          on_state;
} buzzer = { BUZZER_IDLE, 0, 0, false };

// Function Prototypes
void scanForAppendAddress();
void handleSerialCommands();
void updateBuzzer();
void startBuzzerPattern(BuzzerPattern p);

// ============================================================================
// [3] CORE 0: STATE MACHINE & SYSTEM MANAGEMENT
// ============================================================================

void setup() {
    // USB CDC dosen't care about baudrate anyway, but 115200 is a common default for serial monitors
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

    // Configure ADC resolution (RP2350: 12-bit)
    analogReadResolution(12);

    // Allow time for USB enumeration
    uint32_t boot_timer = millis();
    while (!Serial && millis() - boot_timer < 5000) { delay(10); }

    Serial.println("\n--- RapidFish Avionics (LITE) Initializing ---");

    flight_flash_offset = PICO_FLASH_SIZE_BYTES; 
    gpio_set_function(0, GPIO_FUNC_XIP_CS1);
    flash_devinfo_set_cs_size(1, flash_devinfo_bytes_to_size(FLIGHT_DATA_FLASH_SIZE));
    flash_devinfo_set_cs_gpio(1, 0);

    scanForAppendAddress();

    pyro1_fired = false;
    pyro2_fired = false;
    state_start_time = millis();
    current_state = STATE_ARMED;
    Serial.println("System Armed.");

    // Boot confirmation: 3 short beeps
    startBuzzerPattern(BUZZER_BOOT_BEEPS);
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

    // Buzzer pattern update (non-blocking)
    updateBuzzer();

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
                    startBuzzerPattern(BUZZER_LIFTOFF_SPAM);
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

        case STATE_RECOVERY: {
            // --- Pyro Continuity Check & Auto-Deployment ---
            // Determine which pyros have valid e-match continuity
            bool p1_has_cont = (current_p1_voltage >= PYRO_CONTINUITY_THRESHOLD);
            bool p2_has_cont = (current_p2_voltage >= PYRO_CONTINUITY_THRESHOLD);
            bool both_available = (p1_has_cont && p2_has_cont);
            bool one_available  = (p1_has_cont || p2_has_cont);

            // Fire primary pyro (pyro 1 if available, otherwise pyro 2)
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
                state_start_time = current_time; // Reset timer for redundancy window
                break;
            }

            // Redundancy: if primary was fired but no chute/ground transition after timeout,
            // fire the backup pyro if available
            if ((pyro1_fired || pyro2_fired) &&
                (current_time - state_start_time > PYRO_REDEPLOY_TIMEOUT_MS)) {
                
                if (both_available && !pyro2_fired) {
                    // Both were available, pyro1 was fired — fire pyro2 as backup
                    digitalWrite(PYRO2_PIN, HIGH);
                    pyro2_fire_start = current_time;
                    pyro2_active = true;
                    pyro2_fired = true;
                    Serial.println("PYRO2 fired (backup deployment, no chute detected).");
                    state_start_time = current_time;
                    break;
                } else if (!both_available && !pyro1_fired && !pyro2_fired) {
                    // Only one was available and it was already fired — no redundancy left
                    Serial.println("WARN: No backup pyro available, awaiting chute/ground passively.");
                }
            }

            // Transition to CHUTE once the deployment delay has elapsed
            // (allows time for pyro to fire and chute to deploy)
            if (current_time - state_start_time > RECOVERY_DELAY_MS) {
                current_state = STATE_CHUTE;
                state_start_time = current_time;
                Serial.println("State -> CHUTE (awaiting chute detection).");
            }
            break;
        }

        case STATE_CHUTE: {
            // Tolerance increased to 0.4 rad/s (~23 deg/s) to clear the uncalibrated zero-offset noise floor
            const float GYRO_STILL_TOLERANCE = 0.4f;
            const float GROUND_ALTITUDE_TOLERANCE = 50.0f; // Prevent mid-air ground triggers
            
            // Ground detection requires ALL of:
            // 1. Slow barometric descent rate (chute has deployed and is slowing the fall)
            // 2. Gyro magnitude below still threshold (not tumbling)
            // 3. Altitude below ground tolerance
            // 4. All conditions sustained for GROUND_WAIT_MS
            bool chute_deployed = (current_descent_rate < CHUTE_DESCENT_RATE_THRESHOLD);
            bool gyro_still     = (current_gyro_mag <= GYRO_STILL_TOLERANCE);
            bool low_altitude   = (current_altitude <= GROUND_ALTITUDE_TOLERANCE);

            if (chute_deployed && gyro_still && low_altitude) {
                if (current_time - state_start_time > GROUND_WAIT_MS) {
                    current_state = STATE_GROUND;
                    Serial.println("Flight complete (chute descent confirmed).");
                }
            } else {
                // Reset ground timer if any condition fails
                state_start_time = current_time;
            }
            break;
        }

        case STATE_GROUND:
            // SOS beacon runs continuously while on the ground
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

    // --- IMU: LSM6DSO on SPI0 (hardware pins) ---
    pinMode(LSM_CS_PIN, OUTPUT);
    digitalWrite(LSM_CS_PIN, HIGH);
    delay(10);

    // Hardware SPI0: CS=17, SCK=18, MOSI=19, MISO=16
    // The Adafruit library uses the default SPI (SPI0) when only CS is passed.
    if (!lsm.begin_SPI(LSM_CS_PIN)) {
        while (1) { delay(100); }
    }

    lsm.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    // --- BMP390 on I2C0 (Wire) ---
    Wire.setSDA(BMP_SDA_PIN);
    Wire.setSCL(BMP_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
    
    if (bmp.begin_I2C(0x76, &Wire)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ); 
    }

    // I2C1 (GPIO 2/3) is intentionally left uninitialized for user expansion.
    // GNSS UART1 (GPIO 8/9) and Radio SPI1 (GPIO 12-15) are reserved but not implemented.
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

        // 2. Baro Sample & ADC Readings (Decimated to ~50 Hz)
        static uint8_t decimator = 0;
        if (++decimator >= 20) {
            bmp.performReading();
            cached_core_temp = (int8_t)analogReadTemp();
            decimator = 0;
            
            // Barometer reference pressure averaging (ground level)
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

            // Descent rate: positive = descending, computed from baro altitude delta
            // Runs at ~50 Hz (every 20ms), so multiply by 50 to get m/s
            {
                static float prev_altitude = 0.0f;
                float delta = prev_altitude - current_altitude; // positive = descending
                current_descent_rate = (current_descent_rate * 0.7f) + (delta * 50.0f * 0.3f);
                prev_altitude = current_altitude;
            }

            // ADC samples: battery voltage, pyro continuity
            // Map 12-bit ADC (0-4095) to 8-bit (0-255) via >> 4
            // Divider: 200k+100k = 1/3 → real voltage = (raw_8bit * 3.3 * 3) / (255 * 1)
            // Simplified: V_bat = raw_8bit * 0.03882 (e.g. 200 → 7.76V)
            current_bat_voltage = (uint8_t)(analogRead(BAT_ADC_PIN) >> 4);
            current_p1_voltage  = (uint8_t)(analogRead(PYRO1_ADC_PIN) >> 4);
            current_p2_voltage  = (uint8_t)(analogRead(PYRO2_ADC_PIN) >> 4);
        }

        // 3. Logger — APID 0 Core Frames (1 kHz)
        if (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE) {
            LogFrameCore* f = &page_buffer[buffer_index].core;
            
            f->sync_word    = SYNC_WORD;
            f->timestamp    = millis();
            f->apid         = 0;  // APID 0 = core sensor frame
            f->flight_state = (uint8_t)current_state;
            f->flash_used   = (uint8_t)((current_flash_addr * 255) / FLIGHT_DATA_FLASH_SIZE);
            f->core_temp    = cached_core_temp;
            f->ax           = (int16_t)(accel.acceleration.x * 100);
            f->ay           = (int16_t)(accel.acceleration.y * 100);
            f->az           = (int16_t)(accel.acceleration.z * 100);
            f->gx           = (int16_t)(gyro.gyro.x * 1000);
            f->gy           = (int16_t)(gyro.gyro.y * 1000);
            f->gz           = (int16_t)(gyro.gyro.z * 1000);
            f->altitude     = (uint16_t)(current_altitude * 2.0f);
            f->temperature  = (int8_t)bmp.temperature;
            f->bat_voltage  = current_bat_voltage;
            f->p1_voltage   = current_p1_voltage;
            f->p2_voltage   = current_p2_voltage;
            // _pad[2] is implicitly zero (page_buffer is BSS-initialized)

            buffer_index++;

            if (buffer_index >= FRAMES_PER_PAGE) {
                if (current_flash_addr < FLIGHT_DATA_FLASH_SIZE) {
                    rp2040.idleOtherCore(); 
                    uint32_t ints = save_and_disable_interrupts();
                    flash_range_program(flight_flash_offset + current_flash_addr, 
                                        (uint8_t*)page_buffer, FLASH_PAGE_SIZE);
                    restore_interrupts(ints);
                    rp2040.resumeOtherCore();
                    current_flash_addr += FLASH_PAGE_SIZE;
                }
                buffer_index = 0;
            }
        }

        // 4. APID 1 frames (GPS / Magneto) — placeholder for future expansion.
        // These would be logged at a much lower rate (e.g. 1 Hz) and interleaved
        // with APID 0 frames in the same page buffer.
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
            Serial.printf("Descent Rate: %.2f m/s\n", current_descent_rate);
            Serial.printf("Accel X: %.2f G\n", current_accel_x);
            Serial.printf("Gyro Mag: %.2f rad/s\n", current_gyro_mag);
            Serial.printf("Battery ADC: %u\n", current_bat_voltage);
            Serial.printf("Pyro1 Cont: %u | Pyro2 Cont: %u\n", current_p1_voltage, current_p2_voltage);
            Serial.printf("Pyro1 Fired: %s | Pyro2 Fired: %s\n",
                          pyro1_fired ? "YES" : "NO", pyro2_fired ? "YES" : "NO");
            Serial.printf("Flash Usage: %u / %u bytes (%u%%)\n",
                          current_flash_addr, FLIGHT_DATA_FLASH_SIZE,
                          (current_flash_addr * 100) / FLIGHT_DATA_FLASH_SIZE);
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
        case BUZZER_BOOT_BEEPS: {
            // 3 short beeps: 100ms on, 100ms off, repeat 3 times
            const uint32_t CYCLE_MS = 200; // 100 on + 100 off
            const uint8_t  TOTAL_CYCLES = 6; // 3 on + 3 off
            uint8_t cycle = elapsed / CYCLE_MS;
            if (cycle >= TOTAL_CYCLES) {
                buzzer.pattern = BUZZER_IDLE;
                digitalWrite(BUZZER_PIN, LOW);
                return;
            }
            // Even cycles = on, odd cycles = off
            digitalWrite(BUZZER_PIN, (cycle % 2 == 0) ? HIGH : LOW);
            break;
        }

        case BUZZER_LIFTOFF_SPAM: {
            // Rapid spam: 50ms on, 50ms off for 600ms total
            const uint32_t CYCLE_MS = 100; // 50 on + 50 off
            const uint8_t  TOTAL_CYCLES = 6; // 600ms total
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
                60, 60, 60, 60, 60, 120,   // S
                180, 60, 180, 60, 180, 120, // O
                60, 60, 60, 60, 60, 400    // S + pause
            };
            const uint8_t NUM_STEPS = sizeof(sos_pattern) / sizeof(sos_pattern[0]);
            
            // Find current step
            uint32_t accum = 0;
            uint8_t step = 0;
            while (step < NUM_STEPS && accum + sos_pattern[step] <= elapsed) {
                accum += sos_pattern[step];
                step++;
            }
            
            if (step >= NUM_STEPS) {
                // Restart SOS pattern
                buzzer.step_start = now;
                digitalWrite(BUZZER_PIN, LOW);
                return;
            }
            
            // Even steps = on, odd steps = off
            digitalWrite(BUZZER_PIN, (step % 2 == 0) ? HIGH : LOW);
            break;
        }

        default:
            buzzer.pattern = BUZZER_IDLE;
            digitalWrite(BUZZER_PIN, LOW);
            break;
    }
}