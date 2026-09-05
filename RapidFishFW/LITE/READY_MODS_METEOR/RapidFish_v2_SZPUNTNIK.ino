/*
 * =============================================================================
 *  RapidFish v2 — Unified CARP Avionics Firmware (RP2350 Dual-Core)
 * =============================================================================
 *  This firmware merges servo and pyro recovery into a single clean codebase.
 *  It supports the CARP-LITE v1 and v2 boards.
 *
 *  Flash layout (16 MB single chip):
 *    0 – 2 MB:   Firmware / sketch (quarantined, never touched)
 *    2 – 16 MB:  Flight data log (14 MB, hard-clamped)
 *
 *  ============================================================================
 *  HOW TO USE THIS FILE
 *  ============================================================================
 *  1. Open in Arduino IDE or PlatformIO (board = rpipico2).
 *  2. Scroll down to Section [1] — CONFIGURATION — and set your rocket's
 *     parameters. Everything you might need to change is there.
 *  3. Upload and open the Serial Monitor at 115200 baud.
 *  4. Type HELP to see all commands.
 *  5. Type CHECKLIST before flight to verify all sensors and thresholds.
 *
 *  ============================================================================
 *  LEGEND
 *  ============================================================================
 *  [USER]  = You may safely modify this value.
 *  [FIXED] = Do not change unless you know exactly what you are doing.
 *  [INTERNAL] = Internal firmware state; do not modify.
 * =============================================================================
 */

// =============================================================================
// [0] INCLUDES — [FIXED] Required libraries
// =============================================================================
#include <pico.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/critical_section.h"
#include <FastLED.h>
#include <Adafruit_LSM6DSO32.h>
#include "SparkFun_LSM6DSV16X.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <RadioLib.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <Servo.h>

// =============================================================================
// [1] CONFIGURATION — [USER] Modify these values for your rocket
// =============================================================================

// ---------------------------------------------------------------------------
// 1a. Recovery Channel Type — [USER] Choose SERVO or PYRO per channel
// ---------------------------------------------------------------------------
// For each of the two recovery channels (ID1, ID2), select what hardware is
// attached. Options:
//   RECOVERY_TYPE_SERVO  — A servo motor (e.g. for retention / deployment)
//   RECOVERY_TYPE_PYRO   — A pyrotechnic igniter (e.g. e-match)
#define RECOVERY_TYPE_ID1  RECOVERY_TYPE_SERVO
#define RECOVERY_TYPE_ID2  RECOVERY_TYPE_NONE

// ---------------------------------------------------------------------------
// 1b. Servo Angles — [USER] Set lock/unlock positions (0–180°)
// ---------------------------------------------------------------------------
// LOCKED   = the position that keeps the rocket mechanically safe / closed
// UNLOCKED = the position that releases / deploys
// These are only used when the corresponding channel is RECOVERY_TYPE_SERVO.
#define SERVO1_LOCKED_ANGLE   180   // [USER] ID1 servo: safe/closed position
#define SERVO1_UNLOCKED_ANGLE 0     // [USER] ID1 servo: deployed/open position
#define SERVO2_LOCKED_ANGLE   180   // [USER] ID2 servo: safe/closed position
#define SERVO2_UNLOCKED_ANGLE 0     // [USER] ID2 servo: deployed/open position

// ---------------------------------------------------------------------------
// 1c. Testbench Mode — [USER] Enable for bench testing (safe, quiet, low power)
// ---------------------------------------------------------------------------
// When enabled:
//   - LED brightness reduced to 5 (very dim)
//   - Buzzer uses low-frequency PWM at ~4% duty cycle (barely audible)
//   - Radio TX power reduced to TESTBENCH_RADIO_POWER_DBM
//   - Recovery system activations are replaced with serial text (no hardware fire)
// Disable (false) for actual flight.
#define TESTBENCH_MODE false

// ---------------------------------------------------------------------------
// 1d. Hardware Pins — Only change if you have a custom board layout
// ---------------------------------------------------------------------------
#define LED_PIN         25    //  WS2812B data pin
#define NUM_LEDS        1     //  Number of WS2812B LEDs

#define SERVO1_PIN      23    //  Servo/GPIO for recovery channel ID1
#define SERVO2_PIN      20    //  Servo/GPIO for recovery channel ID2
#define PYRO1_PIN       21    //  Pyro trigger for channel ID1
#define PYRO2_PIN       22    //  Pyro trigger for channel ID2

#define LSM_CS_PIN      17    //  IMU chip select (SPI0)
#define BMP_SDA_PIN     4     //  Barometer I2C SDA
#define BMP_SCL_PIN     5     //  Barometer I2C SCL

#define BAT_ADC_PIN     26    //  Battery voltage divider ADC
#define PYRO1_ADC_PIN   28    //  Pyro 1 continuity ADC
#define PYRO2_ADC_PIN   29    //  Pyro 2 continuity ADC

#define BUZZER_PIN      24    //  Buzzer output pin

// ---------------------------------------------------------------------------
// 1e. Flight Physics Thresholds — [USER] Tune for your rocket
// ---------------------------------------------------------------------------
const float LAUNCH_G_THRESHOLD       = 3.0f;   // [USER] G-force to detect liftoff
const float BURNOUT_G_THRESHOLD      = 0.5f;   // [USER] G-force below which = burnout
const float APOGEE_DIP_METERS        = 6.0f;   // [USER] Altitude drop to confirm apogee
const float GROUND_G_TOLERANCE       = 0.1f;   // [USER] G-range considered "still"
const bool  IS_UPSIDE_DOWN           = false;  // [USER] True if rocket is stored upside-down. If SAT logo is upside-down, set true.

// ---------------------------------------------------------------------------
// 1f. Timing Thresholds (milliseconds) — [USER]
// ---------------------------------------------------------------------------
const uint32_t MIN_MOTOR_BURN_MS       = 2500;    // [USER] Minimum burn time before burnout check
const uint32_t MAX_MOTOR_BURN_MS       = 6000;    // [USER] Max burn time (timeout failsafe)
const uint32_t RECOVERY_DELAY_MS       = 0;       // [USER] Delay after deployment before CHUTE
const uint32_t GROUND_WAIT_MS          = 10000;   // [USER] Time on ground before STATE_GROUND
const uint32_t GROUND_LOG_DURATION_MS  = 10000;   // [USER] How long to keep logging after landing (ms)

// ---------------------------------------------------------------------------
// 1g. Pyro Deployment Thresholds — [USER] (only used if PYRO selected)
// ---------------------------------------------------------------------------
const uint8_t  PYRO_CONTINUITY_THRESHOLD   = 10;     // [USER] ADC value for continuity
const uint32_t PYRO_FIRE_DURATION_MS       = 3000;   // [USER] How long to fire pyro (ms)
const uint32_t PYRO_REDEPLOY_TIMEOUT_MS    = 3000;   // [USER] Backup pyro wait (ms)
const float    CHUTE_DESCENT_RATE_THRESHOLD = 15.0f;  // [USER] m/s below which = chute out

// ---------------------------------------------------------------------------
// 1h. Radio Telemetry — [USER] Frequency, power, intervals
// ---------------------------------------------------------------------------
const bool RADIO_ENABLED = true;                    // [USER] Set false to disable radio entirely
#define RADIO_CS_PIN    13                          // [FIXED] LR2021 SPI1 CS
#define RADIO_IRQ_PIN   6                           // [FIXED] LR2021 DIO5
#define RADIO_RST_PIN   11                          // [FIXED] LR2021 Reset
#define RADIO_BUSY_PIN  10                          // [FIXED] LR2021 Busy

#define RADIO_FREQUENCY_MHZ 434.0f                  // [USER] Frequency in MHz (400–510)
#define RADIO_TX_INTERVAL_MS           80          // [USER] Flight core frame interval (ms)
#define RADIO_ARMED_INTERVAL_MS        1000         // [USER] ARMED beacon interval (ms)
#define RADIO_GROUND_CORE_INTERVAL_MS 10000         // [USER] Ground core frame interval (ms)
#define RADIO_GROUND_GPS_INTERVAL_MS   1000         // [USER] Ground GPS frame interval (ms)
#define RADIO_FLIGHT_GPS_GUARANTEE_MS  1000         // [USER] Flight GPS hard floor (~1 Hz) regardless of GPS update cadence
// GPS transmits are gated to land mid-inter-core gap, derived from the cadence constants.
#define RADIO_GPS_TO_CORE_GAP_MS_MIN  15                      // [INTERNAL] floor (ms)
#define RADIO_GPS_TO_CORE_GAP_MS_CAP  (RADIO_FLIGHT_GPS_GUARANTEE_MS / 2) // [INTERNAL] must stay < 1 Hz guarantee
#define RADIO_GPS_TO_CORE_GAP_MS \
    ( ((RADIO_TX_INTERVAL_MS / 2) > RADIO_GPS_TO_CORE_GAP_MS_MIN) \
        ? (((RADIO_TX_INTERVAL_MS / 2) < RADIO_GPS_TO_CORE_GAP_MS_CAP) \
            ? (RADIO_TX_INTERVAL_MS / 2) \
            : RADIO_GPS_TO_CORE_GAP_MS_CAP) \
        : RADIO_GPS_TO_CORE_GAP_MS_MIN )
// Ground GPS: land mid-gap after the last ground core frame.
#define RADIO_GROUND_GPS_CORE_GAP_MS_MIN  20                 // [INTERNAL] floor (ms)
#define RADIO_GROUND_GPS_CORE_GAP_MS \
    ( ((RADIO_GROUND_GPS_INTERVAL_MS / 2) > RADIO_GROUND_GPS_CORE_GAP_MS_MIN) \
        ? (RADIO_GROUND_GPS_INTERVAL_MS / 2) \
        : RADIO_GROUND_GPS_CORE_GAP_MS_MIN )
#define RADIO_POWER_ARMED_DBM          10.0f        // [USER] ARMED TX power (dBm)
#define RADIO_POWER_HIGH_DBM           22.0f        // [USER] Flight + ground TX power (dBm)
#define TESTBENCH_RADIO_POWER_DBM      3.0f         // [USER] Testbench TX power (dBm)

// ---------------------------------------------------------------------------
// 1i. GPS Configuration — [FIXED]
// ---------------------------------------------------------------------------
#define GPS_TX_PIN 8                                // [FIXED] GPS module TX (to RP2350 RX)
#define GPS_RX_PIN 9                                // [FIXED] GPS module RX (to RP2350 TX)
#define GPS_BAUD   115200                           // [FIXED] ATGM336H baud rate

#define GPS_LOCK_LOSS_TIMEOUT_MS 30000              // [FIXED] No-fix timeout before restart
#define GPS_WATCHDOG_COOLDOWN_MS 60000              // [FIXED] Min interval between restarts
#define GPS_FIX_FRESH_MS 10000                      // [FIXED] Max age of fix before considered stale and doing hot restart

// ---------------------------------------------------------------------------
// 1j. Flash Storage — [FIXED] Do not change unless you know the flash layout
// ---------------------------------------------------------------------------
const uint32_t FIRMWARE_RESERVED_SIZE = 2 * 1024 * 1024;  // [FIXED] 2 MB safe zone
const uint32_t FLIGHT_DATA_FLASH_SIZE = 14 * 1024 * 1024; // [FIXED] 14 MB log capacity
const uint32_t SYNC_WORD = 0x1ACFFC1D; // [INTERNAL] Sync word for log frames (32-bit)

// =============================================================================
// [2] INTERNAL DEFINITIONS — [INTERNAL] Do not modify below this line
// =============================================================================

// --- Recovery type enum ---
#define RECOVERY_TYPE_SERVO 0
#define RECOVERY_TYPE_PYRO  1
#define RECOVERY_TYPE_NONE  2

// --- Flight state machine ---
enum FlightState {
    STATE_BOOTING,
    STATE_DISARMED,
    STATE_ERROR,
    STATE_ARMED,
    STATE_ACCELERATING,
    STATE_COAST,
    STATE_RECOVERY,
    STATE_CHUTE,
    STATE_GROUND
};

// Human-readable state names (index must match FlightState order)
static const char* STATE_NAMES[] = {
    "BOOTING", "DISARMED", "ERROR", "ARMED",
    "ACCELERATING", "COAST", "RECOVERY", "CHUTE", "GROUND"
};

// --- APID 0: Core Sensor Frame (32 bytes) — matches dataframes.md ---
struct __attribute__((packed)) LogFrameCore {
    uint32_t sync_word;       // +0  (4) SyncWord (0x1ACFFC1D)
    uint32_t timestamp;       // +4  (4) millis()
    uint8_t  apid;            // +8  (1) = 0 (core)
    uint8_t  flight_state;    // +9  (1)
    uint8_t  flash_used;      // +10 (1) 0–200 = 0–100% in 0.5% increments
    int8_t   core_temp;       // +11 (1) RP2350 internal temp °C
    int16_t  ax, ay, az;      // +12 (6) Accel: (m/s²) * 100
    int16_t  gx, gy, gz;      // +18 (6) Gyro: (rad/s) * 1000
    uint32_t pressure;        // +24 (4) Baro raw Pa (/100 for hPa)
    int8_t   temperature;     // +28 (1) Barometer temp °C
    uint8_t  bat_voltage;     // +29 (1) Battery ADC (0–255 scaled)
    uint8_t  p1_voltage;      // +30 (1) Channel 1 continuity ADC
    uint8_t  p2_voltage;      // +31 (1) Channel 2 continuity ADC
};
static_assert(sizeof(LogFrameCore) == 32, "LogFrameCore must be 32 bytes");

// --- APID 1: GPS Frame (32 bytes) — matches dataframes.md ---
struct __attribute__((packed)) LogFrameGPS {
    uint32_t sync_word;   // +0  (4) SyncWord (0x1ACFFC1D)
    uint32_t timestamp;   // +4  (4) millis()
    uint8_t  apid;        // +8  (1) = 1 (gps)
    int32_t  lat;         // +9  (4) Latitude  * 1e7   (signed)
    int32_t  lon;         // +13 (4) Longitude * 1e7   (signed)
    uint16_t gps_alt;     // +17 (2) MSL altitude in 0.5 m units (meters*2)
    uint8_t  state;       // +19 (1) GPS fix state (1 = valid)
    uint8_t  sats;        // +20 (1) Satellite count
    uint32_t gps_time;    // +21 (4) Unix timestamp (seconds since epoch, UTC)
    uint8_t  hdop;        // +25 (1) HDOP * 10 (clamped to 255)
    int16_t  mx, my, mz;  // +26 (6) Magnetometer raw (placeholder)
};
static_assert(sizeof(LogFrameGPS) == 32, "LogFrameGPS must be 32 bytes");

union LogFrame {
    LogFrameCore core;
    LogFrameGPS  gps;
    uint8_t      bytes[32];
};
static_assert(sizeof(union LogFrame) == 32, "LogFrame union must be 32 bytes");

// --- Hardware objects ---
CRGB leds[NUM_LEDS];
Adafruit_LSM6DSO32 lsm_dsox;
SparkFun_LSM6DSV16X_SPI lsm_dsv;
bool use_dsox = false;
Adafruit_BMP3XX bmp;

// RadioLib re-clocks SPI each transaction; pass explicit SPISettings to override.
#define RADIO_SPI_CLOCK_HZ 8000000
LR2021 radio = new Module(RADIO_CS_PIN, RADIO_IRQ_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI1,
                          SPISettings(RADIO_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
bool radio_ready = false;

// Interrupt-driven TX state
static uint8_t radio_tx_buf[32];
static size_t  radio_tx_len = 0;
volatile bool  radio_tx_busy = false;
volatile bool  radio_tx_done = false;
static volatile uint32_t radio_tx_busy_start_ms = 0;
// Watchdog uses wall-clock time_us_64() because flash writes freeze millis().
static volatile uint64_t radio_tx_busy_start_us = 0;
#define RADIO_TX_BUSY_TIMEOUT_MS 500
#define RADIO_TX_BUSY_TIMEOUT_US (RADIO_TX_BUSY_TIMEOUT_MS * 1000u)

// Radio TX drop / backpressure diagnostics
uint32_t radio_tx_attempted = 0;
uint32_t radio_tx_dropped   = 0;

// Rolling 10-second TX success diagnostic, windowed on true time_us_64().
static uint32_t win_ok       = 0;
static uint32_t win_dropped  = 0;
static uint64_t win_start_us = 0;

// Radio TX cadence timers
uint32_t last_radio_tx      = 0;
uint32_t last_radio_core_tx = 0;
uint32_t last_gps_tx        = 0;
uint32_t last_gps_tx_cnt    = 0;
uint32_t last_gps_flight_tx = 0;
volatile bool beacon_is_gps = false;

// Cached radio output power
static float radio_current_power_dbm = -999.0f;

// --- GPS ---
TinyGPSPlus tinyGPS;
bool gps_ready = false;
uint32_t gps_last_nmea_ms = 0;
uint32_t gps_last_valid_fix_ms = 0;
uint32_t gps_last_restart_ms   = 0;

volatile uint32_t gps_raw_bytes       = 0;
volatile uint32_t gps_valid_sentences = 0;
volatile uint32_t gps_bad_checksum    = 0;

// --- Flash memory ---
uint32_t flight_flash_offset = FIRMWARE_RESERVED_SIZE;
volatile uint32_t current_flash_addr = 0;
const int FRAMES_PER_PAGE = FLASH_PAGE_SIZE / sizeof(union LogFrame);
union LogFrame page_buffer[8];
int buffer_index = 0;

// --- Shared volatile state (core 0 and core 1) ---
volatile FlightState current_state = STATE_BOOTING;
volatile float current_gforce     = 0.0f;
volatile float current_accel_x    = 0.0f;
volatile float current_accel_y    = 0.0f;
volatile float current_accel_z    = 0.0f;
volatile float current_gyro_x     = 0.0f;
volatile float current_gyro_y     = 0.0f;
volatile float current_gyro_z     = 0.0f;
volatile float current_gyro_mag   = 0.0f;
volatile float current_altitude   = 0.0f;
volatile float max_altitude       = 0.0f;
volatile float reference_pressure_hpa = 1013.25f;
volatile uint32_t state_start_time = 0;
volatile uint8_t system_errors = 0;
critical_section_t system_errors_lock;
volatile bool force_armed = false;
volatile bool core1_init_complete = false;
volatile int radio_error_code = 0;

// --- Recovery channel state ---
// ID1
Servo servo1;
bool servo1_attached = false;
bool ch1_fired = false;   // true when channel 1 has been activated
// ID2
Servo servo2;
bool servo2_attached = false;
bool ch2_fired = false;

// Pyro-specific state (only meaningful when channel type is PYRO)
uint32_t pyro1_fire_start = 0;
uint32_t pyro2_fire_start = 0;
bool pyro1_active = false;
bool pyro2_active = false;

volatile float current_descent_rate = 0.0f;
volatile uint8_t current_bat_voltage = 0;
volatile uint8_t current_p1_voltage = 0;
volatile uint8_t current_p2_voltage = 0;
volatile int8_t  current_core_temp = 0;
volatile int8_t  current_baro_temp = 0;
volatile uint32_t current_pressure_pa = 101325;

// --- GPS snapshot (core 0 → frame builders) ---
struct GpsSnapshot {
    volatile uint32_t fix_time_ms;
    volatile uint8_t  fix_valid;
    volatile int32_t  lat_e7;
    volatile int32_t  lon_e7;
    volatile uint16_t alt_half_m;
    volatile uint32_t gps_time;
    volatile uint8_t  hdop_x10;
    volatile uint8_t  sats;
    volatile uint16_t update_cnt;
};
GpsSnapshot gps_snapshot;

static int gps_last_good_year  = 0;
static int gps_last_good_month = 0;
static int gps_last_good_day   = 0;

// Pending GPS frame (core 0 → core 1)
// Guarded by gps_mailbox_lock so the producer's write and the consumer's
// copy+clear are mutually exclusive (no torn read/write window).
critical_section_t gps_mailbox_lock;
volatile bool  gps_pending_flag = false;
union LogFrame gps_pending_frame;

// --- Buzzer patterns ---
enum BuzzerPattern {
    BUZZER_IDLE, BUZZER_ERROR, BUZZER_BOOT_BEEPS, BUZZER_LIFTOFF_SPAM, BUZZER_SOS
};

struct BuzzerState {
    BuzzerPattern pattern;
    uint32_t      step_start;
    uint8_t       step_index;
    bool          on_state;
} buzzer = { BUZZER_IDLE, 0, 0, false };

// --- Testbench mode runtime flag ---
bool testbench_active = TESTBENCH_MODE;

// --- Forward declarations ---
void scanForAppendAddress();
void handleSerialCommands();
void updateBuzzer();
void startBuzzerPattern(BuzzerPattern p);
void radioInit();
bool radioTransmitFrame();
void radioSetFrequency(float mhz);
void radioSetPower(float dbm);
bool radioTransmitGpsFrame();
static bool radioTransmit(const uint8_t* frame, size_t len);
void radioApplyStatePower();
void radioTxDoneISR();
void gpsInit();
void handleGps();
void configureAtgm336h();
void sendPcasCmd(const char* payload);
void gpsSnapshotCapture(uint32_t now);
void gpsFrameFill(LogFrameGPS* f);
void emitGpsFrame();
void drainGpsFrame();
void flushPageBuffer();
bool gpsHasUsableLock();
void gpsLockWatchdog();
void deployChannel1();
void deployChannel2();
void firePyro1();
void firePyro2();
void setServoAngle(int id, int angle);
void printChecklist();
void printStatus();

// =============================================================================
// [3] CORE 0: SETUP & STATE MACHINE
// =============================================================================

void setup() {
    Serial.begin(115200);

    critical_section_init(&system_errors_lock);
    critical_section_init(&gps_mailbox_lock);

    // --- Initialize recovery hardware based on type ---
    if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_SERVO) {
        servo1.attach(SERVO1_PIN);
        servo1_attached = true;
        servo1.write(SERVO1_LOCKED_ANGLE);
        pinMode(PYRO1_PIN, OUTPUT);
        digitalWrite(PYRO1_PIN, LOW);
    } else if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_PYRO) {
        pinMode(PYRO1_PIN, OUTPUT);
        digitalWrite(PYRO1_PIN, LOW);
    } else {
        // RECOVERY_TYPE_NONE: no hardware init for channel 1
    }

    if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_SERVO) {
        servo2.attach(SERVO2_PIN);
        servo2_attached = true;
        servo2.write(SERVO2_LOCKED_ANGLE);
        pinMode(PYRO2_PIN, OUTPUT);
        digitalWrite(PYRO2_PIN, LOW);
    } else if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_PYRO) {
        pinMode(PYRO2_PIN, OUTPUT);
        digitalWrite(PYRO2_PIN, LOW);
    } else {
        // RECOVERY_TYPE_NONE: no hardware init for channel 2
    }

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    if (testbench_active) {
        FastLED.setBrightness(5);
    } else {
        FastLED.setBrightness(255);
    }
    leds[0] = CRGB::Blue;
    FastLED.show();

    analogReadResolution(12);

    uint32_t boot_timer = millis();
    while (!Serial && millis() - boot_timer < 5000) { delay(10); }

    Serial.println("\n==============================================");
    Serial.println("   RapidFish v2 - CARP Avionics");
    Serial.println("==============================================");
    if (testbench_active) {
        Serial.println("***##################### DO NOT FLY ########################***");
        Serial.println("*** TESTBENCH MODE ACTIVE***");
        Serial.println("  - Low LED brightness, quiet buzzer, low radio power");
        Serial.println("  - Recovery outputs replaced with serial text");
        Serial.println("  - Flight WILL result in catastrophic failure! Re-flash your CARP Avionics.");
    }
    Serial.println();

    scanForAppendAddress();
    radioInit();
    gpsInit();

    Serial.println("Waiting for sensors to initialize...");
    while (!core1_init_complete) { delay(10); }

    // --- Boot result ---
    if (system_errors > 0) {
        if (force_armed) {
            Serial.println("\n*** BOOT FAILURE: SENSOR/RADIO ERROR (force_armed=true) ***");
            if (system_errors & 1) Serial.println("- IMU failed. Data zeroed.");
            if (system_errors & 2) Serial.println("- Barometer failed. Data zeroed.");
            if (system_errors & 4) Serial.printf("- Radio failed. Code: %d\n", radio_error_code);
            ch1_fired = false;
            ch2_fired = false;
            state_start_time = millis();
            current_state = STATE_ARMED;
            Serial.println("System Armed (force_armed).");
            startBuzzerPattern(BUZZER_BOOT_BEEPS);
        } else {
            current_state = STATE_ERROR;
            Serial.println("\n*** BOOT FAILURE: SENSOR/RADIO ERROR ***");
            if (system_errors & 1) Serial.println("- IMU failed.");
            if (system_errors & 2) Serial.println("- Barometer failed.");
            if (system_errors & 4) Serial.printf("- Radio failed. Code: %d\n", radio_error_code);
            Serial.println("System halted in STATE_ERROR. Type ARM to force-start, or REBOOT.");
            startBuzzerPattern(BUZZER_ERROR);
        }
    } else {
        ch1_fired = false;
        ch2_fired = false;
        state_start_time = millis();
        current_state = STATE_ARMED;
        Serial.println("System Armed. All systems nominal.");
        startBuzzerPattern(BUZZER_BOOT_BEEPS);
    }

}

// --- Write any partially-filled page buffer to flash, then reset it ---
void flushPageBuffer() {
    // Halt core 1 (and mask IRQs) before touching shared buffers so no
    // concurrent write into page_buffer / buffer_index can occur.
    rp2040.idleOtherCore();
    uint32_t ints = save_and_disable_interrupts();

    if (buffer_index == 0) {
        restore_interrupts(ints);
        rp2040.resumeOtherCore();
        return;
    }

    // Pad the unused tail with 0xFF (erase state).
    uint32_t used_bytes = (uint32_t)buffer_index * sizeof(union LogFrame);
    memset((uint8_t*)page_buffer + used_bytes, 0xFF, FLASH_PAGE_SIZE - used_bytes);

    if (current_flash_addr + FLASH_PAGE_SIZE <= FLIGHT_DATA_FLASH_SIZE) {
        flash_range_program(flight_flash_offset + current_flash_addr,
                            (uint8_t*)page_buffer, FLASH_PAGE_SIZE);
        current_flash_addr += FLASH_PAGE_SIZE;
    }

    buffer_index = 0;
    memset(page_buffer, 0, sizeof(page_buffer));

    restore_interrupts(ints);
    rp2040.resumeOtherCore();
}

void loop() {
    handleGps();
    gpsLockWatchdog();
    handleSerialCommands();
    uint32_t now = millis();

    // --- Pyro auto-shutoff (3 s burn timer) ---
    if (pyro1_active && (now - pyro1_fire_start >= PYRO_FIRE_DURATION_MS)) {
        digitalWrite(PYRO1_PIN, LOW);
        pyro1_active = false;
        Serial.println("PYRO1: burn complete (auto-off).");
    }
    if (pyro2_active && (now - pyro2_fire_start >= PYRO_FIRE_DURATION_MS)) {
        digitalWrite(PYRO2_PIN, LOW);
        pyro2_active = false;
        Serial.println("PYRO2: burn complete (auto-off).");
    }

    // --- LED state indicator (blinking) ---
    static bool last_blink_state = false;
    static FlightState last_indicated_state = STATE_BOOTING;
    bool current_blink_state = false;

    if (current_state == STATE_ERROR) {
        current_blink_state = ((now / 100) % 2 == 0);
    } else {
        current_blink_state = ((now / 500) % 2 == 0);
    }

    if (current_blink_state != last_blink_state || current_state != last_indicated_state) {
        if (current_blink_state) {
            switch (current_state) {
                case STATE_DISARMED:     leds[0] = CRGB::Orange; break;
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

    // --- Radio telemetry ---
    if (radio_ready) {
        uint16_t now_cnt = (uint16_t)gps_snapshot.update_cnt;

        bool clear_busy = false;
        if (radio_tx_busy) {
            if (radio_tx_done) {
                radio.finishTransmit();
                clear_busy = true;
            } else if (time_us_64() - radio_tx_busy_start_us >= RADIO_TX_BUSY_TIMEOUT_US) {
                // Lost DIO5 IRQ: watchdog uses time_us_64(), immune to flash-write millis() freeze.
                radio.finishTransmit();
                clear_busy = true;
                static uint32_t last_wd_print = 0;
                unsigned long wd_now = millis();
                if (wd_now - last_wd_print > 1000) {
                    last_wd_print = wd_now;
                    // Diagnostics: us vs millis() divergence during flash writes.
                    uint64_t busy_age_us  = time_us_64() - radio_tx_busy_start_us;
                    uint32_t busy_age_ms  = (uint32_t)(busy_age_us / 1000u);
                    Serial.printf("[LR2021] TX busy watchdog: recovered stuck transmit "
                                  "(us-age=%lums millis-age=%lums radio_tx_done=%d)\n",
                                  (unsigned long)busy_age_ms,
                                  (unsigned long)(wd_now - radio_tx_busy_start_ms),
                                  (int)radio_tx_done);
                }
            }
            if (clear_busy) {
                radio_tx_busy = false;
                radio_tx_done = false;
                radio_tx_busy_start_ms = 0;
                radio_tx_busy_start_us = 0;
            }
        }

        radioApplyStatePower();

        if (current_state == STATE_DISARMED) {
            // DISARMED: no radio
        } else if (current_state == STATE_ARMED) {
            if (now - last_radio_tx >= RADIO_ARMED_INTERVAL_MS) {
                if (beacon_is_gps) {
                    if (radioTransmitGpsFrame()) {
                        last_radio_tx = now;
                        beacon_is_gps = !beacon_is_gps;
                    }
                } else {
                    if (radioTransmitFrame()) {
                        last_radio_tx = now;
                        beacon_is_gps = !beacon_is_gps;
                    }
                }
            }
        } else if (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE) {
            if (now - last_radio_core_tx >= RADIO_TX_INTERVAL_MS) {
                if (radioTransmitFrame()) {
                    last_radio_core_tx = now;
                }
            }
            // Gate GPS behind RADIO_GPS_TO_CORE_GAP_MS to land mid inter-core gap.
            if (now - last_radio_core_tx >= RADIO_GPS_TO_CORE_GAP_MS) {
                // Normal path: fresh updates; watchdog floors the rate at >=1 Hz.
                if (now_cnt != last_gps_tx_cnt) {
                    if (radioTransmitGpsFrame()) {
                        last_gps_tx_cnt = now_cnt;
                        last_gps_flight_tx = now;
                    }
                } else if (now - last_gps_flight_tx >= RADIO_FLIGHT_GPS_GUARANTEE_MS) {
                    if (radioTransmitGpsFrame()) {
                        last_gps_flight_tx = now;
                    }
                }
            }
        } else if (current_state == STATE_GROUND) {
            if (now - last_radio_core_tx >= RADIO_GROUND_CORE_INTERVAL_MS) {
                if (radioTransmitFrame()) {
                    last_radio_core_tx = now;
                }
            }
            // Ground GPS is gated by a min gap after the last core frame.
            if (now - last_gps_tx >= RADIO_GROUND_GPS_INTERVAL_MS &&
                now - last_radio_core_tx >= RADIO_GROUND_GPS_CORE_GAP_MS) {
                if (radioTransmitGpsFrame()) {
                    last_gps_tx = now;
                }
            }
        }
    }

    // --- Flight state machine ---
    switch (current_state) {

        case STATE_DISARMED:
            // DISARMED: idle, no flight logic, no recovery, no logging.
            break;

        case STATE_ERROR:
            break;

        case STATE_ARMED: {
            static uint32_t launch_detect_start = 0;
            if (current_accel_x > LAUNCH_G_THRESHOLD) {
                if (launch_detect_start == 0) launch_detect_start = now;
                else if (now - launch_detect_start > 50) {
                    current_state = STATE_ACCELERATING;
                    state_start_time = now;
                    Serial.println("Liftoff detected.");
                    startBuzzerPattern(BUZZER_LIFTOFF_SPAM);
                }
            } else {
                launch_detect_start = 0;
            }
            break;
        }

        case STATE_ACCELERATING:
            if ((now - state_start_time > MIN_MOTOR_BURN_MS) &&
                (current_accel_x < BURNOUT_G_THRESHOLD)) {
                current_state = STATE_COAST;
                state_start_time = now;
                Serial.println("Burnout detected.");
            }
            else if (now - state_start_time > MAX_MOTOR_BURN_MS) {
                current_state = STATE_COAST;
                state_start_time = now;
                Serial.println("Burnout detected (timeout failsafe).");
            }
            break;

        case STATE_COAST:
            if (current_altitude < (max_altitude - APOGEE_DIP_METERS)) {
                current_state = STATE_RECOVERY;
                state_start_time = now;
                Serial.println("Apogee reached.");
            }
            break;

        case STATE_RECOVERY: {
            // --- Determine primary and secondary channels ---
            // If ID1 is NONE, ID2 becomes the primary (and there is no secondary).
            // If both are NONE, skip all internal recovery activation.
            const bool ch1_is_primary = (RECOVERY_TYPE_ID1 != RECOVERY_TYPE_NONE);
            const bool ch2_is_primary = (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_NONE && RECOVERY_TYPE_ID2 != RECOVERY_TYPE_NONE);
            const bool has_secondary  = (RECOVERY_TYPE_ID1 != RECOVERY_TYPE_NONE && RECOVERY_TYPE_ID2 != RECOVERY_TYPE_NONE);
            const bool any_recovery   = (RECOVERY_TYPE_ID1 != RECOVERY_TYPE_NONE || RECOVERY_TYPE_ID2 != RECOVERY_TYPE_NONE);

            if (!any_recovery) {
                // Both NONE: skip all internal recovery activation.
                // Wait for external recovery systems, then proceed to CHUTE.
                if (!ch1_fired) {
                    ch1_fired = true; // mark as "entered recovery"
                    state_start_time = now;
                    Serial.println("RECOVERY: Both channels NONE — waiting for external recovery.");
                    break;
                }
                // Fall through to chute confirmation logic below.
            } else if (ch1_is_primary) {
                // --- Deploy channel 1 (primary) immediately ---
                if (!ch1_fired) {
                    deployChannel1();
                    ch1_fired = true;
                    state_start_time = now;
                    Serial.println("RECOVERY: Ch1 deployed, waiting for chute to open...");
                    break;
                }
            } else if (ch2_is_primary) {
                // ID1 is NONE, so ID2 is primary — deploy immediately
                if (!ch2_fired) {
                    deployChannel2();
                    ch2_fired = true;
                    state_start_time = now;
                    Serial.println("RECOVERY: ID1=NONE, deploying Ch2 as primary.");
                    break;
                }
            }

            // --- Ground detection early exit ---
            // If we're low, still, and not descending, we're on the ground.
            // This prevents the descent-rate integrator from falsely confirming
            // a chute when the rocket has already impacted.
            {
                const float GROUND_ALT_TOLERANCE = 50.0f;
                const float GYRO_STILL_TOL = 0.4f;
                const float DESCENT_NEAR_ZERO = 1.0f;
                static uint32_t ground_confirm_start = 0;

                bool on_ground = (current_altitude < GROUND_ALT_TOLERANCE &&
                                  current_gyro_mag <= GYRO_STILL_TOL &&
                                  fabsf(current_descent_rate) < DESCENT_NEAR_ZERO);

                if (on_ground) {
                    if (ground_confirm_start == 0) {
                        ground_confirm_start = now;
                    } else if (now - ground_confirm_start > 1000) {
                        // 1 second confirmed on ground -> GROUND
                        ground_confirm_start = 0;
                        chute_integrator_ms = 0;
                        last_integrator_tick = 0;
                        flushPageBuffer();
                        current_state = STATE_GROUND;
                        state_start_time = now; // start post-landing log timer
                        Serial.println("State -> GROUND (ground detected in RECOVERY).");
                        break;
                    }
                } else {
                    ground_confirm_start = 0;
                }
            }

            // --- Chute confirmation via descent-rate integrator ---
            const uint32_t CHUTE_CONFIRM_MS = 5000;
            static uint32_t chute_integrator_ms = 0;
            static uint32_t last_integrator_tick = 0;

            const uint32_t MIN_CHUTE_DEPLOY_TIME_MS = 3000;
            uint32_t time_in_recovery = now - state_start_time;

            // Elapsed wall-clock time since the previous loop pass.
            if (last_integrator_tick == 0) last_integrator_tick = now;
            uint32_t elapsed = now - last_integrator_tick;
            last_integrator_tick = now;

            bool descent_good = (current_descent_rate < CHUTE_DESCENT_RATE_THRESHOLD);

            // Don't accumulate the integrator if we're too low — a low descent rate
            // at low altitude is likely ground impact, not a deployed chute.
            bool too_low_for_chute = (current_altitude < 50.0f);

            if (descent_good && !too_low_for_chute) {
                if (chute_integrator_ms < CHUTE_CONFIRM_MS) {
                    chute_integrator_ms += elapsed;
                    if (chute_integrator_ms > CHUTE_CONFIRM_MS) chute_integrator_ms = CHUTE_CONFIRM_MS;
                }
                if (chute_integrator_ms >= CHUTE_CONFIRM_MS &&
                    time_in_recovery >= MIN_CHUTE_DEPLOY_TIME_MS) {
                    chute_integrator_ms = 0;
                    current_state = STATE_CHUTE;
                    state_start_time = now;
                    Serial.println("State -> CHUTE (chute confirmed by descent rate).");
                }
            } else {
                if (chute_integrator_ms > (2 * elapsed)) chute_integrator_ms -= 2 * elapsed;
                else chute_integrator_ms = 0;
            }

            // --- Smart panic: time-to-ground based ---
            if (time_in_recovery < MIN_CHUTE_DEPLOY_TIME_MS) break;

            if (system_errors & 2) {
                if (time_in_recovery > 8000 && has_secondary && !ch2_fired) {
                    Serial.println("RECOVERY PANIC: Baro failed, deploying backup channel 2.");
                    deployChannel2();
                    ch2_fired = true;
                    state_start_time = now;
                } else if (time_in_recovery > 8000 && (!has_secondary || ch2_fired)) {
                    Serial.println("RECOVERY PANIC: All channels deployed (baro failed). Proceeding to CHUTE.");
                    current_state = STATE_CHUTE;
                    state_start_time = now;
                }
                break;
            }

            // Normal case: altitude-based panic
            float dr = fabsf(current_descent_rate);
            // 5 m floor prevents any 0-altitude glitch from triggering panic,
            // while still allowing ground-level detection (was 30m — too high).
            float alt_for_calc = fmaxf(current_altitude, 5.0f);
            float time_to_ground_s = alt_for_calc / fmaxf(dr, 0.1f);

            bool should_panic = false;
            if (time_to_ground_s < 3.0f) {
                should_panic = true;
                Serial.printf("RECOVERY PANIC: %.0f m at %.1f m/s (%.1f s left)! ",
                              current_altitude, dr, time_to_ground_s);
            } else if (time_to_ground_s < 5.0f) {
                should_panic = true;
                Serial.printf("RECOVERY PANIC: %.0f m at %.1f m/s (%.1f s to ground). ",
                              current_altitude, dr, time_to_ground_s);
            } else if (time_to_ground_s < 10.0f && time_in_recovery > 6000) {
                should_panic = true;
                Serial.printf("RECOVERY PANIC: %.0f m at %.1f m/s, primary may have failed. ",
                              current_altitude, dr);
            }

            if (should_panic) {
                if (has_secondary && !ch2_fired) {
                    Serial.println("Deploying backup channel 2.");
                    deployChannel2();
                    ch2_fired = true;
                    state_start_time = now;
                } else {
                    Serial.println("All channels deployed. Proceeding to CHUTE.");
                    current_state = STATE_CHUTE;
                    state_start_time = now;
                }
            }
            break;
        }

        case STATE_CHUTE: {
            const float GYRO_STILL_TOLERANCE = 0.4f;
            const float GROUND_ALTITUDE_TOLERANCE = 50.0f;
            const uint32_t CHUTE_PANIC_TIMEOUT_MS = 30000; // 30 s in CHUTE without ground → panic
            static uint32_t chute_entry_time = 0;

            const bool has_ch2 = (RECOVERY_TYPE_ID2 != RECOVERY_TYPE_NONE);

            // Latch the entry time on first visit
            if (chute_entry_time == 0) {
                chute_entry_time = now;
            }

            bool chute_deployed = (current_descent_rate < CHUTE_DESCENT_RATE_THRESHOLD);
            bool gyro_still     = (current_gyro_mag <= GYRO_STILL_TOLERANCE);
            bool low_altitude   = (current_altitude <= GROUND_ALTITUDE_TOLERANCE);

            if (chute_deployed && gyro_still && low_altitude) {
                if (now - state_start_time > GROUND_WAIT_MS) {
                    chute_entry_time = 0;
                    flushPageBuffer();
                    current_state = STATE_GROUND;
                    state_start_time = now; // start post-landing log timer
                    Serial.println("Flight complete (chute descent confirmed).");
                }
            } else {
                // Reset ground timer if conditions aren't met
                state_start_time = now;

                // Panic: in CHUTE too long; deploy any remaining channels.
                if (now - chute_entry_time > CHUTE_PANIC_TIMEOUT_MS) {
                    if (has_ch2 && !ch2_fired) {
                        Serial.println("CHUTE PANIC: Deploying backup channel 2.");
                        deployChannel2();
                        ch2_fired = true;
                    }
                    chute_entry_time = 0;
                    flushPageBuffer();
                    current_state = STATE_GROUND;
                    state_start_time = now; // start post-landing log timer
                    Serial.println("CHUTE PANIC: Forcing GROUND state.");
                }
            }
            break;
        }

        case STATE_GROUND: {
            // Post-landing log timer: keep logging for GROUND_LOG_DURATION_MS,
            // then stop (state transitions to a terminal no-log state).
            static uint32_t ground_entry_time = 0;
            if (ground_entry_time == 0) {
                ground_entry_time = now;
            }
            if (now - ground_entry_time > GROUND_LOG_DURATION_MS) {
                // Post-landing log window expired — flush and stop logging.
                // We stay in STATE_GROUND but the logging gate on core 1 will
                // see state > STATE_CHUTE and stop.
                // Reset ground_entry_time so it doesn't keep flushing.
                ground_entry_time = 0;
            }

            if (buzzer.pattern != BUZZER_SOS) {
                startBuzzerPattern(BUZZER_SOS);
            }
            break;
        }

        case STATE_BOOTING:
            break;
    }
}

// =============================================================================
// [4] CORE 1: HIGH-FREQUENCY SENSOR ACQUISITION & FLASH LOGGING
// =============================================================================

void setup1() {
    delay(5000);

    pinMode(LSM_CS_PIN, OUTPUT);
    digitalWrite(LSM_CS_PIN, HIGH);
    delay(10);

    SPI.setRX(16);
    SPI.setSCK(18);
    SPI.setTX(19);
    SPI.begin();

    digitalWrite(LSM_CS_PIN, LOW);
    delay(5);
    digitalWrite(LSM_CS_PIN, HIGH);
    delay(5);

    if (lsm_dsox.begin_SPI(LSM_CS_PIN)) {
        use_dsox = true;
        lsm_dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
        lsm_dsox.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
        lsm_dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
        lsm_dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    } else {
        digitalWrite(LSM_CS_PIN, HIGH);
        delay(10);

        if (lsm_dsv.begin(LSM_CS_PIN)) {
            use_dsox = false;
            lsm_dsv.deviceReset();
            while (!lsm_dsv.getDeviceReset()) { delay(1); }
            lsm_dsv.enableBlockDataUpdate();
            lsm_dsv.setAccelDataRate(LSM6DSV16X_ODR_AT_1920Hz);
            lsm_dsv.setAccelFullScale(LSM6DSV16X_16g);
            lsm_dsv.setGyroDataRate(LSM6DSV16X_ODR_AT_1920Hz);
            lsm_dsv.setGyroFullScale(LSM6DSV16X_2000dps);
        } else {
            critical_section_enter_blocking(&system_errors_lock);
            system_errors |= 1;
            critical_section_exit(&system_errors_lock);
        }
    }

    Wire.setSDA(BMP_SDA_PIN);
    Wire.setSCL(BMP_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    bool bmp_ok = false;
    for (int attempt = 0; attempt < 3 && !bmp_ok; attempt++) {
        if (attempt > 0) {
            delay(50);
            Wire.end();
            Wire.begin();
            Wire.setClock(400000);
        }
        bmp_ok = bmp.begin_I2C(0x76, &Wire);
    }

    if (!bmp_ok) {
        critical_section_enter_blocking(&system_errors_lock);
        system_errors |= 2;
        critical_section_exit(&system_errors_lock);
    } else {
        bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
        bmp.setOutputDataRate(BMP3_ODR_200_HZ);
    }

    core1_init_complete = true;
}

void loop1() {
    static uint32_t last_sample_micros = 0;
    static int8_t cached_core_temp = 0;

    if (system_errors > 0 && !force_armed) return;

    if (micros() - last_sample_micros >= 1000) {
        if (micros() - last_sample_micros > 10000) last_sample_micros = micros();
        else last_sample_micros += 1000;

        float ax_val = 0.0f, ay_val = 0.0f, az_val = 0.0f;
        float gx_val = 0.0f, gy_val = 0.0f, gz_val = 0.0f;

        if (!(system_errors & 1)) {
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
                sfe_lsm_data_t accelData, gyroData;
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
        }

        float x_mod = IS_UPSIDE_DOWN ? -1.0f : 1.0f;

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
        if (++decimator >= 5) {
            if (!(system_errors & 2)) {
                bmp.performReading();
                current_baro_temp = (int8_t)bmp.temperature;
                current_pressure_pa = (uint32_t)bmp.pressure;
            }
            cached_core_temp = (int8_t)analogReadTemp();
            current_core_temp = cached_core_temp;
            decimator = 0;

            if (current_state == STATE_BOOTING || current_state == STATE_ARMED) {
                float current_hpa = (system_errors & 2) ? reference_pressure_hpa
                                                        : (bmp.pressure / 100.0f);
                reference_pressure_hpa = (reference_pressure_hpa * 0.9f) + (current_hpa * 0.1f);
            }

            float alt = (system_errors & 2) ? 0.0f : bmp.readAltitude(reference_pressure_hpa);
            if (current_altitude == 0.0f) current_altitude = alt;
            else current_altitude = (current_altitude * 0.8f) + (alt * 0.2f);

            if (current_altitude > max_altitude && current_state >= STATE_ACCELERATING) {
                max_altitude = current_altitude;
            }

            {
                static float prev_altitude = 0.0f;
                float delta = prev_altitude - current_altitude;
                // Barometer is read every 5 ms (1 kHz loop, decimator=5).
                // Conversion from delta-per-5ms to m/s: delta * (1000/5) = delta * 200.
                current_descent_rate = (current_descent_rate * 0.7f) + (delta * 200.0f * 0.3f);
                prev_altitude = current_altitude;
            }

            current_bat_voltage = (uint8_t)(analogRead(BAT_ADC_PIN) >> 4);
            current_p1_voltage  = (uint8_t)(analogRead(PYRO1_ADC_PIN) >> 4);
            current_p2_voltage  = (uint8_t)(analogRead(PYRO2_ADC_PIN) >> 4);
        }

        // --- Flash logging (flight + post-landing) ---
        {
            bool should_log = (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE);
            if (!should_log && current_state == STATE_GROUND) {
                // Post-landing logging window: keep recording for GROUND_LOG_DURATION_MS
                static uint32_t ground_log_start_ms = 0;
                if (ground_log_start_ms == 0) {
                    ground_log_start_ms = millis();
                }
                should_log = (millis() - ground_log_start_ms < GROUND_LOG_DURATION_MS);
                if (!should_log) {
                    ground_log_start_ms = 0; // reset for next flight
                }
            }

            if (should_log) {
                LogFrameCore* f = &page_buffer[buffer_index].core;

                f->sync_word    = SYNC_WORD;
                f->timestamp    = millis();
                f->apid         = 0;
                f->flight_state = (uint8_t)current_state;
                {
                    uint32_t half_pct = ((uint64_t)current_flash_addr * 200) / FLIGHT_DATA_FLASH_SIZE;
                    if (half_pct > 200) half_pct = 200;
                    f->flash_used = (uint8_t)half_pct;
                }
                f->core_temp    = cached_core_temp;
                f->ax           = (int16_t)(ax_val * 100);
                f->ay           = (int16_t)(ay_val * 100);
                f->az           = (int16_t)(az_val * 100);
                f->gx           = (int16_t)(gx_val * 1000);
                f->gy           = (int16_t)(gy_val * 1000);
                f->gz           = (int16_t)(gz_val * 1000);
                f->pressure     = current_pressure_pa;
                f->temperature  = (int8_t)bmp.temperature;
                f->bat_voltage  = current_bat_voltage;
                f->p1_voltage   = current_p1_voltage;
                f->p2_voltage   = current_p2_voltage;

                buffer_index++;

                if (buffer_index >= FRAMES_PER_PAGE) {
                    if (current_flash_addr + FLASH_PAGE_SIZE <= FLIGHT_DATA_FLASH_SIZE) {
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
        }
    }

    drainGpsFrame();
}

// --- Drain pending GPS frame into flash page buffer (core 1) ---
void drainGpsFrame() {
    // Atomic copy+clear under the lock so no torn read vs. producer (core 0).
    LogFrameGPS snapped;
    bool have_frame;
    critical_section_enter_blocking(&gps_mailbox_lock);
    if (gps_pending_flag) {
        snapped = gps_pending_frame.gps;
        gps_pending_flag = false;
        have_frame = true;
    } else {
        have_frame = false;
    }
    critical_section_exit(&gps_mailbox_lock);

    if (!have_frame) return;
    // Allow GPS logging during post-landing window (STATE_GROUND)
    if (current_state < STATE_ACCELERATING) return;
    if (current_state > STATE_CHUTE && current_state != STATE_GROUND) return;
    if (buffer_index >= FRAMES_PER_PAGE) return;

    memcpy(&page_buffer[buffer_index].gps, &snapped, sizeof(LogFrameGPS));
    buffer_index++;

    if (buffer_index >= FRAMES_PER_PAGE) {
        if (current_flash_addr + FLASH_PAGE_SIZE <= FLIGHT_DATA_FLASH_SIZE) {
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

// =============================================================================
// [5] RECOVERY DEPLOYMENT FUNCTIONS
// =============================================================================

void deployChannel1() {
    if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_NONE) {
        Serial.println("CH1: channel is NONE — no action.");
        return;
    }

    if (testbench_active) {
        Serial.println("[TESTBENCH] CH1 deployment requested — suppressed (serial only).");
        return;
    }

    if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_SERVO) {
        Serial.printf("CH1 (SERVO1): moving to UNLOCKED angle (%d°).\n", SERVO1_UNLOCKED_ANGLE);
        servo1.write(SERVO1_UNLOCKED_ANGLE);
    } else {
        Serial.println("CH1 (PYRO1): firing pyro.");
        firePyro1();
    }
}

void deployChannel2() {
    if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_NONE) {
        Serial.println("CH2: channel is NONE — no action.");
        return;
    }

    if (testbench_active) {
        Serial.println("[TESTBENCH] CH2 deployment requested — suppressed (serial only).");
        return;
    }

    if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_SERVO) {
        Serial.printf("CH2 (SERVO2): moving to UNLOCKED angle (%d°).\n", SERVO2_UNLOCKED_ANGLE);
        servo2.write(SERVO2_UNLOCKED_ANGLE);
    } else {
        Serial.println("CH2 (PYRO2): firing pyro.");
        firePyro2();
    }
}

void firePyro1() {
    digitalWrite(PYRO1_PIN, HIGH);
    pyro1_fire_start = millis();
    pyro1_active = true;
    Serial.println("PYRO1: fire command executed (3 s burn).");
}

void firePyro2() {
    digitalWrite(PYRO2_PIN, HIGH);
    pyro2_fire_start = millis();
    pyro2_active = true;
    Serial.println("PYRO2: fire command executed (3 s burn).");
}

void setServoAngle(int id, int angle) {
    if (angle < 0 || angle > 180) {
        Serial.printf("SERVO: angle %d out of range (0–180).\n", angle);
        return;
    }

    if (id == 1) {
        if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_NONE) {
            Serial.println("SERVO1: channel is NONE.");
            return;
        }
        if (RECOVERY_TYPE_ID1 != RECOVERY_TYPE_SERVO) {
            Serial.println("SERVO1: channel is configured as PYRO, not SERVO.");
            return;
        }
        if (!servo1_attached) {
            Serial.println("SERVO1: not attached.");
            return;
        }
        servo1.write(angle);
        Serial.printf("SERVO1: angle set to %d°.\n", angle);
    } else if (id == 2) {
        if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_NONE) {
            Serial.println("SERVO2: channel is NONE.");
            return;
        }
        if (RECOVERY_TYPE_ID2 != RECOVERY_TYPE_SERVO) {
            Serial.println("SERVO2: channel is configured as PYRO, not SERVO.");
            return;
        }
        if (!servo2_attached) {
            Serial.println("SERVO2: not attached.");
            return;
        }
        servo2.write(angle);
        Serial.printf("SERVO2: angle set to %d°.\n", angle);
    } else {
        Serial.printf("SERVO: invalid ID %d (use 1 or 2).\n", id);
    }
}

// =============================================================================
// [6] SERIAL COMMAND SYSTEM
// =============================================================================

// Prompt Y/N before a risky action; true if user typed Y/YES.
static bool confirmAction(const char* prompt) {
    Serial.printf("⚠  %s (y/N): ", prompt);
    while (Serial.available()) Serial.read();

    unsigned long timeout = millis() + 30000;
    String response = "";
    while (millis() < timeout) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') break;
            response += c;
        }
        delay(10);
    }
    response.trim();
    response.toUpperCase();

    if (response == "Y" || response == "YES") {
        Serial.println("Y");
        return true;
    }
    Serial.println("N (cancelled)");
    return false;
}

void handleSerialCommands() {
    if (Serial.available() <= 0) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    bool flight_active = (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE);

    // --- RADIO_FREQ <freq> ---
    if (cmd.startsWith("RADIO_FREQ")) {
        float freq = cmd.substring(10).toFloat();
        if (freq >= 400.0f && freq <= 510.0f) {
            radioSetFrequency(freq);
        } else {
            Serial.println("RADIO_FREQ: out of range (400–510 MHz).");
        }
    }
    // --- RADIO_POWER <dBm> ---
    else if (cmd.startsWith("RADIO_POWER")) {
        float dbm = cmd.substring(11).toFloat();
        if (dbm >= -9.0f && dbm <= 22.0f) {
            radioSetPower(dbm);
        } else {
            Serial.println("RADIO_POWER: out of range (-9 to 22 dBm).");
        }
    }
    // --- RADIO_TEST ---
    else if (cmd == "RADIO_TEST") {
        if (radio_ready) {
            Serial.println("RADIO_TEST: transmitting test frame...");
            radioTransmitFrame();
        } else {
            Serial.println("RADIO_TEST: radio not ready.");
        }
    }
    // --- WIPE_FLASH ---
    else if (cmd == "WIPE_FLASH" && !flight_active) {
        if (!confirmAction("WIPE_FLASH will erase ALL flight data. Continue?")) return;
        Serial.println("Flash erase started. 2 MB boot partition protected.");
        leds[0] = CRGB::Red; FastLED.show();

        // Erase in 64 KB chunks for LED progress feedback.
        const uint32_t CHUNK_SIZE = 64 * 1024;
        uint32_t remaining = FLIGHT_DATA_FLASH_SIZE;
        uint32_t offset = 0;
        uint32_t total_chunks = (FLIGHT_DATA_FLASH_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE;
        uint32_t chunk_count = 0;

        while (remaining > 0) {
            uint32_t erase_size = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

            rp2040.idleOtherCore();
            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(flight_flash_offset + offset, erase_size);
            restore_interrupts(ints);
            rp2040.resumeOtherCore();

            leds[0] = (chunk_count % 2 == 0) ? CRGB::Red : CRGB::Black;
            FastLED.show();

            offset += erase_size;
            remaining -= erase_size;
            chunk_count++;
            Serial.printf("Progress: %d%%\n", (chunk_count * 100) / total_chunks);
        }
        leds[0] = CRGB::Red; FastLED.show(); // solid red when done
        current_flash_addr = 0;
        buffer_index = 0;
        memset(page_buffer, 0, sizeof(page_buffer));
        Serial.println("Erase complete.");
    }
    // --- DUMP_FLASH ---
    else if (cmd == "DUMP_FLASH" && !flight_active) {
        Serial.println("DUMP_START");
        uint8_t* flash_ptr = (uint8_t*)(XIP_BASE + flight_flash_offset);
        for (uint32_t i = 0; i < current_flash_addr; i += 256) {
            Serial.write(flash_ptr + i, 256);
        }
        Serial.println("\nDUMP_END");
    }
    // --- DISARM ---
    else if (cmd == "DISARM") {
        if (current_state == STATE_DISARMED) {
            Serial.println("SYSTEM: Already DISARMED.");
        } else {
            current_state = STATE_DISARMED;
            state_start_time = millis();
            ch1_fired = false;
            ch2_fired = false;
            // Return servos to locked position
            if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_SERVO && servo1_attached) {
                servo1.write(SERVO1_LOCKED_ANGLE);
            }
            if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_SERVO && servo2_attached) {
                servo2.write(SERVO2_LOCKED_ANGLE);
            }
            Serial.println("SYSTEM: State -> DISARMED. All flight systems stopped.");
            startBuzzerPattern(BUZZER_IDLE);
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
    // --- ARM ---
    else if (cmd == "ARM") {
        if (current_state == STATE_DISARMED) {
            current_state = STATE_ARMED;
            state_start_time = millis();
            max_altitude = 0.0f;
            current_altitude = 0.0f;
            ch1_fired = false;
            ch2_fired = false;
            buffer_index = 0;
            memset(page_buffer, 0, sizeof(page_buffer));
            Serial.println("SYSTEM: State -> ARMED (exited DISARMED).");
            startBuzzerPattern(BUZZER_BOOT_BEEPS);
        } else if (current_state == STATE_ERROR) {
            if (!confirmAction("ARM from ERROR will force-start with failed sensors. Continue?")) return;
            force_armed = true;
            current_state = STATE_ARMED;
            state_start_time = millis();
            max_altitude = 0.0f;
            current_altitude = 0.0f;
            ch1_fired = false;
            ch2_fired = false;
            buffer_index = 0;
            memset(page_buffer, 0, sizeof(page_buffer));
            Serial.println("SYSTEM: State -> ARMED (force_armed, zeroing failed sensors).");
            startBuzzerPattern(BUZZER_BOOT_BEEPS);
        } else {
            Serial.println("Command ignored: Can only ARM from DISARMED or ERROR.");
        }
    }
    // --- SET_STATE <state_name> ---
    else if (cmd.startsWith("SET_STATE")) {
        String arg = cmd.substring(9);
        arg.trim();
        arg.toUpperCase();

        FlightState new_state = STATE_BOOTING;
        bool found = false;

        if (arg == "BOOTING")       { new_state = STATE_BOOTING;       found = true; }
        else if (arg == "DISARMED") { new_state = STATE_DISARMED;      found = true; }
        else if (arg == "ERROR")    { new_state = STATE_ERROR;         found = true; }
        else if (arg == "ARMED")    { new_state = STATE_ARMED;         found = true; }
        else if (arg == "ACCEL")    { new_state = STATE_ACCELERATING;  found = true; }
        else if (arg == "ACCELERATING") { new_state = STATE_ACCELERATING; found = true; }
        else if (arg == "COAST")    { new_state = STATE_COAST;         found = true; }
        else if (arg == "RECOVERY") { new_state = STATE_RECOVERY;      found = true; }
        else if (arg == "CHUTE")    { new_state = STATE_CHUTE;         found = true; }
        else if (arg == "GROUND")   { new_state = STATE_GROUND;        found = true; }

        if (found) {
            current_state = new_state;
            state_start_time = millis();
            Serial.printf("SYSTEM: State manually set to %s.\n", STATE_NAMES[new_state]);
        } else {
            Serial.printf("SET_STATE: unknown state '%s'. Options: BOOTING, DISARMED, ERROR, ARMED, ACCELERATING, COAST, RECOVERY, CHUTE, GROUND\n", arg.c_str());
        }
    }
    // --- SERVO <id> <angle> ---
    else if (cmd.startsWith("SERVO")) {
        // Format: SERVO 1 90  or  SERVO 2 45
        int id = 0, angle = 0;
        if (sscanf(cmd.c_str(), "SERVO %d %d", &id, &angle) == 2) {
            setServoAngle(id, angle);
        } else {
            Serial.println("SERVO: usage — SERVO <1|2> <angle 0–180>");
        }
    }
    // --- SERVO1_LOCK ---
    else if (cmd == "SERVO1_LOCK") {
        setServoAngle(1, SERVO1_LOCKED_ANGLE);
    }
    // --- SERVO1_UNLOCK ---
    else if (cmd == "SERVO1_UNLOCK") {
        setServoAngle(1, SERVO1_UNLOCKED_ANGLE);
    }
    // --- SERVO2_LOCK ---
    else if (cmd == "SERVO2_LOCK") {
        setServoAngle(2, SERVO2_LOCKED_ANGLE);
    }
    // --- SERVO2_UNLOCK ---
    else if (cmd == "SERVO2_UNLOCK") {
        setServoAngle(2, SERVO2_UNLOCKED_ANGLE);
    }
    // --- P1_FIRE ---
    else if (cmd == "P1_FIRE") {
        if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_PYRO) {
            if (confirmAction("P1_FIRE will fire pyro channel 1. Continue?")) {
                firePyro1();
            }
        } else {
            Serial.println("P1_FIRE: channel 1 is SERVO, not PYRO. Use SERVO commands.");
        }
    }
    // --- P2_FIRE ---
    else if (cmd == "P2_FIRE") {
        if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_PYRO) {
            if (confirmAction("P2_FIRE will fire pyro channel 2. Continue?")) {
                firePyro2();
            }
        } else {
            Serial.println("P2_FIRE: channel 2 is SERVO, not PYRO. Use SERVO commands.");
        }
    }
    // --- STATUS ---
    else if (cmd == "STATUS") {
        printStatus();
    }
    // --- CHECKLIST ---
    else if (cmd == "CHECKLIST") {
        printChecklist();
    }
    // --- RESET_ARMED ---
    else if (cmd == "RESET_ARMED") {
        current_state = STATE_ARMED;
        max_altitude = 0.0f;
        current_altitude = 0.0f;
        ch1_fired = false;
        ch2_fired = false;
        Serial.println("SYSTEM: State reset to ARMED.");
    }
    // --- BUZZER_ON ---
    else if (cmd == "BUZZER_ON") {
        if (testbench_active) {
            // Testbench: use PWM at low frequency, ~4% duty
            analogWrite(BUZZER_PIN, 10); // 10/255 ≈ 4%
        } else {
            digitalWrite(BUZZER_PIN, HIGH);
        }
        Serial.println("BUZZER ON");
    }
    // --- BUZZER_OFF ---
    else if (cmd == "BUZZER_OFF") {
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("BUZZER OFF");
    }
    // --- BUZZER_TEST ---
    else if (cmd == "BUZZER_TEST") {
        Serial.println("BUZZER TEST: executing boot beep pattern.");
        startBuzzerPattern(BUZZER_BOOT_BEEPS);
    }
    // --- REBOOT ---
    else if (cmd == "REBOOT") {
        if (!confirmAction("REBOOT will restart the microcontroller. Continue?")) return;
        Serial.println("REBOOT: system restarting...");
        Serial.flush();
        delay(100);
        rp2040.reboot();
    }
    // --- UF2_BOOT ---
    else if (cmd == "UF2_BOOT") {
        if (!confirmAction("UF2_BOOT will enter USB bootloader mode. Continue?")) return;
        Serial.println("UF2_BOOT: entering USB mass-storage bootloader...");
        Serial.flush();
        delay(200);
        // Reboot into UF2 mass-storage bootloader using SDK bootrom call
        reset_usb_boot(0, 0);
    }
    // --- HELP ---
    else if (cmd == "HELP") {
        Serial.println("\n==============================================");
        Serial.println("       RAPIDFISH v2 — COMMAND REFERENCE");
        Serial.println("==============================================");
        Serial.println("--- State Control ---");
        Serial.println("  SET_STATE <name>     Force any flight state");
        Serial.println("  ARM                  Enter ARMED (from DISARMED or ERROR)");
        Serial.println("  DISARM               Enter safe serial-only mode");
        Serial.println("  RESET_ARMED          Reset to ARMED (post-flight)");
        Serial.println("  REBOOT               Reboot the microcontroller");
        Serial.println("  UF2_BOOT             Enter USB mass-storage bootloader (drag .uf2)");
        Serial.println();
        Serial.println("--- Recovery Channels ---");
        Serial.println("  SERVO <id> <angle>   Set servo angle (id=1|2, angle=0–180)");
        Serial.println("  SERVO1_LOCK          Servo 1 → locked angle");
        Serial.println("  SERVO1_UNLOCK        Servo 1 → unlocked angle");
        Serial.println("  SERVO2_LOCK          Servo 2 → locked angle");
        Serial.println("  SERVO2_UNLOCK        Servo 2 → unlocked angle");
        Serial.println("  P1_FIRE              Fire pyro channel 1 (3 s burn)");
        Serial.println("  P2_FIRE              Fire pyro channel 2 (3 s burn)");
        Serial.println();
        Serial.println("--- Radio ---");
        Serial.println("  RADIO_FREQ <MHz>     Set frequency (400–510)");
        Serial.println("  RADIO_POWER <dBm>    Set TX power (-9 to 22)");
        Serial.println("  RADIO_TEST           Transmit a test frame");
        Serial.println();
        Serial.println("--- Data & Diagnostics ---");
        Serial.println("  STATUS               Full system status report");
        Serial.println("  CHECKLIST            Pre-flight checklist");
        Serial.println("  WIPE_FLASH           Erase all flight data");
        Serial.println("  DUMP_FLASH           Dump flight data via serial");
        Serial.println();
        Serial.println("--- Buzzer ---");
        Serial.println("  BUZZER_ON            Turn buzzer on");
        Serial.println("  BUZZER_OFF           Turn buzzer off");
        Serial.println("  BUZZER_TEST          Run boot beep pattern");
        Serial.println();
        Serial.println("--- General ---");
        Serial.println("  HELP                 Print this list");
        Serial.println("==============================================\n");
    }
}

// =============================================================================
// [7] RADIO TELEMETRY (LR2021 LoRa on SPI1)
// =============================================================================

void radioInit() {
    if (!RADIO_ENABLED) {
        Serial.println(F("[LR2021] Radio disabled via config. Skipping init."));
        return;
    }

    SPI1.setRX(12);
    SPI1.setSCK(14);
    SPI1.setTX(15);
    SPI1.begin();

    radio.tcxoVoltage = 2.7;
    radio.irqDioNum = 5;

    Serial.print(F("[LR2021] Initializing (LoRa implicit header) ... "));
    ConfigLoRa_t config;
    config.frequency = RADIO_FREQUENCY_MHZ;
    config.bandwidth = 125.0;
    config.spreadingFactor = 6;
    config.codingRate = 5;
    int state = radio.begin(config);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print(F("failed, code "));
        Serial.println(state);
        radio_ready = false;
        radio_error_code = state;
        critical_section_enter_blocking(&system_errors_lock);
        system_errors |= 4;
        critical_section_exit(&system_errors_lock);
        return;
    }
    Serial.println(F("success!"));

    radio.setFrequency(RADIO_FREQUENCY_MHZ);
    radio.setBandwidth(250.0);
    radio.setSpreadingFactor(7);
    radio.setCodingRate(5);
    radio.setPreambleLength(8);
    radio.implicitHeader(28);
    radio.setSyncWord(0x12);
    radio.setCRC(true);
    int pw_state = radio.setOutputPower(10.0);

    radio_current_power_dbm = (pw_state == RADIOLIB_ERR_NONE) ? 10.0f : -999.0f;
    radio_ready = true;
    radioApplyStatePower();
    radio.clearPacketSentAction();
    radio.setPacketSentAction(radioTxDoneISR);
    Serial.printf("[LR2021] Radio ready @ %.1f MHz\n", RADIO_FREQUENCY_MHZ);
}

bool radioTransmitFrame() {
    LogFrameCore f;
    memset(&f, 0, sizeof(f));

    f.sync_word    = SYNC_WORD;
    f.timestamp    = millis();
    f.apid         = 0;
    f.flight_state = (uint8_t)current_state;
    {
        uint32_t half_pct = ((uint64_t)current_flash_addr * 200) / FLIGHT_DATA_FLASH_SIZE;
        if (half_pct > 200) half_pct = 200;
        f.flash_used = (uint8_t)half_pct;
    }
    f.core_temp    = current_core_temp;
    f.ax           = (int16_t)(current_accel_x * 9.81f * 100.0f);
    f.ay           = (int16_t)(current_accel_y * 9.81f * 100.0f);
    f.az           = (int16_t)(current_accel_z * 9.81f * 100.0f);
    f.gx           = (int16_t)(current_gyro_x * 1000.0f);
    f.gy           = (int16_t)(current_gyro_y * 1000.0f);
    f.gz           = (int16_t)(current_gyro_z * 1000.0f);
    f.pressure     = current_pressure_pa;
    f.temperature  = current_baro_temp;
    f.bat_voltage  = current_bat_voltage;
    f.p1_voltage   = current_p1_voltage;
    f.p2_voltage   = current_p2_voltage;

    return radioTransmit((const uint8_t*)&f, sizeof(LogFrameCore));
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

void radioSetPower(float dbm) {
    if (!radio_ready) return;
    if (dbm == radio_current_power_dbm) return;
    if (radio_tx_busy) return;
    int state = radio.setOutputPower(dbm);
    if (state == RADIOLIB_ERR_NONE) {
        radio_current_power_dbm = dbm;
        Serial.printf("[LR2021] Power set to %.1f dBm\n", dbm);
    } else if (state == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        Serial.printf("[LR2021] setOutputPower(%.1f) out of range (-9..22 dBm), ignored.\n", dbm);
    } else {
        static uint32_t last_err = 0;
        if (millis() - last_err > 1000) {
            last_err = millis();
            Serial.printf("[LR2021] setOutputPower(%.1f) failed, code %d\n", dbm, state);
        }
    }
}

void radioApplyStatePower() {
    if (!radio_ready) return;
    float dbm;
    if (testbench_active) {
        dbm = TESTBENCH_RADIO_POWER_DBM;
    } else if (current_state == STATE_ARMED) {
        dbm = RADIO_POWER_ARMED_DBM;
    } else {
        dbm = RADIO_POWER_HIGH_DBM;
    }
    radioSetPower(dbm);
}

bool radioTransmitGpsFrame() {
    union LogFrame g;
    gpsFrameFill(&g.gps);
    return radioTransmit(g.bytes, sizeof(LogFrameGPS));
}

void radioTxDoneISR() {
    radio_tx_done = true;
}

// Rolling 10 s TX success report: prints success % + sends/sec over the current
// window once a second, and rolls the window (reset counters) every 10 s of REAL
// wall-clock time. Uses time_us_64() for both the rate-limit and the window so a
// flash write that freezes millis() on this core cannot skew the elapsed/rate
// labels or delay the window reset.
static void radioReportTxStatus() {
    // Testbench-only diagnostic; resets the window when inactive.
    if (!testbench_active) {
        win_start_us = time_us_64();
        win_ok = 0;
        win_dropped = 0;
        return;
    }

    static uint64_t last_report_us = 0;
    uint64_t now_us = time_us_64();
    if (now_us - last_report_us < 1000000ull) return;   // ~1 report/sec
    last_report_us = now_us;

    uint64_t elapsed_us = now_us - win_start_us;
    if (elapsed_us > 10000000ull) elapsed_us = 10000000ull;   // cap at 10 s

    // win_attempted = ok + dropped (true submit + busy-drop cadence).
    uint32_t win_attempted = win_ok + win_dropped;
    uint32_t succeeded      = win_ok;
    float success_pct = (win_attempted > 0) ? (100.0f * (float)succeeded / (float)win_attempted) : 100.0f;
    float sends_per_sec = (elapsed_us > 0) ? ((float)win_attempted * 1000000.0f / (float)elapsed_us) : 0.0f;

    Serial.printf("[LR2021] TX: %.1f%% success (%lu ok, %lu drop) ~%.0f/s over %.1fs\n",
                  success_pct,
                  (unsigned long)succeeded,
                  (unsigned long)win_dropped,
                  sends_per_sec,
                  (double)(elapsed_us / 1000000.0f));

    if (now_us - win_start_us >= 10000000ull) {
        // Roll the window: reset BOTH counters together on true 10 s elapsed.
        win_ok = 0;
        win_dropped = 0;
        win_start_us = now_us;
    }
}

static bool radioTransmit(const uint8_t* frame, size_t len) {
    radioReportTxStatus();

    if (!radio_ready || radio_tx_busy || len < (4 + 28)) {
        if (radio_ready && radio_tx_busy) {
            radio_tx_dropped++;
            if (testbench_active) win_dropped++;
        }
        return false;
    }
    memcpy(radio_tx_buf, &frame[4], 28);
    radio_tx_len = 28;

    // Stamp busy-start only on the true not-busy->busy edge so a re-submit
    // can't reset the watchdog clock and evade the timeout.
    radio_tx_busy = true;
    radio_tx_busy_start_ms = millis();
    radio_tx_busy_start_us = time_us_64();
    radio_tx_done = false;
    radio_tx_attempted++;
    if (testbench_active) win_ok++;
    int state = radio.startTransmit(radio_tx_buf, radio_tx_len);
    if (state != RADIOLIB_ERR_NONE) {
        radio_tx_busy = false;
        static uint32_t last_err_print = 0;
        if (millis() - last_err_print > 1000) {
            last_err_print = millis();
            Serial.printf("[LR2021] TX start failed, code %d\n", state);
        }
        return false;
    }
    return true;
}

// =============================================================================
// [8] BUZZER PATTERN GENERATOR (non-blocking)
// =============================================================================

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
            if (testbench_active) {
                // Quieter PWM in testbench (~4% duty)
                analogWrite(BUZZER_PIN, (cycle < 50) ? 10 : 0);
            } else {
                digitalWrite(BUZZER_PIN, (cycle < 50) ? HIGH : LOW);
            }
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
            if (testbench_active) {
                analogWrite(BUZZER_PIN, (cycle % 2 == 0) ? 10 : 0);
            } else {
                digitalWrite(BUZZER_PIN, (cycle % 2 == 0) ? HIGH : LOW);
            }
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
            if (testbench_active) {
                analogWrite(BUZZER_PIN, (cycle % 2 == 0) ? 10 : 0);
            } else {
                digitalWrite(BUZZER_PIN, (cycle % 2 == 0) ? HIGH : LOW);
            }
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

            if (testbench_active) {
                analogWrite(BUZZER_PIN, (step % 2 == 0) ? 10 : 0);
            } else {
                digitalWrite(BUZZER_PIN, (step % 2 == 0) ? HIGH : LOW);
            }
            break;
        }

        default:
            buzzer.pattern = BUZZER_IDLE;
            digitalWrite(BUZZER_PIN, LOW);
            break;
    }
}

// =============================================================================
// [9] GPS (ATGM336H via TinyGPSPlus on core 0)
// =============================================================================

void gpsInit() {
    Serial2.setRX(GPS_RX_PIN);
    Serial2.setTX(GPS_TX_PIN);
    Serial2.begin(GPS_BAUD);
    configureAtgm336h();
    Serial.printf("[GPS] Initialized on pins TX=%d RX=%d @ %lu baud\n",
                  GPS_TX_PIN, GPS_RX_PIN, (unsigned long)GPS_BAUD);
}

void gpsSnapshotCapture(uint32_t now) {
    gps_snapshot.fix_time_ms = now;
    gps_snapshot.fix_valid   = tinyGPS.location.isValid() ? 1 : 0;
    gps_snapshot.lat_e7      = tinyGPS.location.isValid()
        ? (int32_t)(tinyGPS.location.lat() * 10000000.0) : 0;
    gps_snapshot.lon_e7      = tinyGPS.location.isValid()
        ? (int32_t)(tinyGPS.location.lng() * 10000000.0) : 0;
    gps_snapshot.alt_half_m  = tinyGPS.altitude.isValid()
        ? (uint16_t)(tinyGPS.altitude.meters() * 2.0) : 0;
    {
        const bool dateOk = tinyGPS.date.isValid();
        const int  cy     = tinyGPS.date.year();
        const int  cm     = tinyGPS.date.month();
        const int  cd     = tinyGPS.date.day();
        const bool sane   = (cy >= 1970 && cm >= 1 && cm <= 12 && cd >= 1 && cd <= 31);

        if (dateOk && sane) {
            gps_last_good_year  = cy;
            gps_last_good_month = cm;
            gps_last_good_day   = cd;
        }

        const int y = (dateOk && sane) ? cy : gps_last_good_year;
        const int m = (dateOk && sane) ? cm : gps_last_good_month;
        const int d = (dateOk && sane) ? cd : gps_last_good_day;

        if (tinyGPS.time.isValid() && gps_last_good_year >= 1970) {
            const int hh = tinyGPS.time.hour();
            const int mm = tinyGPS.time.minute();
            const int ss = tinyGPS.time.second();
            uint32_t days = 0;
            for (int yr = 1970; yr < y; ++yr) {
                days += 365;
                if ((yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0)) days += 1;
            }
            static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
            for (int mo = 0; mo < m - 1; ++mo) days += mdays[mo];
            const bool leap = (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
            if (m > 2 && leap) days += 1;
            days += (uint32_t)(d - 1);
            gps_snapshot.gps_time =
                days * 86400UL + (uint32_t)(hh * 3600 + mm * 60 + ss);
        } else {
            gps_snapshot.gps_time = 0;
        }
    }
    gps_snapshot.hdop_x10    = tinyGPS.hdop.isValid()
        ? (uint8_t)((tinyGPS.hdop.hdop() * 10.0f) > 255.0f
            ? 255 : (uint8_t)(tinyGPS.hdop.hdop() * 10.0f)) : 0;
    gps_snapshot.sats        = tinyGPS.satellites.isValid()
        ? (uint8_t)tinyGPS.satellites.value() : 0;
    gps_snapshot.update_cnt += 1;
}

void handleGps() {
    while (Serial2.available() > 0) {
        gps_raw_bytes++;
        tinyGPS.encode((char)Serial2.read());
    }

    gps_valid_sentences = tinyGPS.passedChecksum();
    gps_bad_checksum    = tinyGPS.failedChecksum();

    uint32_t now = millis();

    if (tinyGPS.location.isUpdated() || tinyGPS.satellites.isUpdated() ||
        tinyGPS.altitude.isUpdated() || tinyGPS.speed.isUpdated() ||
        tinyGPS.hdop.isUpdated()) {
        gps_last_nmea_ms = now;
        gps_ready = true;
        gpsSnapshotCapture(now);
    }

    if (gpsHasUsableLock()) {
        gps_last_valid_fix_ms = now;
    }

    static uint16_t last_emitted_cnt = 0;
    if (gps_snapshot.update_cnt != last_emitted_cnt) {
        last_emitted_cnt = gps_snapshot.update_cnt;
        emitGpsFrame();
    }
}

bool gpsHasUsableLock() {
    if (tinyGPS.location.isValid() &&
        tinyGPS.location.age() < GPS_FIX_FRESH_MS) {
        return true;
    }
    return false;
}

void gpsLockWatchdog() {
    if (millis() - gps_last_restart_ms < GPS_WATCHDOG_COOLDOWN_MS) return;
    if (gps_last_valid_fix_ms == 0) return;
    if (millis() - gps_last_valid_fix_ms < GPS_LOCK_LOSS_TIMEOUT_MS) return;

    sendPcasCmd("PCAS00");
    sendPcasCmd("PCAS10,0");
    gps_last_restart_ms = millis();
    gps_last_valid_fix_ms = millis();
    Serial.println("[GPS] Lock lost, hot restart.");
}

void gpsFrameFill(LogFrameGPS* f) {
    f->sync_word = SYNC_WORD;
    f->timestamp = millis();
    f->apid      = 1;
    f->lat       = gps_snapshot.lat_e7;
    f->lon       = gps_snapshot.lon_e7;
    f->gps_alt   = gps_snapshot.alt_half_m;
    f->state     = gps_snapshot.fix_valid;
    f->sats      = gps_snapshot.sats;
    f->gps_time  = gps_snapshot.gps_time;
    f->hdop      = gps_snapshot.hdop_x10;
    f->mx = f->my = f->mz = 0;
}

void emitGpsFrame() {
    // Write the frame then set the pending flag under the lock so the consumer
    // (core 1) can never observe/memcpy a partially-written frame, and can never
    // be overwriting while it is copying out.
    critical_section_enter_blocking(&gps_mailbox_lock);
    gpsFrameFill(&gps_pending_frame.gps);
    gps_pending_flag = true;
    critical_section_exit(&gps_mailbox_lock);
}

uint8_t nmeaChecksum(const char* payload) {
    uint8_t ck = 0;
    for (const char* p = payload; *p && *p != '\r' && *p != '\n'; ++p)
        ck ^= (uint8_t)*p;
    return ck;
}

void sendPcasCmd(const char* payload) {
    const char hex[] = "0123456789ABCDEF";
    char frame[40];
    uint8_t ck = nmeaChecksum(payload);
    int n = snprintf(frame, sizeof(frame), "$%s*%c%c\r\n",
                     payload, hex[(ck >> 4) & 0x0F], hex[ck & 0x0F]);
    Serial2.write(frame, n);
    Serial2.flush();
    Serial.printf("[GPS] TX: %s", frame);
}

void configureAtgm336h() {
    sendPcasCmd("PCAS04,3");
    delay(100);
    sendPcasCmd("PCAS02,100");
    delay(100);
    sendPcasCmd("PCAS01,5");
    delay(100);
    Serial.println("[GPS] ATGM336H configured: GPS+BDS @ 10 Hz @ 115200 baud.");
}

// =============================================================================
// [10] FLASH SCANNING
// =============================================================================

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

// =============================================================================
// [11] STATUS REPORT
// =============================================================================

void printStatus() {
    float bat_v = (current_bat_voltage * 9.9f) / 255.0f;
    float p1_v  = (current_p1_voltage * 9.9f) / 255.0f;
    float p2_v  = (current_p2_voltage * 9.9f) / 255.0f;

    Serial.println("\n==============================================");
    Serial.println("         RAPIDFISH v2 — STATUS REPORT");
    Serial.println("==============================================");
    Serial.printf("Flight State  : %s\n", STATE_NAMES[current_state]);
    Serial.printf("Uptime        : %lu ms\n", millis());
    Serial.printf("Testbench     : %s\n", testbench_active ? "ACTIVE" : "OFF");
    Serial.printf("Core Temp     : %d °C\n", current_core_temp);
    Serial.printf("Baro Temp     : %d °C\n", current_baro_temp);
    Serial.println("----------------------------------------------");
    Serial.printf("Altitude      : %.2f m  (Max: %.2f m)", current_altitude, max_altitude);
    if (current_state == STATE_ARMED || current_state == STATE_BOOTING) {
        Serial.printf("  [0 in ARMED/BOOTING — ref. pressure stops updating on launch]");
    }
    Serial.println();
    Serial.printf("Descent Rate  : %.2f m/s\n", current_descent_rate);
    Serial.printf("Accel         : X:%.2f G | Y:%.2f G | Z:%.2f G\n",
                  current_accel_x, current_accel_y, current_accel_z);
    Serial.printf("Gyro Mag      : %.2f rad/s\n", current_gyro_mag);
    Serial.println("----------------------------------------------");
    Serial.printf("Main Battery  : %.2f V\n", bat_v);
    if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_SERVO) {
        Serial.printf("Ch1 (SERVO)   : %d°  (locked=%d°, unlocked=%d°)\n",
                      servo1_attached ? servo1.read() : -1,
                      SERVO1_LOCKED_ANGLE, SERVO1_UNLOCKED_ANGLE);
    } else if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_PYRO) {
        Serial.printf("Ch1 (PYRO)    : %.2f V  (continuity ADC: %u)\n", p1_v, current_p1_voltage);
    } else {
        Serial.println("Ch1 (NONE)    : no recovery hardware");
    }
    if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_SERVO) {
        Serial.printf("Ch2 (SERVO)   : %d°  (locked=%d°, unlocked=%d°)\n",
                      servo2_attached ? servo2.read() : -1,
                      SERVO2_LOCKED_ANGLE, SERVO2_UNLOCKED_ANGLE);
    } else if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_PYRO) {
        Serial.printf("Ch2 (PYRO)    : %.2f V  (continuity ADC: %u)\n", p2_v, current_p2_voltage);
    } else {
        Serial.println("Ch2 (NONE)    : no recovery hardware");
    }
    Serial.println("----------------------------------------------");
    if (tinyGPS.location.isValid()) {
        Serial.printf("GPS Fix       : VALID (%d sats)\n",
                      tinyGPS.satellites.isValid() ? tinyGPS.satellites.value() : 0);
        Serial.printf("Latitude      : %.6f\n", tinyGPS.location.lat());
        Serial.printf("Longitude     : %.6f\n", tinyGPS.location.lng());
        if (tinyGPS.altitude.isValid())
            Serial.printf("GPS Alt (MSL) : %.1f m\n", tinyGPS.altitude.meters());
        if (tinyGPS.hdop.isValid())
            Serial.printf("GPS HDOP      : %.1f\n", tinyGPS.hdop.hdop());
        Serial.printf("GPS Speed     : %.1f km/h\n",
                      tinyGPS.speed.isValid() ? tinyGPS.speed.kmph() : 0.0f);
    } else {
        Serial.printf("GPS Fix       : NO FIX (%s)\n",
                      gps_ready ? "waiting for lock" : "no data");
    }
    Serial.printf("GPS Link      : %lu raw | %lu valid | %lu bad checksum\n",
                  (unsigned long)gps_raw_bytes,
                  (unsigned long)gps_valid_sentences,
                  (unsigned long)gps_bad_checksum);
    Serial.println("----------------------------------------------");
    Serial.printf("Flash Usage   : %u / %u bytes (%u%%)\n",
                  current_flash_addr, FLIGHT_DATA_FLASH_SIZE,
                  (current_flash_addr * 100) / FLIGHT_DATA_FLASH_SIZE);
    Serial.printf("Radio         : %s\n", radio_ready ? "READY" : "FAILED");
    Serial.printf("System Errors : 0x%02X\n", system_errors);
    Serial.println("==============================================\n");
}

// =============================================================================
// [12] PRE-FLIGHT CHECKLIST (interactive — press ENTER between sections)
// =============================================================================

// Helper: wait for a single ENTER from the user, drain any buffered input.
static void waitForContinue() {
    Serial.println("  --- Press ENTER to continue ---");
    while (Serial.available()) Serial.read();
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') break;
        }
        delay(10);
    }
}

void printChecklist() {
    Serial.println("\n==============================================");
    Serial.println("       PRE-FLIGHT CHECKLIST");
    Serial.println("==============================================\n");

    // --- 1. Sensor detection ---
    Serial.println("[1/9] SENSOR DETECTION");
    Serial.printf("  IMU (LSM6DSO32/LSM6DSV16X) : %s\n",
                  (system_errors & 1) ? "FAILED ✗" : "DETECTED ✓");
    Serial.printf("  Barometer (BMP390)         : %s\n",
                  (system_errors & 2) ? "FAILED ✗" : "DETECTED ✓");
    Serial.printf("  Radio (LR2021)             : %s\n",
                  (system_errors & 4) ? "FAILED ✗" : "DETECTED ✓");
    Serial.printf("  GPS (ATGM336H)             : %s\n",
                  gps_ready ? "DETECTED ✓" : "NO DATA (still acquiring)");
    waitForContinue();

    // --- 2. Orientation check (X-axis acceleration) ---
    Serial.println("[2/9] ORIENTATION CHECK");
    Serial.printf("  X-axis acceleration : %.2f G\n", current_accel_x);
    if (current_accel_x >= 0.9f && current_accel_x <= 1.1f) {
        Serial.println("  ✓ Rocket is vertical, IS_UPSIDE_DOWN is correct.");
    } else if (current_accel_x >= -1.1f && current_accel_x <= -0.9f) {
        Serial.println("  ✗ CRITICAL: X-axis reads ~-1 G but IS_UPSIDE_DOWN is set to "
                        + String(IS_UPSIDE_DOWN ? "false" : "true") + ".");
        Serial.println("    If the rocket is nose-up on the rail, set IS_UPSIDE_DOWN to "
                        + String(IS_UPSIDE_DOWN ? "true" : "false") + ".");
        Serial.println("    If IS_UPSIDE_DOWN is already correct, the rocket may be upside-down on the rail.");
    } else {
        Serial.println("  ✗ CRITICAL: X-axis reads " + String(current_accel_x, 2) + " G — not ~±1 G.");
        Serial.println("    The rocket is NOT vertical on the launch rail, OR IS_UPSIDE_DOWN is wrong.");
        Serial.println("    Check that the rocket is standing upright and IS_UPSIDE_DOWN matches orientation.");
    }
    waitForContinue();

    // --- 3. Recovery channel configuration ---
    Serial.println("[3/9] RECOVERY CHANNELS");
    Serial.printf("  Channel 1 (ID1): %s\n",
                  (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_SERVO) ? "SERVO" :
                  (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_PYRO)  ? "PYRO"  : "NONE");
    if (RECOVERY_TYPE_ID1 == RECOVERY_TYPE_SERVO) {
        Serial.printf("    Locked angle  : %d°\n", SERVO1_LOCKED_ANGLE);
        Serial.printf("    Unlocked angle: %d°\n", SERVO1_UNLOCKED_ANGLE);
    }
    Serial.printf("  Channel 2 (ID2): %s\n",
                  (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_SERVO) ? "SERVO" :
                  (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_PYRO)  ? "PYRO"  : "NONE");
    if (RECOVERY_TYPE_ID2 == RECOVERY_TYPE_SERVO) {
        Serial.printf("    Locked angle  : %d°\n", SERVO2_LOCKED_ANGLE);
        Serial.printf("    Unlocked angle: %d°\n", SERVO2_UNLOCKED_ANGLE);
    }
    waitForContinue();

    // --- 4. Flight thresholds ---
    Serial.println("[4/9] FLIGHT THRESHOLDS");
    Serial.printf("  Launch G-force threshold : %.1f G\n", LAUNCH_G_THRESHOLD);
    Serial.printf("  Burnout G-force threshold: %.1f G\n", BURNOUT_G_THRESHOLD);
    Serial.printf("  Apogee dip detection     : %.1f m\n", APOGEE_DIP_METERS);
    Serial.printf("  Ground G tolerance       : %.1f G\n", GROUND_G_TOLERANCE);
    Serial.printf("  Orientation (IS_UPSIDE_DOWN): %s\n", IS_UPSIDE_DOWN ? "true" : "false");
    waitForContinue();

    // --- 5. Timing thresholds ---
    Serial.println("[5/9] TIMING THRESHOLDS");
    Serial.printf("  Min motor burn : %lu ms\n", (unsigned long)MIN_MOTOR_BURN_MS);
    Serial.printf("  Max motor burn : %lu ms\n", (unsigned long)MAX_MOTOR_BURN_MS);
    Serial.printf("  Recovery delay : %lu ms\n", (unsigned long)RECOVERY_DELAY_MS);
    Serial.printf("  Ground wait    : %lu ms\n", (unsigned long)GROUND_WAIT_MS);
    waitForContinue();

    // --- 6. Pyro thresholds (if applicable) ---
    Serial.println("[6/9] PYRO THRESHOLDS");
    Serial.printf("  Continuity threshold : %u (ADC)\n", PYRO_CONTINUITY_THRESHOLD);
    Serial.printf("  Fire duration        : %lu ms\n", (unsigned long)PYRO_FIRE_DURATION_MS);
    Serial.printf("  Redeploy timeout     : %lu ms\n", (unsigned long)PYRO_REDEPLOY_TIMEOUT_MS);
    Serial.printf("  Chute descent rate   : %.1f m/s\n", CHUTE_DESCENT_RATE_THRESHOLD);
    waitForContinue();

    // --- 7. Radio configuration ---
    Serial.println("[7/9] RADIO CONFIGURATION");
    Serial.printf("  Frequency : %.1f MHz\n", RADIO_FREQUENCY_MHZ);
    Serial.printf("  ARMED power : %.1f dBm\n", RADIO_POWER_ARMED_DBM);
    Serial.printf("  Flight power: %.1f dBm\n", RADIO_POWER_HIGH_DBM);
    Serial.printf("  ARMED beacon interval : %lu ms\n", (unsigned long)RADIO_ARMED_INTERVAL_MS);
    Serial.printf("  Flight TX interval    : %lu ms\n", (unsigned long)RADIO_TX_INTERVAL_MS);
    Serial.printf("  Radio hardware        : %s\n", radio_ready ? "READY ✓" : "FAILED ✗");
    waitForContinue();

    // --- 8. GPS configuration ---
    Serial.println("[8/9] GPS CONFIGURATION");
    Serial.printf("  Baud rate     : %lu\n", (unsigned long)GPS_BAUD);
    Serial.printf("  Lock loss timeout : %lu ms\n", (unsigned long)GPS_LOCK_LOSS_TIMEOUT_MS);
    Serial.printf("  Watchdog cooldown : %lu ms\n", (unsigned long)GPS_WATCHDOG_COOLDOWN_MS);
    Serial.printf("  GPS hardware      : %s\n", gps_ready ? "DETECTED ✓" : "NO DATA");
    waitForContinue();

    // --- 9. Testbench mode ---
    Serial.println("[9/9] TESTBENCH MODE");
    Serial.printf("  Testbench mode : %s\n", testbench_active ? "ACTIVE ⚠" : "OFF ✓");
    if (testbench_active) {
        Serial.println("  ⚠ WARNING: Testbench mode is ON. Do not fly!");
        Serial.println("  Recovery outputs are replaced with serial text.");
        Serial.println("  Radio power is reduced.");
    }
    waitForContinue();

    // --- Summary ---
    Serial.println("----------------------------------------------");
    if (system_errors == 0 && gps_ready) {
        Serial.println("  ✓ ALL SYSTEMS NOMINAL — READY FOR FLIGHT");
    } else if (system_errors == 0 && !gps_ready) {
        Serial.println("  ⚠ All sensors OK, but GPS not yet locked.");
        Serial.println("  Wait for GPS fix before flight, or proceed without.");
    } else {
        Serial.println("  ✗ SYSTEM ERRORS DETECTED — DO NOT FLY");
        Serial.println("  Check sensor failures above. Use ARM to force-start.");
    }
    Serial.println("==============================================\n");
}