#include <pico.h>
#include <stdint.h>

#define RS_MAX_PARITY 28   // supports up to 100% overhead on a 28-byte payload
#define RS_MAX_MSG    64

static uint8_t rs_gf_exp[512];
static uint8_t rs_gf_log[256];

static uint8_t rs_gpoly[RS_MAX_PARITY + 1];
static uint8_t rs_mult[RS_MAX_PARITY][256];
static int     rs_parity = 0;

static uint8_t rs_gf_mul(uint8_t a, uint8_t b) {
    if (a == 0 || b == 0) return 0;
    return rs_gf_exp[rs_gf_log[a] + rs_gf_log[b]];
}

static void rsFecInit(void) {
    rs_gf_log[1] = 0;
    uint16_t x = 1;
    for (int i = 0; i < 255; i++) {
        rs_gf_exp[i] = x;
        rs_gf_log[x] = i;
        x <<= 1;
        if (x & 0x100) x ^= 0x187;   // primitive poly x^8+x^7+x^2+x+1
    }
    rs_gf_exp[255] = rs_gf_exp[0];
    for (int i = 256; i < 512; i++) rs_gf_exp[i] = rs_gf_exp[i - 255];
}

static void rsFecSetParity(int parity) {
    if (parity < 0 || parity > RS_MAX_PARITY) parity = 0;
    rs_parity = parity;

    for (int i = 0; i <= parity; i++) rs_gpoly[i] = 0;
    rs_gpoly[0] = 1;
    for (int i = 0; i < parity; i++) {
        uint8_t root = rs_gf_exp[((112 + i) * 11) % 255];
        for (int j = i + 1; j > 0; j--) {
            rs_gpoly[j] = rs_gpoly[j - 1] ^ rs_gf_mul(rs_gpoly[j], root);
        }
        rs_gpoly[0] = rs_gf_mul(rs_gpoly[0], root);
    }

    for (int j = 0; j < parity; j++) {
        uint8_t coeff = rs_gpoly[(parity - 1) - j];
        for (int val = 0; val < 256; val++) {
            rs_mult[j][val] = rs_gf_mul(coeff, val);
        }
    }
}

static void __not_in_flash_func(rsFecEncode)(const uint8_t* msg,
                                             uint8_t* parity, int msg_len) {
    int P = rs_parity;
    if (P <= 0) return;
    for (int i = 0; i < P; i++) parity[i] = 0;

    for (int i = 0; i < msg_len; i++) {
        uint8_t fb = msg[i] ^ parity[0];
        if (fb != 0) {
            for (int j = 0; j < P - 1; j++) {
                parity[j] = parity[j + 1] ^ rs_mult[j][fb];
            }
            parity[P - 1] = rs_mult[P - 1][fb];
        } else {
            for (int j = 0; j < P - 1; j++) {
                parity[j] = parity[j + 1];
            }
            parity[P - 1] = 0;
        }
    }
}

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <FastLED.h>
#include <Adafruit_LSM6DSO32.h>
#include "SparkFun_LSM6DSV16X.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <RadioLib.h>
#include <SPI.h>
#include <TinyGPSPlus.h>

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
const bool  IS_UPSIDE_DOWN       = true;

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
const uint32_t SYNC_WORD = 0x1ACFFC1D;              

// --- Radio Telemetry (LR2021 GFSK on SPI1) ---
const bool RADIO_ENABLED = true; 
#define RADIO_CS_PIN    13
#define RADIO_IRQ_PIN   6    
#define RADIO_RST_PIN   11
#define RADIO_BUSY_PIN  10

#define RADIO_FREQUENCY_MHZ 434.0f
#define RADIO_TX_INTERVAL_MS           100   // flight: CORE every 100 ms
#define RADIO_ARMED_INTERVAL_MS        5000  // ARMED: one frame every 5 s (CORE/GPS interleaved)
#define RADIO_GROUND_CORE_INTERVAL_MS 10000  // GROUND: CORE every 10 s
#define RADIO_GROUND_GPS_INTERVAL_MS   1000  // GROUND: GPS every 1 s
#define RADIO_POWER_ARMED_DBM          10.0f // ARMED output power
#define RADIO_POWER_HIGH_DBM           22.0f // flight + GROUND output power
#define RADIO_RS_PARITY                28    // 0/7/14/28 = no FEC / 25% / 50% / 100% overhead

// --- Reed-Solomon FEC (over-the-air only; flash frames carry no parity) ---
// RS encodes the 28-byte payload. Options:
// 0/7/14/28 parity = no FEC / 25% / 50% / 100% overhead.
#ifndef RADIO_RS_PARITY
#define RADIO_RS_PARITY 7   // 25% overhead default; options 0/7/14/28
#endif

// --- GPS (ATGM336H, TinyGPSPlus on UART1) ---
// If pins are reversed on the module, swap GPS_TX_PIN / GPS_RX_PIN below.
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9

// ATGM336H is configured to 115200 baud NMEA.
#define GPS_BAUD 115200

// GPS lock-loss watchdog (core 0). Ordering: GPS_FIX_FRESH_MS <
// GPS_LOCK_LOSS_TIMEOUT_MS < GPS_WATCHDOG_COOLDOWN_MS (prevents restart loop).
#define GPS_LOCK_LOSS_TIMEOUT_MS 30000  // 30 s without a usable fix -> restart
#define GPS_WATCHDOG_COOLDOWN_MS 60000  // 60 s min between restarts (> timeout)

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

// --- APID 1: GPS Frame (32 bytes); field scaling documented inline ---
struct __attribute__((packed)) LogFrameGPS {
    uint32_t sync_word;   // +0  (4) SyncWord (0x1ACFFC1D)
    uint32_t timestamp;   // +4  (4) millis()
    uint8_t  apid;        // +8  (1) = 1 (gps)
    int32_t  lat;         // +9  (4) Latitude  * 1e7   (SIGNED, negative coords ok)
    int32_t  lon;         // +13 (4) Longitude * 1e7   (SIGNED)
    uint16_t gps_alt;     // +17 (2) MSL altitude in 0.5 m units (meters*2)
    uint8_t  state;       // +19 (1) GPS fix state (1 = valid)
    uint8_t  sats;        // +20 (1) Satellite count
    uint32_t gps_time;    // +21 (4) Unix timestamp (seconds since epoch, UTC)
    uint8_t  hdop;        // +25 (1) HDOP * 10 (clamped to 255)
    int16_t  mx, my, mz;  // +26 (6) Magnetometer raw (placeholder 0, for later)
};
static_assert(sizeof(LogFrameGPS) == 32, "LogFrameGPS must be 32 bytes");

union LogFrame {
    LogFrameCore core;
    LogFrameGPS  gps;
    uint8_t      bytes[32];
};
static_assert(sizeof(union LogFrame) == 32, "LogFrame union must be 32 bytes");

CRGB leds[NUM_LEDS];
Adafruit_LSM6DSO32 lsm_dsox;
SparkFun_LSM6DSV16X_SPI lsm_dsv;
bool use_dsox = false;
Adafruit_BMP3XX bmp;

LR2021 radio = new Module(RADIO_CS_PIN, RADIO_IRQ_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI1);
bool radio_ready = false;
// Interrupt-driven TX state; radio_tx_buf must stay valid until DIO IRQ fires.
static uint8_t radio_tx_buf[64];       // room for 32-byte frame + 28 parity
static size_t  radio_tx_len = 0;       // length handed to startTransmit
volatile bool  radio_tx_busy = false;  // a TX is in flight
volatile bool  radio_tx_done = false;  // DIO IRQ fired for the current TX
// State-based TX cadence timers (ARMED interleave / flight / GROUND GPS+CORE).
uint32_t last_radio_tx      = 0; // shared interleave timer (ARMED)
uint32_t last_radio_core_tx = 0; // CORE-frame timer (flight / GROUND)
uint32_t last_gps_tx        = 0; // GPS-frame timer (GROUND)
uint32_t last_gps_tx_cnt    = 0; // last GPS snapshot update_cnt sent (flight)
// ARMED alternation: core,gps,core,gps; flips each armed tick.
volatile bool beacon_is_gps = false;

// GPS instance and decode state (TinyGPSPlus). Processed on core 0.
TinyGPSPlus tinyGPS;
bool gps_ready = false;
uint32_t gps_last_nmea_ms = 0;
// Last usable-fix / last-restart times (feed lock-loss watchdog).
uint32_t gps_last_valid_fix_ms = 0;
uint32_t gps_last_restart_ms   = 0;

// GPS diagnostics: raw bytes vs checksummed NMEA sentences.
volatile uint32_t gps_raw_bytes      = 0;  // every byte drained from Serial2
volatile uint32_t gps_valid_sentences = 0; // NMEA sentences that passed checksum
volatile uint32_t gps_bad_checksum    = 0; // NMEA sentences that failed checksum

// Flash memory state tracking
uint32_t flight_flash_offset = FIRMWARE_RESERVED_SIZE;
volatile uint32_t current_flash_addr = 0;
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
// Serializes cross-core system_errors |= via sysErrSetBits().
static spin_lock_t* sys_err_lock = NULL;
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

// GPS snapshot from core 0 to frame builders; update_cnt written last => coherent.
struct GpsSnapshot {
    volatile uint32_t fix_time_ms; // millis() when last fix was captured
    volatile uint8_t  fix_valid;   // 1 = valid lat/lon fix
    volatile int32_t  lat_e7;      // degrees * 1e7 (signed)
    volatile int32_t  lon_e7;      // degrees * 1e7 (signed)
    volatile uint16_t alt_half_m;  // MSL altitude in 0.5 m units (meters*2)
    volatile uint32_t gps_time;    // Unix timestamp (seconds since epoch, UTC)
    volatile uint8_t  hdop_x10;    // HDOP * 10 (0..255, saturating)
    volatile uint8_t  sats;        // satellite count
    volatile uint16_t update_cnt;  // monotonic, incremented per fresh fix
};
GpsSnapshot gps_snapshot;

// Latch of last-good UTC date; keeps epoch valid if RMC date is momentarily bad.
static int gps_last_good_year  = 0;
static int gps_last_good_month = 0;
static int gps_last_good_day   = 0;

// Fill a LogFrameGPS (APID 1) from the shared snapshot.
void gpsFrameFill(LogFrameGPS* f) {
    f->sync_word = SYNC_WORD;
    f->timestamp = millis();
    f->apid      = 1;
    f->lat       = gps_snapshot.lat_e7;      // degrees * 1e7 (signed)
    f->lon       = gps_snapshot.lon_e7;      // degrees * 1e7 (signed)
    f->gps_alt   = gps_snapshot.alt_half_m;  // MSL altitude, 0.5 m units (m*2)
    f->state     = gps_snapshot.fix_valid;   // 1 = valid fix
    f->sats      = gps_snapshot.sats;        // satellite count
    f->gps_time  = gps_snapshot.gps_time;    // Unix seconds since epoch (UTC)
    f->hdop      = gps_snapshot.hdop_x10;    // HDOP * 10 (already clamped)
    // Magnetometer placeholder for a future mag sensor; 0 for now.
    f->mx = f->my = f->mz = 0;
}

// Ping-pong GPS buffer (core 0 -> core 1): slot N&1, publish gps_seq after write.
union LogFrame    gps_pending_frame[2];
volatile uint16_t gps_seq = 0;

// CCSDS 8-bit randomizer (x^8 + x^7 + x^5 + x^3 + 1)
// 255-byte table, repeats every 255 bytes
static constexpr uint8_t CCSDS_RANDOMIZER[255] = {
    0xFF, 0x48, 0x0E, 0xC0, 0x9A, 0x0D, 0x70, 0xBC, 0x8E, 0x2C, 0x93, 0xAD, 0xA7, 0xB7, 0x46, 0xCE,
    0x5A, 0x97, 0x7D, 0xCC, 0x32, 0xA2, 0xBF, 0x3E, 0x0A, 0x10, 0xF1, 0x88, 0x94, 0xCD, 0xEA, 0xB1,
    0xFE, 0x90, 0x1D, 0x81, 0x34, 0x1A, 0xE1, 0x79, 0x1C, 0x59, 0x27, 0x5B, 0x4F, 0x6E, 0x8D, 0x9C,
    0xB5, 0x2E, 0xFB, 0x98, 0x65, 0x45, 0x7E, 0x7C, 0x14, 0x21, 0xE3, 0x11, 0x29, 0x9B, 0xD5, 0x63,
    0xFD, 0x20, 0x3B, 0x02, 0x68, 0x35, 0xC2, 0xF2, 0x38, 0xB2, 0x4E, 0xB6, 0x9E, 0xDD, 0x1B, 0x39,
    0x6A, 0x5D, 0xF7, 0x30, 0xCA, 0x8A, 0xFC, 0xF8, 0x28, 0x43, 0xC6, 0x22, 0x53, 0x37, 0xAA, 0xC7,
    0xFA, 0x40, 0x76, 0x04, 0xD0, 0x6B, 0x85, 0xE4, 0x71, 0x64, 0x9D, 0x6D, 0x3D, 0xBA, 0x36, 0x72,
    0xD4, 0xBB, 0xEE, 0x61, 0x95, 0x15, 0xF9, 0xF0, 0x50, 0x87, 0x8C, 0x44, 0xA6, 0x6F, 0x55, 0x8F,
    0xF4, 0x80, 0xEC, 0x09, 0xA0, 0xD7, 0x0B, 0xC8, 0xE2, 0xC9, 0x3A, 0xDA, 0x7B, 0x74, 0x6C, 0xE5,
    0xA9, 0x77, 0xDC, 0xC3, 0x2A, 0x2B, 0xF3, 0xE0, 0xA1, 0x0F, 0x18, 0x89, 0x4C, 0xDE, 0xAB, 0x1F,
    0xE9, 0x01, 0xD8, 0x13, 0x41, 0xAE, 0x17, 0x91, 0xC5, 0x92, 0x75, 0xB4, 0xF6, 0xE8, 0xD9, 0xCB,
    0x52, 0xEF, 0xB9, 0x86, 0x54, 0x57, 0xE7, 0xC1, 0x42, 0x1E, 0x31, 0x12, 0x99, 0xBD, 0x56, 0x3F,
    0xD2, 0x03, 0xB0, 0x26, 0x83, 0x5C, 0x2F, 0x23, 0x8B, 0x24, 0xEB, 0x69, 0xED, 0xD1, 0xB3, 0x96,
    0xA5, 0xDF, 0x73, 0x0C, 0xA8, 0xAF, 0xCF, 0x82, 0x84, 0x3C, 0x62, 0x25, 0x33, 0x7A, 0xAC, 0x7F,
    0xA4, 0x07, 0x60, 0x4D, 0x06, 0xB8, 0x5E, 0x47, 0x16, 0x49, 0xD6, 0xD3, 0xDB, 0xA3, 0x67, 0x2D,
    0x4B, 0xBE, 0xE6, 0x19, 0x51, 0x5F, 0x9F, 0x05, 0x08, 0x78, 0xC4, 0x4A, 0x66, 0xF5, 0x58,
};

// XOR payload [4,len) with CCSDS_RANDOMIZER[(i-4)%255]; ASM bytes 0..3 untouched.
static void ccsdsRandomize(uint8_t* buf, size_t len) {
    for (size_t i = 4; i < len; i++) {
        buf[i] ^= CCSDS_RANDOMIZER[(i - 4) % 255];
    }
}

// Decoder (e.g. tool.py) de-scrambles bytes 4..N, then uses the (28, 28+P)
// shortened RS code to verify/correct the payload (see rsFecEncode above).
// IRQ completion handler; finishTransmit() is deferred to loop() (SPI-safe).
void radioTxDoneISR() {
    radio_tx_done = true;
}

// Interrupt-driven TX on a COPY (source stays raw for flash): append RS parity
// over payload [4,32) if enabled, CCSDS-scramble [4,total), then startTransmit.
static bool radioTransmitRandomized(const uint8_t* frame, size_t len) {
    if (!radio_ready || radio_tx_busy || len <= 4) return false;
    if (len > 32) len = 32;      // FEC is defined over the 28-byte payload
    memcpy(radio_tx_buf, frame, len);

    size_t total = len;
    if (RADIO_RS_PARITY > 0 && len >= (4 + 28)) {
        rsFecEncode(&radio_tx_buf[4], &radio_tx_buf[len], 28); // parity over payload 4..31
        total = len + RADIO_RS_PARITY;
    }
    ccsdsRandomize(radio_tx_buf, total); // bytes 0..3 (ASM) preserved over the air
    radio_tx_len = total;

    radio_tx_busy = true;
    radio_tx_done = false;
    int state = radio.startTransmit(radio_tx_buf, radio_tx_len);
    if (state != RADIOLIB_ERR_NONE) {
        radio_tx_busy = false;   // never held forever on a start failure
        static uint32_t last_err_print = 0;
        if (millis() - last_err_print > 1000) {
            last_err_print = millis();
            Serial.printf("[LR2021] TX start failed, code %d\n", state);
        }
        return false;
    }
    return true;
}

// Queue a GPS frame for the core 1 flash log.
void emitGpsFrame() {
    gpsFrameFill(&gps_pending_frame[gps_seq & 1].gps);
    gps_seq++; // publish after write
}

// Cached output power; -999 sentinel => first set is always sent to the chip.
static float radio_current_power_dbm = -999.0f;

// Set TX power; cached (only on change) and deferred while a TX is in flight
// (LR2021 rejects power config mid-transmission with -706).
void radioSetPower(float dbm) {
    if (!radio_ready) return;
    if (dbm == radio_current_power_dbm) return;  // no change -> no SPI traffic
    if (radio_tx_busy) return;                   // defer until this TX finishes
    int state = radio.setOutputPower(dbm);
    if (state == RADIOLIB_ERR_NONE) {
        radio_current_power_dbm = dbm;
    } else if (state == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        // Benign: requested value outside the validated range. Never spam.
        Serial.printf("[LR2021] setOutputPower(%.1f) out of range (-9..22 dBm), ignored\n", dbm);
    } else {
        // Any other error (e.g. -706) is a real command rejection - log throttled.
        static uint32_t last_err = 0;
        if (millis() - last_err > 1000) {
            last_err = millis();
            Serial.printf("[LR2021] setOutputPower(%.1f) failed, code %d\n", dbm, state);
        }
    }
}

// Apply state-dependent power; no-op unless the cached value changed.
void radioApplyStatePower() {
    if (!radio_ready) return;
    float dbm = (current_state == STATE_ARMED) ? RADIO_POWER_ARMED_DBM : RADIO_POWER_HIGH_DBM;
    radioSetPower(dbm);
}

// Transmit a GPS frame (GROUND 1 s slot / ARMED interleave slot).
bool radioTransmitGpsFrame() {
    union LogFrame g;
    gpsFrameFill(&g.gps);
    return radioTransmitRandomized(g.bytes, sizeof(LogFrameGPS));
}

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
bool radioTransmitFrame();
void radioSetFrequency(float mhz);
void radioSetPower(float dbm);
bool radioTransmitGpsFrame();
void radioTxDoneISR();
void gpsInit();
void handleGps();
void configureAtgm336h();
void sendPcasCmd(const char* payload);
void gpsSnapshotCapture(uint32_t now);
void gpsFrameFill(LogFrameGPS* f);
void emitGpsFrame();
void drainGpsFrame();
bool gpsHasUsableLock();
void gpsLockWatchdog();

// ============================================================================
// [3] CORE 0: STATE MACHINE & SYSTEM MANAGEMENT
// ============================================================================

// Atomic cross-core system_errors |= (see sys_err_lock).
static void sysErrSetBits(uint8_t mask) {
    uint32_t save = save_and_disable_interrupts();
    spin_lock_unsafe_blocking(sys_err_lock);
    system_errors |= mask;
    spin_unlock_unsafe(sys_err_lock);
    restore_interrupts(save);
}

void setup() {
    sys_err_lock = spin_lock_init(0);
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

    // Init RS FEC tables for RADIO_RS_PARITY (0/7/14/28 = no/25/50/100% overhead).
    rsFecInit();
    rsFecSetParity(RADIO_RS_PARITY);
    if (RADIO_RS_PARITY > 0) {
        Serial.printf("[RS] FEC enabled: parity=%d (25%%=7, 50%%=14, 100%%=28)\n",
                      RADIO_RS_PARITY);
    } else {
        Serial.println("[RS] FEC disabled (raw 32-byte frames).");
    }

    radioInit();
    gpsInit();

    Serial.println("Waiting for sensors to initialize...");
    while (!core1_init_complete) { delay(10); }

    if (system_errors > 0) {
        current_state = STATE_ERROR;
        Serial.println("\n*** BOOT FAILURE: SENSOR/RADIO ERROR ***");
        if (system_errors & 1) Serial.println("- IMU (LSM6DSO32/LSM6DSV16X) failed to initialize.");
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
    handleGps();                       // core 0: drain UART2 into TinyGPSPlus
    gpsLockWatchdog();                 // core 0: hot-restart on prolonged lock loss
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
        uint16_t now_cnt = (uint16_t)gps_snapshot.update_cnt;

        // Defer finishTransmit() to loop() (not ISR) to keep the ISR SPI-free.
        if (radio_tx_busy && radio_tx_done) {
            radio.finishTransmit();
            radio_tx_busy = false;
            radio_tx_done = false;
        }

        // State power (cached, deferred while TX in flight); no per-loop SPI flood.
        radioApplyStatePower();

        if (current_state == STATE_ARMED) {
            // ARMED: interleave CORE / GPS every 5 s @ 10 dBm; advance only on send.
            if (current_time - last_radio_tx >= RADIO_ARMED_INTERVAL_MS) {
                bool sent = beacon_is_gps ? radioTransmitGpsFrame()
                                          : radioTransmitFrame();
                if (sent) {
                    last_radio_tx = current_time;
                    beacon_is_gps = !beacon_is_gps; // flip for next armed tick
                }
            }
        } else if (current_state >= STATE_ACCELERATING && current_state <= STATE_CHUTE) {
            // Flight: CORE every 100 ms + GPS on each fresh fix, @ 22 dBm.
            // GPS preempts CORE; advance each timer only on a real send.
            bool gps_pending = (now_cnt != last_gps_tx_cnt);
            if (gps_pending && !radio_tx_busy) {
                if (radioTransmitGpsFrame()) {
                    last_gps_tx_cnt = now_cnt;
                }
            }
            if (current_time - last_radio_core_tx >= RADIO_TX_INTERVAL_MS &&
                !radio_tx_busy) {
                if (radioTransmitFrame()) last_radio_core_tx = current_time;
            }
        } else if (current_state == STATE_GROUND) {
            // GROUND: CORE every 10 s + GPS every 1 s, @ 22 dBm.
            if (current_time - last_radio_core_tx >= RADIO_GROUND_CORE_INTERVAL_MS) {
                if (radioTransmitFrame()) last_radio_core_tx = current_time;
            }
            if (current_time - last_gps_tx >= RADIO_GROUND_GPS_INTERVAL_MS) {
                if (radioTransmitGpsFrame()) last_gps_tx = current_time;
            }
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
                state_start_time = current_time; // start the backup redeploy window
                break;
            }

            // Primary fired: stay until the redeploy window elapses.
            if (pyro1_fired || pyro2_fired) {
                uint32_t since_fire = current_time - state_start_time;
                bool backup_available = (p2_has_cont && !pyro2_fired);

                if (since_fire > PYRO_REDEPLOY_TIMEOUT_MS && backup_available) {
                    digitalWrite(PYRO2_PIN, HIGH);
                    pyro2_fire_start = current_time;
                    pyro2_active = true;
                    pyro2_fired = true;
                    Serial.println("PYRO2 fired (backup deployment, no chute detected).");
                    state_start_time = current_time;
                    break;
                }

                if (since_fire > PYRO_REDEPLOY_TIMEOUT_MS &&
                    !p2_has_cont && !pyro2_fired) {
                    Serial.println("WARN: No backup pyro available, awaiting chute/ground passively.");
                }

                if (since_fire > PYRO_REDEPLOY_TIMEOUT_MS) {
                    current_state = STATE_CHUTE;
                    state_start_time = current_time;
                    Serial.println("State -> CHUTE (awaiting chute detection).");
                }
            } else {
                // No pyro fired: stand by briefly, then proceed.
                if (current_time - state_start_time > RECOVERY_DELAY_MS) {
                    current_state = STATE_CHUTE;
                    state_start_time = current_time;
                    Serial.println("State -> CHUTE (awaiting chute detection).");
                }
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
        lsm_dsox.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
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

    // BMP390 probe with bounded retry (marginal power/clock can spuriously fail).
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
        sysErrSetBits(2);
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

    drainGpsFrame();
}

// Drain pending core 0 GPS frame into the flash page buffer (core 1).
void drainGpsFrame() {
    static uint16_t last_drained = 0;

    const uint16_t avail = (uint16_t)gps_seq;
    if (avail == last_drained) return;

    if (current_state < STATE_ACCELERATING || current_state > STATE_CHUTE) {
        last_drained = avail; // skip outside the flight window
        return;
    }

    if (buffer_index >= FRAMES_PER_PAGE) {
        last_drained = avail;
        return; // should not happen; page full
    }

    const uint8_t slot = (uint8_t)((avail - 1) & 1);
    memcpy(&page_buffer[buffer_index].gps, &gps_pending_frame[slot].gps, sizeof(LogFrameGPS));
    buffer_index++;
    last_drained = avail;

    if (buffer_index >= FRAMES_PER_PAGE) {
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
            if (tinyGPS.location.isValid()) {
                Serial.printf("GPS Fix      : VALID (%d sats)\n", tinyGPS.satellites.isValid() ? tinyGPS.satellites.value() : 0);
                Serial.printf("Latitude     : %.6f\n", tinyGPS.location.lat());
                Serial.printf("Longitude    : %.6f\n", tinyGPS.location.lng());
                // GPS alt is MSL (informational); barometer is source of truth.
                if (tinyGPS.altitude.isValid()) {
                    Serial.printf("GPS Alt (MSL): %.1f m\n", tinyGPS.altitude.meters());
                } else {
                    Serial.printf("GPS Alt (MSL): n/a (no valid altitude)\n");
                }
                // HDOP from GGA; keep GGA enabled.
                if (tinyGPS.hdop.isValid()) {
                    Serial.printf("GPS HDOP     : %.1f\n", tinyGPS.hdop.hdop());
                } else {
                    Serial.printf("GPS HDOP     : n/a\n");
                }
                Serial.printf("GPS Speed    : %.1f km/h\n", tinyGPS.speed.isValid() ? tinyGPS.speed.kmph() : 0.0f);
            } else {
                Serial.printf("GPS Fix      : NO FIX (%s)\n",
                              gps_ready ? "waiting for lock" : "no data");
            }
            // GPS link diag: raw bytes vs checksummed sentence counts.
            Serial.printf("GPS Link     : %lu raw bytes | %lu valid sentences | %lu bad checksum\n",
                          (unsigned long)gps_raw_bytes,
                          (unsigned long)gps_valid_sentences,
                          (unsigned long)gps_bad_checksum);
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
        sysErrSetBits(4);
        return;
    }
    Serial.println(F("success!"));

    radio.setFrequency(RADIO_FREQUENCY_MHZ);
    // TX and RX must use identical settings for decode (modulation index ~1.0).
    radio.setBitRate(24.0);             // 24 kbit/s
    radio.setFrequencyDeviation(12.0);  // 12 kHz -> modulation index ~1.0
    radio.setRxBandwidth(58.6);         // nearest 27-entry LUT step above required ~48 kHz
    int pw_state = radio.setOutputPower(10.0);
    radio.setDataShaping(RADIOLIB_SHAPING_1_0);
    radio.setPreambleLength(64);
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);

    uint8_t syncWord[] = {0x1A, 0xCF, 0xFC, 0x1D};
    radio.setSyncWord(syncWord, 4);
    radio.setCRC(0);

    // Adopt initial power (10 dBm) into the power cache.
    radio_current_power_dbm = (pw_state == RADIOLIB_ERR_NONE) ? 10.0f : -999.0f;
    radio_ready = true;
    radioApplyStatePower();
    // DIO interrupt for non-blocking TX completion (LR2021 -> setPacketSentAction).
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
    f.altitude     = (uint16_t)(current_altitude * 2.0f);
    f.temperature  = current_baro_temp;
    f.bat_voltage  = current_bat_voltage;
    f.p1_voltage   = current_p1_voltage;
    f.p2_voltage   = current_p2_voltage;

    // Flight-active core transmit (also the core beacon slot): randomized copy.
    return radioTransmitRandomized((const uint8_t*)&f, sizeof(LogFrameCore));
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

// ============================================================================
// [7] GPS (ATGM336H via TinyGPSPlus on core 0)
// ============================================================================
//
// ATGM336H NMEA on UART2 (pins 8/9); all GPS work here on core 0.

void gpsInit() {
    // Pins 8/9 = UART2 on the RP2350 core.
    Serial2.setRX(GPS_RX_PIN);
    Serial2.setTX(GPS_TX_PIN);
    Serial2.begin(GPS_BAUD);

    // Configure the ATGM336H via its native $PCAS commands: select GPS+BDS
    // ($PCAS04,3) and raise the update rate to 10 Hz ($PCAS02,100 = 100 ms).
    configureAtgm336h();

    Serial.printf("[GPS] Initialized on pins TX=%d RX=%d @ %lu baud\n",
                  GPS_TX_PIN, GPS_RX_PIN, (unsigned long)GPS_BAUD);
}

// Snapshot parsed GPS into gps_snapshot; update_cnt written last for coherence.
void gpsSnapshotCapture(uint32_t now) {
    gps_snapshot.fix_time_ms = now;
    gps_snapshot.fix_valid   = tinyGPS.location.isValid() ? 1 : 0;
    gps_snapshot.lat_e7      = tinyGPS.location.isValid()
        ? (int32_t)(tinyGPS.location.lat() * 10000000.0) : 0;
    gps_snapshot.lon_e7      = tinyGPS.location.isValid()
        ? (int32_t)(tinyGPS.location.lng() * 10000000.0) : 0;
    // MSL altitude in 0.5 m units (m*2), 0 when invalid.
    gps_snapshot.alt_half_m  = tinyGPS.altitude.isValid()
        ? (uint16_t)(tinyGPS.altitude.meters() * 2.0) : 0;
    // Unix epoch from UTC date/time; fall back to latched last-good date.
    {
        const bool dateOk = tinyGPS.date.isValid();
        const int  cy     = tinyGPS.date.year();
        const int  cm     = tinyGPS.date.month();
        const int  cd     = tinyGPS.date.day();
        const bool sane   = (cy >= 1970 && cm >= 1 && cm <= 12 && cd >= 1 && cd <= 31);

        // Refresh the last-good date latch whenever we have a current, sane one.
        if (dateOk && sane) {
            gps_last_good_year  = cy;
            gps_last_good_month = cm;
            gps_last_good_day   = cd;
        }

        // Pick the date to use: current if sane, else the latched one.
        const int y = (dateOk && sane) ? cy : gps_last_good_year;
        const int m = (dateOk && sane) ? cm : gps_last_good_month;
        const int d = (dateOk && sane) ? cd : gps_last_good_day;

        if (tinyGPS.time.isValid() && gps_last_good_year >= 1970) {
            const int hh = tinyGPS.time.hour();
            const int mm = tinyGPS.time.minute();
            const int ss = tinyGPS.time.second();
            // Days since 1970-01-01 (leap-aware).
            uint32_t days = 0;
            for (int yr = 1970; yr < y; ++yr) {
                days += 365;
                if ((yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0)) days += 1;
            }
            // Days within the year, zero-based month table (leap-corrected below).
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
    // HDOP * 10, saturated into a single byte (0..255).
    gps_snapshot.hdop_x10    = tinyGPS.hdop.isValid()
        ? (uint8_t)((tinyGPS.hdop.hdop() * 10.0f) > 255.0f
            ? 255 : (uint8_t)(tinyGPS.hdop.hdop() * 10.0f)) : 0;
    gps_snapshot.sats        = tinyGPS.satellites.isValid()
        ? (uint8_t)tinyGPS.satellites.value() : 0;
    gps_snapshot.update_cnt += 1; // write LAST
}

// Feed incoming GPS bytes into TinyGPSPlus (10 Hz, core 0).
void handleGps() {
    while (Serial2.available() > 0) {
        gps_raw_bytes++;
        tinyGPS.encode((char)Serial2.read());
    }

    // Track NMEA checksum pass/fail counters.
    gps_valid_sentences = tinyGPS.passedChecksum();
    gps_bad_checksum    = tinyGPS.failedChecksum();

    uint32_t now = millis();

    // Mark GPS live + refresh the shared snapshot on any fresh sentence.
    if (tinyGPS.location.isUpdated() || tinyGPS.satellites.isUpdated() ||
        tinyGPS.altitude.isUpdated() || tinyGPS.speed.isUpdated() ||
        tinyGPS.hdop.isUpdated()) {
        gps_last_nmea_ms = now;
        gps_ready = true;
        gpsSnapshotCapture(now);
    }

    // Refresh last-usable-fix stamp for the lock-loss watchdog.
    if (gpsHasUsableLock()) {
        gps_last_valid_fix_ms = now;
    }

    // Persist one GPS frame per fresh fix via emitGpsFrame().
    static uint16_t last_emitted_cnt = 0;
    if (gps_snapshot.update_cnt != last_emitted_cnt) {
        last_emitted_cnt = gps_snapshot.update_cnt;
        emitGpsFrame();
    }
}

// A fix is usable only when fresh and valid; sats>=4 alone is NOT a lock
// (would prevent the watchdog from ever firing).
#define GPS_FIX_FRESH_MS 10000  // a fix older than 10 s is considered lost
bool gpsHasUsableLock() {
    if (tinyGPS.location.isValid() &&
        tinyGPS.location.age() < GPS_FIX_FRESH_MS) {
        return true;
    }
    return false;
}

// Core 0 lock-loss watchdog: hot-restart after prolonged no-fix (see config above).
void gpsLockWatchdog() {
    // Enforce the minimum interval between two restarts.
    if (millis() - gps_last_restart_ms < GPS_WATCHDOG_COOLDOWN_MS) {
        return;
    }
    // Bail out until a first usable fix (don't restart a healthy cold start).
    if (gps_last_valid_fix_ms == 0) {
        return;
    }
    if (millis() - gps_last_valid_fix_ms < GPS_LOCK_LOSS_TIMEOUT_MS) {
        return;
    }

    // Persist the 10 Hz / GPS+BDS / 115200 settings, then hot restart.
    sendPcasCmd("PCAS00");    // $PCAS00*01  save configuration
    sendPcasCmd("PCAS10,0");  // $PCAS10,0*1C  hot restart
    gps_last_restart_ms = millis();
    gps_last_valid_fix_ms = millis(); // give the module a fresh cooldown window
    Serial.println("[GPS] lock lost, hot restart");
}

// ----------------------------------------------------------------------------
// ATGM336H native configuration via $PCAS NMEA commands.
//   $PCAS02,<rate>  -> update RATE in ms (100 => 10 Hz)
//   $PCAS04,<sys>   -> constellation; 1 = GPS only, 3 = GPS+BDS  <-- used here
//   $PCAS01,<baud>  -> UART baud; 5 = 115200  (keep in sync with GPS_BAUD)
// Checksums are computed at runtime by nmeaChecksum() in sendPcasCmd().
// ----------------------------------------------------------------------------
// Compute the NMEA XOR checksum over payload (chars between '$' and '*').
uint8_t nmeaChecksum(const char* payload) {
    uint8_t ck = 0;
    for (const char* p = payload; *p && *p != '\r' && *p != '\n'; ++p)
        ck ^= (uint8_t)*p;
    return ck;
}

// Build and transmit a full "$<payload>*hh\r\n" frame over Serial2.
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
    // 1) GPS + BeiDou (BDS) constellation select ($PCAS04,3).
    sendPcasCmd("PCAS04,3");
    delay(100);

    // 2) 10 Hz update rate = 100 ms interval ($PCAS02,100).
    sendPcasCmd("PCAS02,100");
    delay(100);

    // 3) Keep UART at 115200 baud, in sync with GPS_BAUD ($PCAS01,5).
    sendPcasCmd("PCAS01,5");
    delay(100);

    Serial.println("[GPS] ATGM336H configured: GPS+BDS (PCAS04,3) @ 10Hz (PCAS02,100) @ 115200 baud (PCAS01,5).");
}
