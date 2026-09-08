#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Preferences.h>
#include <stdint.h>

// ============================================================
// Board selection & pin mapping.
// The active board is chosen by compiling the matching env in
// platformio.ini; it defines BOARD_TTGO_T3 for that target.
// ============================================================
#if defined(BOARD_TTGO_T3)
// ---------- TTGO T3 LoRa32 v1.6.1 (ESP32 + SX1276) ----------
#define RADIO_CS_PIN   18
#define RADIO_RST_PIN  23
#define RADIO_DIO1_PIN 26   // DIO0 (RxDone IRQ)
#define RADIO_BUSY_PIN RADIOLIB_NC
#define SPI_SCK_PIN    5
#define SPI_MISO_PIN   19
#define SPI_MOSI_PIN   27
#define HAS_ONBOARD_GPS 0

#else
// ---------- Heltec Wireless Tracker Stick (ESP32-S3 + SX1262 + GPS) ----------
#define RADIO_CS_PIN   8
#define RADIO_RST_PIN  12
#define RADIO_DIO1_PIN 14
#define RADIO_BUSY_PIN 13
#define SPI_SCK_PIN    9
#define SPI_MISO_PIN   11
#define SPI_MOSI_PIN   10
#define HAS_ONBOARD_GPS 1

#define GPS_RX_PIN     33   // ESP32 RX  <- GNSS TX
#define GPS_TX_PIN     34   // ESP32 TX  -> GNSS RX
#define GPS_BAUD       115200
#define GPS_PWR_PIN    3    // GNSS power-enable, V1.1 (drive HIGH to power GNSS)
#define GPS_FRESH_MS   5000

// Set to 1 to echo each complete NMEA sentence to the USB serial.
// Each dump is a clearly tagged, non-JSON line so JSON parsers ignore it.
#define GPS_DUMP_NMEA  0
#endif

#if defined(BOARD_TTGO_T3)
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#else
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

// ============================================================
// Persistent radio settings (stored in NVS via Preferences).
// ============================================================
typedef struct {
  float   freq_mhz;   // carrier frequency, MHz
  float   bw_khz;     // LoRa bandwidth, kHz
  uint8_t sf;         // spreading factor 6..12
  uint8_t cr;         // coding rate 5..8 (4/5 .. 4/8)
  uint16_t preamble;  // preamble length, symbols
  uint8_t accept_crc_mismatch; // 1 = decode frames with failed CRC
} AirSettings;

// Defaults applied when nothing is stored in NVS yet.
const AirSettings DEFAULT_SETTINGS = {
  .freq_mhz = 434.0f,
  .bw_khz   = 250.0f,
  .sf       = 7,
  .cr       = 5,
  .preamble = 8,
  .accept_crc_mismatch = 0,
};

AirSettings settings;
Preferences prefs;

// Protocol constants (NOT runtime-configurable).
#define PAYLOAD_LEN    28      // implicit-header payload length (bytes)
#define SYNC_WORD      0x1ACFFC1D
#define RX_SYNC_WORD   0x12    // one-byte modem sync word
#define SERIAL_BAUD    115200

// ============================================================
// Interrupt flag
// ============================================================
volatile bool receivedFlag = false;
void setFlag(void) { receivedFlag = true; }

// ============================================================
// Settings: load / save (NVS)
// ============================================================
static void loadSettings(void) {
  settings = DEFAULT_SETTINGS;
  prefs.begin("radio_sets", false);
  settings.freq_mhz  = prefs.getFloat("freq",  DEFAULT_SETTINGS.freq_mhz);
  settings.bw_khz    = prefs.getFloat("bw",    DEFAULT_SETTINGS.bw_khz);
  settings.sf        = prefs.getUChar("sf",    DEFAULT_SETTINGS.sf);
  settings.cr        = prefs.getUChar("cr",    DEFAULT_SETTINGS.cr);
  settings.preamble  = prefs.getUShort("pre",  DEFAULT_SETTINGS.preamble);
  settings.accept_crc_mismatch = prefs.getUChar("crcm", DEFAULT_SETTINGS.accept_crc_mismatch);
  // Sanity-clamp anything stale/corrupt.
  if (settings.freq_mhz < 150.0f || settings.freq_mhz > 1000.0f) settings.freq_mhz = DEFAULT_SETTINGS.freq_mhz;
  if (settings.bw_khz < 7.8f || settings.bw_khz > 500.0f)       settings.bw_khz   = DEFAULT_SETTINGS.bw_khz;
  if (settings.sf < 6 || settings.sf > 12)                      settings.sf       = DEFAULT_SETTINGS.sf;
  if (settings.cr < 5 || settings.cr > 8)                       settings.cr       = DEFAULT_SETTINGS.cr;
  if (settings.preamble < 6 || settings.preamble > 65535)       settings.preamble = DEFAULT_SETTINGS.preamble;
  settings.accept_crc_mismatch = settings.accept_crc_mismatch ? 1 : 0;
  prefs.end();
}

static void saveSettings(void) {
  prefs.begin("radio_sets", false);
  prefs.putFloat("freq", settings.freq_mhz);
  prefs.putFloat("bw",   settings.bw_khz);
  prefs.putUChar("sf",   settings.sf);
  prefs.putUChar("cr",   settings.cr);
  prefs.putUShort("pre", settings.preamble);
  prefs.putUChar("crcm", settings.accept_crc_mismatch);
  prefs.end();
}

// ============================================================
// Radio bring-up / reconfiguration
// Calls RadioLib setters; both SX1262 and SX1276 expose these
// identical methods, so one routine serves both boards.
// ============================================================
static void printRadioSettings(void) {
  Serial.printf("freq F %.1f\n",     settings.freq_mhz);
  Serial.printf("bandwidth BW %.1f\n", settings.bw_khz);
  Serial.printf("spreading factor SF %u\n", settings.sf);
  Serial.printf("coding rate CR %u\n", settings.cr);
  Serial.printf("preamble PL %u\n",   settings.preamble);
  Serial.printf("crc mismatch ACCEPT %u\n", settings.accept_crc_mismatch);
}

static int applyRadioParams(void) {
  int state;

  radio.standby();

  state = radio.setFrequency(settings.freq_mhz);
  if (state) return state;
  state = radio.setBandwidth(settings.bw_khz);
  if (state) return state;
  state = radio.setSpreadingFactor(settings.sf);
  if (state) return state;
  state = radio.setCodingRate(settings.cr);
  if (state) return state;
  state = radio.setPreambleLength(settings.preamble);
  if (state) return state;
  state = radio.setSyncWord(RX_SYNC_WORD);
  if (state) return state;
  state = radio.setCRC(true);
  if (state) return state;
  state = radio.implicitHeader(PAYLOAD_LEN);
  if (state) return state;

  return radio.startReceive();
}

// ============================================================
// Serial command interface (plain text, newline-terminated).
//   F  <MHz>   set frequency        e.g. F 434.0
//   BW <kHz>   set bandwidth        e.g. BW 250.0
//   SF <n>     set spreading factor e.g. SF 7   (6..12)
//   CR <n>     set coding rate      e.g. CR 5   (5..8)
//   PL <n>     set preamble length  e.g. PL 8   (symbols)
//   CRC <0|1>  accept crc mismatch  e.g. CRC 0  (0 = reject, 1 = accept)
//   SHOW       print current settings
//   SAVE       persist to NVS
// Any change is applied live and printed back in "changing ..." form.
// ============================================================
static char lineBuf[40];
static uint8_t lineLen = 0;

static void handleCommand(const char* cmd) {
  char c0 = cmd[0];

  if (c0 == 'S' && strncmp(cmd, "SHOW", 4) == 0) {
    printRadioSettings();
    return;
  }
  if (c0 == 'S' && strncmp(cmd, "SAVE", 4) == 0) {
    saveSettings();
    Serial.println("saved");
    return;
  }

  switch (c0) {
    case 'F': case 'f': {
      float v = atof(cmd + 1);
      if (v < 150.0f || v > 1000.0f) { Serial.println("bad F"); return; }
      settings.freq_mhz = v;
      int st = applyRadioParams();
      Serial.printf("changing frequency F %.1f %s\n", settings.freq_mhz, st ? "FAIL" : "ok");
      if (!st) saveSettings();
      break;
    }
    case 'B': case 'b': { // BW
      if (strncmp(cmd, "BW", 2) != 0 && strncmp(cmd, "bw", 2) != 0) { Serial.println("bad BW"); return; }
      float v = atof(cmd + 2);
      if (v < 7.8f || v > 500.0f) { Serial.println("bad BW"); return; }
      settings.bw_khz = v;
      int st = applyRadioParams();
      Serial.printf("changing bandwidth BW %.1f %s\n", settings.bw_khz, st ? "FAIL" : "ok");
      if (!st) saveSettings();
      break;
    }
    case 'S': case 's': { // SF
      if (strncmp(cmd, "SF", 2) != 0 && strncmp(cmd, "sf", 2) != 0) { Serial.println("bad SF"); return; }
      uint8_t v = atoi(cmd + 2);
      if (v < 6 || v > 12) { Serial.println("bad SF"); return; }
      settings.sf = v;
      int st = applyRadioParams();
      Serial.printf("changing spreading factor SF %u %s\n", settings.sf, st ? "FAIL" : "ok");
      if (!st) saveSettings();
      break;
    }
    case 'C': case 'c': { // CRC or CR (CRC must be checked first)
      if (strncmp(cmd, "CRC", 3) == 0 || strncmp(cmd, "crc", 3) == 0) {
        uint8_t v = atoi(cmd + 3);
        if (v > 1) { Serial.println("bad CRC"); return; }
        settings.accept_crc_mismatch = v;
        Serial.printf("changing crc mismatch ACCEPT %u ok\n", settings.accept_crc_mismatch);
        saveSettings();
      } else if (strncmp(cmd, "CR", 2) == 0 || strncmp(cmd, "cr", 2) == 0) {
        uint8_t v = atoi(cmd + 2);
        if (v < 5 || v > 8) { Serial.println("bad CR"); return; }
        settings.cr = v;
        int st = applyRadioParams();
        Serial.printf("changing coding rate CR %u %s\n", settings.cr, st ? "FAIL" : "ok");
        if (!st) saveSettings();
      } else {
        Serial.println("bad cmd");
      }
      break;
    }
    case 'P': case 'p': { // PL
      if (strncmp(cmd, "PL", 2) != 0 && strncmp(cmd, "pl", 2) != 0) { Serial.println("bad PL"); return; }
      uint16_t v = atoi(cmd + 2);
      if (v < 6) { Serial.println("bad PL"); return; }
      settings.preamble = v;
      int st = applyRadioParams();
      Serial.printf("changing preamble PL %u %s\n", settings.preamble, st ? "FAIL" : "ok");
      if (!st) saveSettings();
      break;
    }
    default:
      Serial.println("bad cmd");
  }
}

static void handleSerial(void) {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) handleCommand(lineBuf);
      lineLen = 0;
    } else if (c != '\r' && c != ' ' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else if (c != '\r' && c != ' ') {
      // skip excess (drop line on overflow, keep parsing)
    }
  }
}

// ============================================================
// On-air frame layout helpers (little-endian payload)
// ============================================================
static inline uint32_t rd_u32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline uint16_t rd_u16(const uint8_t* p) {
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static inline int16_t rd_i16(const uint8_t* p) {
  return (int16_t)rd_u16(p);
}
static inline int32_t rd_i32(const uint8_t* p) {
  return (int32_t)rd_u32(p);
}

// human-readable flight state names (apid 0)
static const char* flight_state_str(uint8_t s) {
  switch (s) {
    case 0:  return "BOOTING";
    case 1:  return "DISARMED";
    case 2:  return "ERROR";
    case 3:  return "ARMED";
    case 4:  return "ACCELERATING";
    case 5:  return "COAST";
    case 6:  return "RECOVERY";
    case 7:  return "CHUTE";
    case 8:  return "GROUND";
    default: return "UNKNOWN";
  }
}

// ============================================================
// JSON output
// ============================================================
static void emit_json_common(uint32_t timestamp, uint8_t apid, float rssi, float snr) {
  Serial.printf("{\"sync\":\"0x%08X\",\"apid\":%u,\"timestamp\":%u",
                (unsigned)SYNC_WORD, apid, (unsigned)timestamp);
  if (rssi > -200.0f) Serial.printf(",\"rssi\":%.1f,\"snr\":%.1f", rssi, snr);
}

static void decode_core(const uint8_t* p, float rssi, float snr) {
  uint32_t timestamp = rd_u32(p + 0);
  uint8_t  apid      = p[4];
  uint8_t  fstate    = p[5];
  uint8_t  flash_raw = p[6];
  int8_t   core_temp = (int8_t)p[7];
  int16_t  ax = rd_i16(p + 8),  ay = rd_i16(p + 10),  az = rd_i16(p + 12);
  int16_t  gx = rd_i16(p + 14), gy = rd_i16(p + 16),  gz = rd_i16(p + 18);
  uint32_t pressure  = rd_u32(p + 20);
  int8_t   temp      = (int8_t)p[24];
  uint8_t  bat       = p[25];
  uint8_t  p1        = p[26];
  uint8_t  p2        = p[27];

  emit_json_common(timestamp, apid, rssi, snr);
  Serial.printf(",\"flight_state\":%u,\"flight_state_str\":\"%s\"", fstate, flight_state_str(fstate));
  Serial.printf(",\"flash_used\":%u,\"flash_used_pct\":%.1f", flash_raw, flash_raw * 0.5f);
  Serial.printf(",\"core_temp_c\":%d", (int)core_temp);
  Serial.printf(",\"accel\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}", ax / 100.0f, ay / 100.0f, az / 100.0f);
  Serial.printf(",\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}", gx / 1000.0f, gy / 1000.0f, gz / 1000.0f);
  Serial.printf(",\"pressure_hpa\":%.2f", pressure / 100.0f);
  Serial.printf(",\"temperature_c\":%d", (int)temp);
  Serial.printf(",\"bat_voltage\":%.2f", bat * 9.9f / 255.0f);
  Serial.printf(",\"p1_voltage\":%.2f", p1 * 9.9f / 255.0f);
  Serial.printf(",\"p2_voltage\":%.2f", p2 * 9.9f / 255.0f);
  Serial.println('}');
}

static void decode_gps(const uint8_t* p, float rssi, float snr) {
  uint32_t timestamp = rd_u32(p + 0);
  uint8_t  apid      = p[4];
  int32_t  lat       = rd_i32(p + 5);
  int32_t  lon       = rd_i32(p + 9);
  uint16_t galt      = rd_u16(p + 13);
  uint8_t  state     = p[15];
  uint8_t  sats      = p[16];
  uint32_t gtime     = rd_u32(p + 17);
  uint8_t  hdop      = p[21];
  int16_t  mx = rd_i16(p + 22), my = rd_i16(p + 24), mz = rd_i16(p + 26);

  emit_json_common(timestamp, apid, rssi, snr);
  Serial.printf(",\"lat\":%.7f,\"lon\":%.7f", lat / 1e7, lon / 1e7);
  Serial.printf(",\"gps_alt\":%.1f", galt / 2.0f);
  Serial.printf(",\"state\":%u,\"sats\":%u", state, sats);
  Serial.printf(",\"gps_time\":%u", (unsigned)gtime);
  Serial.printf(",\"hdop\":%.1f", hdop / 10.0f);
  Serial.printf(",\"mag\":{\"x\":%d,\"y\":%d,\"z\":%d}", mx, my, mz);
  Serial.println('}');
}

static void process_packet(const uint8_t* buf, size_t len, float rssi, float snr) {
  if (len < PAYLOAD_LEN) return;
  switch (buf[4]) { // apid at offset +4
    case 0: decode_core(buf, rssi, snr); break;
    case 1: decode_gps(buf, rssi, snr);  break;
    default: break; // unknown apid -> ignore
  }
}

// ============================================================
// Integrated GPS parsing (TinyGPSPlus) — only when present
// ============================================================
#if HAS_ONBOARD_GPS
HardwareSerial gpsSerial(1);

// GPS receive diagnostics counters (cumulative over the board's lifetime).
static uint32_t gps_bytes     = 0;  // total bytes read from the GPS serial
static uint32_t gps_sentences = 0;  // count of '\n' seen in the GPS stream

// Raw-NMEA accumulation buffer for optional dumping (GPS_DUMP_NMEA).
static char gps_dump_buf[96];
static uint8_t gps_dump_len = 0;
#endif
TinyGPSPlus gps;

// ============================================================
// Heartbeat (apid 256 JSON, rate-limited to ~1 s)
// ============================================================
static uint32_t last_hb = 0;
static uint32_t frame_count = 0;
static uint32_t rx_errors = 0;
static uint32_t last_rxerr_print = 0;

static void heartbeat(void) {
  uint32_t now = millis();
  if (now - last_hb < 1000) return;
  last_hb = now;

  Serial.printf("{\"sync\":\"0x%08X\",\"apid\":256,\"millis\":%u,\"frames\":%u,\"rx_success\":%u,\"rx_errors\":%u,\"local_gps\":",
                (unsigned)SYNC_WORD, (unsigned)now,
                (unsigned)frame_count, (unsigned)frame_count, (unsigned)rx_errors);

#if HAS_ONBOARD_GPS
  bool fresh = gps.location.isValid() && (gps.location.age() < GPS_FRESH_MS);
#else
  bool fresh = false;
#endif
  Serial.printf("{\"fix\":%s", fresh ? "true" : "false");
#if HAS_ONBOARD_GPS
  Serial.printf(",\"gps_bytes\":%u,\"gps_sentences\":%u",
                (unsigned)gps_bytes, (unsigned)gps_sentences);
#endif
  if (fresh) {
    Serial.printf(",\"lat\":%.7f,\"lon\":%.7f", gps.location.lat(), gps.location.lng());
    Serial.printf(",\"altitude\":%.1f", gps.altitude.meters());
    Serial.printf(",\"speed\":%.2f", gps.speed.mps());
    Serial.printf(",\"course\":%.1f", gps.course.deg());
    Serial.printf(",\"sats\":%u", (unsigned)gps.satellites.value());
  } else {
    Serial.print(",\"lat\":null,\"lon\":null");
  }
  Serial.println("}}");
}

// ============================================================
// Setup / loop
// ============================================================
void setup(void) {
  Serial.begin(SERIAL_BAUD);
  Serial.println("{\"sync\":0x1ACFFC1D,\"apid\":512,\"event\":\"boot\"}");
  Serial.flush();
  delay(2000);

  // Load persisted radio settings, then configure the radio.
  loadSettings();

#if HAS_ONBOARD_GPS
  // Power up the GNSS module (Heltec V1.1 uses GPIO 3 as GNSS power-enable).
  pinMode(GPS_PWR_PIN, OUTPUT);
  digitalWrite(GPS_PWR_PIN, HIGH);
  delay(100);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif

  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, RADIO_CS_PIN);

  int state = radio.begin(settings.freq_mhz, settings.bw_khz, settings.sf,
                          settings.cr, RX_SYNC_WORD, settings.preamble);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[init] radio.begin failed, code %d\n", state);
    while (1) { delay(10); }
  }

  state = applyRadioParams();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[init] radio apply failed, code %d\n", state);
    while (1) { delay(10); }
  }

  radio.setPacketReceivedAction(setFlag);

  Serial.println("[init] ok");
  printRadioSettings();
}

void loop(void) {
#if HAS_ONBOARD_GPS
  // One-time (then infrequent) [gps] diagnostics hint on the USB serial.
  static uint32_t last_gps_stat = 0;
  uint32_t gps_now = millis();
  if ((last_gps_stat == 0 && gps_now >= 10000) || (last_gps_stat != 0 && gps_now - last_gps_stat >= 60000)) {
    last_gps_stat = gps_now;
    Serial.printf("[gps] bytes=%u sentences=%u fix=%d sats=%u\n",
                  (unsigned)gps_bytes, (unsigned)gps_sentences,
                  gps.location.isValid() ? 1 : 0,
                  (unsigned)gps.satellites.value());
  }

  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    gps_bytes++;
    if (c == '\n') gps_sentences++;   // each NMEA sentence ends in '\r\n'
    gps.encode(c);

#if GPS_DUMP_NMEA
    // Accumulate a sentence; flush on '\n' so each dump is one clean line.
    if (gps_dump_len < sizeof(gps_dump_buf) - 1) gps_dump_buf[gps_dump_len++] = c;
    if (c == '\n') {
      gps_dump_buf[gps_dump_len] = '\0';
      if (gps_dump_len > 2) Serial.printf("[nmea] %s<end>\n", gps_dump_buf);
      gps_dump_len = 0;
    }
#endif
  }
#endif

  handleSerial();

  if (receivedFlag) {
    receivedFlag = false;

    uint8_t buf[PAYLOAD_LEN];
    size_t  len = radio.getPacketLength();
    if (len > PAYLOAD_LEN) len = PAYLOAD_LEN;
    int state = radio.readData(buf, len);

    if (settings.accept_crc_mismatch && (state == RADIOLIB_ERR_CRC_MISMATCH)) {
      state = RADIOLIB_ERR_NONE;
    }

    if (state == RADIOLIB_ERR_NONE) {
      frame_count++;
      process_packet(buf, len, radio.getRSSI(), radio.getSNR());
    } else {
      rx_errors++;
      if (millis() - last_rxerr_print > 2000) {
        last_rxerr_print = millis();
        Serial.printf("[rxerr] code=%d len=%u rssi=%.1f snr=%.1f\n",
                      state, (unsigned)radio.getPacketLength(),
                      radio.getRSSI(), radio.getSNR());
      }
    }
    // SX1262 auto-returns to RX, but SX1276 must be re-armed explicitly.
    radio.startReceive();
  }

  heartbeat();
}