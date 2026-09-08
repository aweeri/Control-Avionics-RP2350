# CORE APID 0 (32 bytes)
```cpp
{
    uint32_t sync_word;       // +0  (4)  SyncWord (0x1ACFFC1D) — verify frame alignment
    uint32_t timestamp;       // +4  (4)  millis() — boot-relative uptime in ms
    uint8_t  apid;            // +8  (1)  Application Process ID (0=core)
    uint8_t  flight_state;    // +9  (1)  Flight state enum: 0=BOOTING, 1=DISARMED, 2=ERROR,
                              //              3=ARMED, 4=ACCELERATING, 5=COAST,
                              //              6=RECOVERY, 7=CHUTE, 8=GROUND
    uint8_t  flash_used;      // +10 (1)  Flash usage: 0–200 = 0–100% in 0.5% increments
    int8_t   core_temp;       // +11 (1)  RP2350 internal temp °C (direct, no scaling)

    int16_t ax, ay, az;       // +12 (6)  Accel: raw int16 → m/s² = value / 100
    int16_t gx, gy, gz;       // +18 (6)  Gyro:  raw int16 → rad/s  = value / 1000

    uint32_t pressure;        // +24 (4)  Baro: raw Pa → hPa = value / 100
    int8_t   temperature;     // +28 (1)  Barometer temp °C (direct, no scaling)

    uint8_t  bat_voltage;     // +29 (1)  Battery ADC 0–255 → V = value * 9.9 / 255
    uint8_t  p1_voltage;      // +30 (1)  Ch1 pyro continuity ADC 0–255 → V = value * 9.9 / 255
    uint8_t  p2_voltage;      // +31 (1)  Ch2 pyro continuity ADC 0–255 → V = value * 9.9 / 255
};
```

# GPS APID 1 (32 bytes)
```cpp
{
    uint32_t sync_word;       // +0  (4)  SyncWord (0x1ACFFC1D) — verify frame alignment
    uint32_t timestamp;       // +4  (4)  millis() — boot-relative uptime in ms
    uint8_t  apid;            // +8  (1)  Application Process ID (1=gps)
    int32_t  lat;             // +9  (4)  Latitude  → deg = value / 1e7  (SIGNED, negative = south)
    int32_t  lon;             // +13 (4)  Longitude → deg = value / 1e7  (SIGNED, negative = west)
    uint16_t gps_alt;         // +17 (2)  GPS MSL altitude → m = value / 2  (0.5 m units)
    uint8_t  state;           // +19 (1)  Fix state: 1 = valid 3D fix, 0 = no fix
    uint8_t  sats;            // +20 (1)  Satellite count (direct)
    uint32_t gps_time;        // +21 (4)  Unix timestamp (seconds since epoch, UTC)
    uint8_t  hdop;            // +25 (1)  HDOP → unitless = value / 10  (clamped to 255)
    int16_t  mx, my, mz;      // +26 (6)  Magnetometer raw (placeholder — always 0)
};