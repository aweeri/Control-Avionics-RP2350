# CORE APID 0 (32 bytes)
```cpp
{
    uint32_t sync_word;       // +0  (4)  Standard SyncWord (0x1ACFFC1D)
    uint32_t timestamp;       // +4  (4)  millis()
    uint8_t  apid;            // +8  (1)  Application Process ID (0=core)
    uint8_t  flight_state;    // +9  (1)  Current state machine stage
    uint8_t  flash_used;      // +10 (1)  0-200 = 0-100% in 0.5% increments
    int8_t   core_temp;       // +11 (1)  RP2350 internal temp °C

    int16_t ax, ay, az;       // +12 (6)  Accel: (m/s²) * 100
    int16_t gx, gy, gz;       // +18 (6)  Gyro: (rad/s) * 1000

    uint32_t pressure;        // +24 (4)  Baro: raw Pa pressure (/100 for hPa)
    int8_t   temperature;     // +28 (1)  Barometer temp °C

    uint8_t  bat_voltage;     // +29 (1)  Battery ADC (0-255 scaled)
    uint8_t  p1_voltage;      // +30 (1)  Pyro 1 continuity ADC
    uint8_t  p2_voltage;      // +31 (1)  Pyro 2 continuity ADC
};
```

# GPS APID 1 (32 bytes)
```cpp
{
    uint32_t sync_word;       // +0  (4)  Standard SyncWord (0x1ACFFC1D)
    uint32_t timestamp;       // +4  (4)  millis()
    uint8_t  apid;            // +8  (1)  Application Process ID (1=gps)
    int32_t  lat;             // +9  (4)  Latitude  * 1e7   (SIGNED)
    int32_t  lon;             // +13 (4)  Longitude * 1e7   (SIGNED)
    uint16_t gps_alt;         // +17 (2)  GPS altitude (MSL) in 0.5 m units (meters*2)
    uint8_t  state;           // +19 (1)  GPS fix state
    uint8_t  sats;            // +20 (1)  Satellite count
    uint32_t gps_time;        // +21 (4)  Unix timestamp (seconds since epoch, UTC)
    uint8_t  hdop;            // +25 (1)  Horizontal dilution of precision * 10
    int16_t  mx, my, mz;      // +26 (6)  Magnetometer raw
}
```
