# CORE APID 0 (32 bytes)
{
    uint32_t sync_word;       // +0  (4)  APID 0 identifier (0x1ACFFC1D)
    uint32_t timestamp;       // +4  (4)  millis()
    uint8_t  apid;            // +8  (1)  Application Process ID (0=core)
    uint8_t  flight_state;    // +9  (1)  Current state machine stage
    uint8_t  flash_used;      // +10 (1)  0-255 = 0-100% flash usage
    int8_t   core_temp;       // +11 (1)  RP2350 internal temp °C

    int16_t ax, ay, az;       // +12 (6)  Accel: (m/s²) * 100
    int16_t gx, gy, gz;       // +18 (6)  Gyro: (rad/s) * 1000

    uint16_t altitude;        // +24 (2)  Baro: meters MSL * 2 (0-131070m)
    int8_t   temperature;     // +26 (1)  Barometer temp °C

    uint8_t  bat_voltage;     // +27 (1)  Battery ADC (0-255 scaled)
    uint8_t  p1_voltage;      // +28 (1)  Pyro 1 continuity ADC
    uint8_t  p2_voltage;      // +29 (1)  Pyro 2 continuity ADC

    uint8_t  _pad[2];         // +30 (2)  Padding to 32 bytes
};

# GPS APID 1 (32 bytes)
{
    uint32_t sync_word;       // +0  (4)  APID 1 identifier (0x5ACFFC1D)
    uint32_t timestamp;       // +4  (4)  millis()
    uint8_t  apid;            // +8  (1)  Application Process ID (1=gps)
    uint32_t lat;             // +9  (4)  Latitude  * 1e7
    uint32_t lon;             // +13 (4)  Longitude * 1e7
    uint16_t gps_alt;         // +17 (2)  GPS altitude (meters)
    uint8_t  state;           // +19 (1)  GPS fix state
    uint8_t  sats;            // +20 (1)  Satellite count
    int16_t  mx, my, mz;      // +21 (6)  Magnetometer raw
    uint8_t  _pad[5];         // +27 (5)  Padding to 32 bytes
}