# CORE APID 0
{
    uint32_t sync_word;
    uint32_t timestamp;
    (idk what format) apid_id;
    uint8_t flight_state;    // Current state machine stage
    uint8_t flash_used       // 0-255 (% used on data flash mapped to 8 bits)
    int8_t core_temp;        // RP2040 internal temp

    int16_t ax, ay, az;      // Accel: (m/s²) * 100
    int16_t gx, gy, gz;      // Gyro: (rad/s) * 1000

    uint16_t altitude;       // Baro: calculated meters above sea * 2
    int8_t temperature;      // Barometer C

    uint8_t bat_voltage      // Battery ADC
    uint8_t p1_voltage       // can make less if needed        
    uint8_t p2_voltage       // can make less if needed
    
    uint8_t _pad[3];         // PADDING TO FILL FRAME
};

# GPS APID 1
{
    uint32_t sync_word;
    uint32_t timestamp;
    (idk what format) apid_id;
    uint32_t lat;
    uint32_t lon;
    uint16_t gps_alt;
    uint8_t state;
    uint8_t sats;
    int16_t mx, my, mz;      // Magneto multiplied by idk
    uint8_t _pad[N]; // fill frame to match apid 0
}