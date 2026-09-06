# INS Test Firmware — Barebones ESKF for CARP-LITE

Single-file test firmware that runs an Error-State Kalman Filter (ESKF) on the RP2350's core 1, fusing IMU + baro + GPS into a continuous 3D state estimate. No flight state machine, no recovery logic — just sensor fusion.

## Files

| File | Purpose |
|------|---------|
| [`ins_test.ino`](ins_test.ino) | Everything — quaternion math, ESKF, dual-core setup, radio telemetry |

## How It Works

**Core 1** reads the IMU at 1 kHz and runs the ESKF prediction step (integrates gyro into attitude, accel into velocity/position). It reads the baro at 200 Hz and runs correction steps for accel (50 Hz) and baro (50 Hz). GPS corrections are applied when new fixes arrive via the mailbox.

**Core 0** reads the fused state from shared memory, builds APID 0 and APID 1 frames, and transmits them over the LR2021 radio at 80 ms intervals. The existing ground station decodes and displays everything with zero changes.

## Ground Station Compatibility

The fused state is fed into the existing APID 0 and APID 1 frame formats:

- **APID 0**: `accel.x/y/z` contains the fused gravity vector (bias-corrected, gyro-stabilized). The ground station's `updateAttitude()` builds the 3D visualization from these fields — it will show smooth, accurate orientation automatically.

- **APID 1**: `lat/lon` contains the fused position (GPS + IMU dead-reckoned). The map will show smoother tracking between GPS updates.

## Building

Open in Arduino IDE or PlatformIO with board = `rpipico2`. Requires the same libraries as RapidFish v2:
- Adafruit LSM6DSO32
- SparkFun LSM6DSV16X
- Adafruit BMP3XX
- RadioLib
- TinyGPSPlus

## Commands (Serial at 115200 baud)

| Command | Description |
|---------|-------------|
| `STATUS` | Print current fused state (attitude, position, velocity, biases, confidence) |
| `RESET` | Reset the filter |
| `HELP` | Print this message |

## Tuning

The ESKF's tuning parameters are in the constructor defaults (search for `sigma_gyro`). Key parameters:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `sigma_gyro` | 0.005 rad/s/√Hz | Gyro noise density |
| `sigma_accel_meas` | 0.5 m/s² | Trust in accel for attitude correction |
| `sigma_baro_meas` | 1.0 m | Trust in baro altitude |
| `sigma_gps_pos` | 5.0 m | Trust in GPS position |
| `boost_accel_threshold` | 12.0 m/s² | Above this, accel noise scales up |

## Test Plan

1. **Static**: Place on desk, verify attitude converges to level in ~5 seconds
2. **Rotation**: Manually rotate, verify 3D visualization tracks correctly
3. **Baro**: Lift 2 meters, verify altitude tracks within ±0.5 m
4. **GPS**: Walk 100 meters, verify position tracks within ±5 m