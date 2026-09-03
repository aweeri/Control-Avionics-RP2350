# RapidFish GroundStation

A standalone desktop telemetry visualizer and CSV logger for RapidFish serial
data. Built with **Bun** as a local web server whose browser UI (Chrome/Edge)
reads the serial port via the **Web Serial API** and streams live frames back
to the server over WebSocket for persistence and broadcasting.

## Requirements

- [Bun](https://bun.sh) **1.x** (to build/run from source). The compiled exe
  doesn't need Bun installed.
- A Web Serial capable browser - Chromium Based

## Quick start (build the exe)

```sh
cd Decoder/GroundStation
bun install
bun run build        # -> dist/RapidFishGroundStation.exe
```

Then double-click the exe (or run it). It prints the local URL, auto-opens the
browser, and you:

1. Click **Connect serial**.
2. Choose the RapidFish USB-UART port (baud is hard-coded to 115200).
3. Telemetry appears live as cards + charts, and every JSON frame is logged to
   `./logs/rapidfish-<timestamp>.csv`.

## Run from source (hot reload)

```sh
cd Decoder/GroundStation
bun install
bun run dev          # listen on http://localhost:8765
bun run start        # same but no hot reload
```

## Configuration (env vars)

| Variable  | Default        | Purpose                                |
|-----------|----------------|----------------------------------------|
| `PORT`    | `8765`         | Local HTTP/WebSocket port              |
| `LOG_DIR` | `./logs`       | Where CSV files are written            |
| `NO_OPEN` | (unset)        | Set to `1` to prevent auto-opening the browser |

## Build targets

```sh
bun run build          # Windows x64 exe
bun run bundle:macos   # macOS arm64 binary
bun run bundle:win     # Windows x64 exe (same as build)
```

## CSV output

Every session writes to a timestamped **subdirectory** of the log root, with one
CSV file per apid:

```
logs/
└─ rapidfish-2026-09-03-10-15-14/
   ├─ apid-0.csv
   ├─ apid-1.csv
   └─ apid-256.csv
```

Because frames from different apids have different schemas, each file keeps its
own **dynamic union of columns seen**, flattens nested objects into dot-notation
keys (e.g. `accel.x`, `local_gps.lat`, `mag.z`), and writes a single header.
Rows are appended continuously and consolidated on a 2.5s interval (or when a new
column appears) so steady-state telemetry is reliably persisted. A fresh session
directory is created each time you connect the serial port.

Example `apid-0.csv`:

```csv
sync,apid,timestamp,rssi,snr,flight_state,flight_state_str,accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z,pressure_hpa,temperature_c,bat_voltage
449838109,0,3795422,-77,11.2,3,ARMED,0,0,0,0,0,0,992.4,27,0.43
```

Example `apid-1.csv`:

```csv
sync,apid,timestamp,rssi,snr,lat,lon,gps_alt,sats,hdop,mag.x,mag.y,mag.z
449838109,1,3800422,-76,11,0,0,12.5,4,1.2,1,2,3
```

## Project layout

```
Decoder/GroundStation/
├─ server.ts               # Bun HTTP + WebSocket server, CSV writer, browser launcher
├─ csvLogger.ts            # Dynamic-column CSV logger (flatten + union columns)
├─ package.json            # Scripts: dev/start/build/bundle + UI generator
├─ public/index.html       # Single-file browser UI (cards, canvas charts, serial)
├─ scripts/
│  └─ generate-ui.ts       # Inlines public/index.html into generated/ui.ts for exe
├─ generated/ui.ts         # (generated) embedded HTML
└─ dist/                   # (generated) compiled exe
```

## How it works

1. The exe starts a Bun `Bun.serve` on `localhost:PORT` serving the embedded UI
   and a `/ws` WebSocket endpoint, then opens the default browser.
2. The browser connects to `/ws`, registers as the **serial source**, and shows
   the serial **Connect** button.
3. On connect, the browser uses `navigator.serial` to open the port,
   reads lines, and forwards each to the server as `{type:"line", data}`
   messages.
4. The server parses JSON frames: logs them to CSV (if recording) and
   broadcasts them to all connected tabs as `{type:"telemetry", data}`.
5. Each tab renders telemetry cards and live canvas charts (altitude/pressure,
   RSSI/SNR, accel, gyro) with no external dependencies, so the whole thing
   works fully offline.

## Recording control

The **● Rec** toggle in the header enables/disables CSV writing server-side.
Pausing recording stops new rows from being appended while the UI keeps
updating.