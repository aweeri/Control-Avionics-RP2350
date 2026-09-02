# requires pyserial: pip install pyserial
# requires tqdm: pip install tqdm
# requires matplotlib: pip install matplotlib numpy
import serial
import sys
import threading
import struct
import csv
import time
import os

STATE = "NORMAL"
BUFFER = bytearray()

# Fixed 32-byte frame size for both APID 0 (CORE) and APID 1 (GPS)
FRAME_SIZE = 32
SYNC_WORD = 0x1ACFFC1D
FLIGHT_STATES = ["BOOTING", "ERROR", "ARMED", "ACCELERATING", "COAST", "RECOVERY", "CHUTE", "GROUND"]


def parse_dump(raw_bytes):
    """Parse a binary flash dump containing 32-byte LogFrame records.
    
    Handles both APID 0 (CORE sensor frames) and APID 1 (GPS frames).
    Writes two separate CSV files.
    """
    end_idx = raw_bytes.find(b"\nDUMP_END")
    if end_idx != -1:
        raw_bytes = raw_bytes[:end_idx]

    ts = int(time.time())
    core_csv = f"flight_data_{ts}_core.csv"
    gps_csv  = f"flight_data_{ts}_gps.csv"

    total_frames = len(raw_bytes) // FRAME_SIZE
    core_count = 0
    gps_count = 0

    try:
        from tqdm import tqdm
        progress = tqdm(total=total_frames, desc="Parsing dump", unit="frames",
                        bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt} frames [{elapsed}<{remaining}]")
        has_tqdm = True
    except ImportError:
        has_tqdm = False
        print(f"\n[CLI] Parsing {total_frames} frames...")

    # --- CORE (APID 0) CSV writer ---
    with open(core_csv, "w", newline="") as f_core, \
         open(gps_csv,  "w", newline="") as f_gps:

        core_writer = csv.writer(f_core)
        core_writer.writerow([
            "Timestamp_ms", "State", "Flash_Used_%", "Core_Temp_C",
            "Accel_X_ms2", "Accel_Y_ms2", "Accel_Z_ms2",
            "Gyro_X_rads", "Gyro_Y_rads", "Gyro_Z_rads",
            "Altitude_m", "Baro_Temp_C",
            "Bat_V", "Pyro1_V", "Pyro2_V"
        ])

        gps_writer = csv.writer(f_gps)
        gps_writer.writerow([
            "Timestamp_ms", "Latitude", "Longitude", "GPS_Alt_m",
            "Fix_State", "Sats", "GPS_Time_Unix", "HDOP",
            "Mag_X", "Mag_Y", "Mag_Z"
        ])

        for i in range(0, len(raw_bytes), FRAME_SIZE):
            chunk = raw_bytes[i:i + FRAME_SIZE]
            if len(chunk) < FRAME_SIZE:
                break

            # Unpack the common header: sync_word(4), timestamp(4), apid(1)
            sync, timestamp, apid = struct.unpack_from('<IIB', chunk, 0)

            if sync != SYNC_WORD:
                if has_tqdm:
                    progress.update(1)
                continue

            if apid == 0:
                # --- CORE frame (APID 0) ---
                # struct: IIBBBbhhhhhhHbBBBxx
                fields = struct.unpack_from('<IIBBBbhhhhhhHbBBB', chunk, 0)
                # fields: sync, ts, apid, f_state, flash_u, core_t,
                #         ax, ay, az, gx, gy, gz, alt, baro_t, bat, p1, p2
                _, ts_val, _, f_state, flash_u, core_t, \
                    ax, ay, az, gx, gy, gz, alt, baro_t, bat, p1, p2 = fields

                state_str = FLIGHT_STATES[f_state] if f_state < len(FLIGHT_STATES) else str(f_state)

                core_writer.writerow([
                    ts_val,
                    state_str,
                    flash_u / 2.0,          # 0-200 → 0-100 %
                    core_t,                  # °C
                    ax / 100.0,              # m/s²
                    ay / 100.0,
                    az / 100.0,
                    gx / 1000.0,             # rad/s
                    gy / 1000.0,
                    gz / 1000.0,
                    alt / 2.0,               # meters MSL
                    baro_t,                  # barometer temp °C
                    (bat * 9.9) / 255.0,     # battery voltage (V)
                    (p1 * 9.9) / 255.0,      # pyro 1 continuity (V)
                    (p2 * 9.9) / 255.0,      # pyro 2 continuity (V)
                ])
                core_count += 1

            elif apid == 1:
                # --- GPS frame (APID 1) ---
                # struct: IIBiiHBBIBhhh
                fields = struct.unpack_from('<IIBiiHBBIBhhh', chunk, 0)
                # fields: sync, ts, apid, lat, lon, gps_alt, state, sats,
                #         gps_time, hdop, mx, my, mz
                _, ts_val, _, lat, lon, gps_alt, fix_state, sats, \
                    gps_time, hdop, mx, my, mz = fields

                gps_writer.writerow([
                    ts_val,
                    lat / 1e7,               # degrees (signed)
                    lon / 1e7,               # degrees (signed)
                    gps_alt / 2.0,           # meters MSL
                    fix_state,               # 1 = valid fix
                    sats,                    # satellite count
                    gps_time,                # Unix seconds since epoch (UTC)
                    hdop / 10.0,             # HDOP
                    mx, my, mz,              # magnetometer raw counts
                ])
                gps_count += 1

            if has_tqdm:
                progress.update(1)

    if has_tqdm:
        progress.close()

    print(f"\n[CLI] Saved {core_count} CORE frames → {core_csv}")
    print(f"[CLI] Saved {gps_count} GPS frames  → {gps_csv}")
    if gps_count == 0:
        os.remove(gps_csv)
        print(f"[CLI] (removed empty {gps_csv})")
    print("> ", end="", flush=True)


def export_csv_charts(csv_path):
    """Read a CORE flight data CSV and generate a comprehensive set of matplotlib charts.
    
    Produces a multi-panel figure with:
    - Time on X-axis (shared across all panels)
    - Flight state backgrounds color-coded
    - Properly scaled axes for each sensor type
    """
    if not os.path.isfile(csv_path):
        print(f"[CLI] File not found: {csv_path}")
        return

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.patches import Patch
    except ImportError:
        print("[CLI] matplotlib is required for chart export. Install: pip install matplotlib numpy")
        return

    print(f"[CLI] Loading CSV: {csv_path}")

    # Read CSV data
    timestamps = []
    states_list = []
    flash_used = []
    core_temp = []
    accel_x, accel_y, accel_z = [], [], []
    gyro_x, gyro_y, gyro_z = [], [], []
    altitude = []
    baro_temp = []
    bat_v = []
    pyro1_v, pyro2_v = [], []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        total_rows = sum(1 for _ in open(csv_path, "r")) - 1
        f.seek(0)
        next(reader)
        f.seek(0)
        reader = csv.DictReader(f)

        try:
            from tqdm import tqdm
            row_iter = tqdm(reader, total=total_rows, desc="Reading CSV", unit="rows",
                            bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt} rows [{elapsed}<{remaining}]")
        except ImportError:
            row_iter = reader
            print(f"[CLI] Reading {total_rows} rows...")

        for row in row_iter:
            timestamps.append(int(row["Timestamp_ms"]))
            states_list.append(row["State"])
            flash_used.append(float(row["Flash_Used_%"]))
            core_temp.append(float(row["Core_Temp_C"]))
            accel_x.append(float(row["Accel_X_ms2"]))
            accel_y.append(float(row["Accel_Y_ms2"]))
            accel_z.append(float(row["Accel_Z_ms2"]))
            gyro_x.append(float(row["Gyro_X_rads"]))
            gyro_y.append(float(row["Gyro_Y_rads"]))
            gyro_z.append(float(row["Gyro_Z_rads"]))
            altitude.append(float(row["Altitude_m"]))
            baro_temp.append(float(row["Baro_Temp_C"]))
            bat_v.append(float(row["Bat_V"]))
            pyro1_v.append(float(row["Pyro1_V"]))
            pyro2_v.append(float(row["Pyro2_V"]))

    # Time axis (seconds from start)
    t0 = timestamps[0]
    t_sec = np.array([(ts - t0) / 1000.0 for ts in timestamps])
    t_start = t_sec[0]
    t_end = t_sec[-1]

    # Compute derived quantities
    accel_mag = np.sqrt(np.array(accel_x)**2 + np.array(accel_y)**2 + np.array(accel_z)**2)
    # Compute g-force (accel magnitude / 9.81)
    g_force = accel_mag / 9.81

    # Generate output filenames
    base_name = os.path.splitext(os.path.basename(csv_path))[0]
    output_png = f"{base_name}_charts.png"
    output_pdf = f"{base_name}_charts.pdf"

    print(f"[CLI] Generating charts...")

    # =========================================================================
    # State color map and background helper
    # =========================================================================
    state_colors = {
        "BOOTING":      "#888888",
        "ERROR":        "#FF0000",
        "ARMED":        "#FFA500",
        "ACCELERATING": "#FFD700",
        "COAST":        "#228B22",
        "RECOVERY":     "#1E90FF",
        "CHUTE":        "#8B008B",
        "GROUND":       "#8B4513",
    }

    state_alpha = 0.12

    def add_state_regions(ax, place_top=True):
        """Fill vertical spans for each flight state region + label bar at top."""
        if len(states_list) < 2:
            return
        # Determine state transition points
        transitions = [0]
        for i in range(1, len(states_list)):
            if states_list[i] != states_list[i-1]:
                transitions.append(i)
        transitions.append(len(states_list) - 1)

        # Fill background spans
        for i in range(len(transitions) - 1):
            idx_start = transitions[i]
            idx_end = transitions[i+1]
            st = states_list[idx_start]
            color = state_colors.get(st, "#CCCCCC")
            x0 = t_sec[idx_start]
            x1 = t_sec[idx_end]
            ax.axvspan(x0, x1, alpha=state_alpha, color=color, zorder=0)

        # State label bar at the top of the axes
        if place_top:
            ylo, yhi = ax.get_ylim()
            bar_height = (yhi - ylo) * 0.06
            bar_bottom = yhi - bar_height
            for i in range(len(transitions) - 1):
                idx_start = transitions[i]
                idx_end = transitions[i+1]
                st = states_list[idx_start]
                color = state_colors.get(st, "#CCCCCC")
                x0 = t_sec[idx_start]
                x1 = t_sec[idx_end]
                ax.axvspan(x0, x1, alpha=0.35, color=color, zorder=3)
                mid_x = (x0 + x1) / 2
                ax.text(mid_x, bar_bottom + bar_height / 2, st[:4],
                        ha='center', va='center', fontsize=5.5, fontweight='bold',
                        color='black', zorder=4)

    # =========================================================================
    # Build figure: 4 rows × 2 cols + state legend strip
    # =========================================================================
    fig = plt.figure(figsize=(18, 20))
    fig.suptitle(f"Flight Data Analysis — {base_name}", fontsize=15, fontweight='bold', y=0.985)

    # Layout: rows 0-3 are 2-col data panels, row 4 is wide legend
    gs = fig.add_gridspec(5, 2, hspace=0.28, wspace=0.22,
                           left=0.07, right=0.97, top=0.96, bottom=0.06)

    axes = {}
    labels = ["A", "B", "C", "D", "E", "F", "G", "H"]

    # --- Row 0, Col 0: Acceleration (3-axis) ---
    ax = fig.add_subplot(gs[0, 0])
    axes["accel"] = ax
    ax.plot(t_sec, accel_x, label="X", color="#E41A1C", linewidth=0.6)
    ax.plot(t_sec, accel_y, label="Y", color="#377EB8", linewidth=0.6)
    ax.plot(t_sec, accel_z, label="Z", color="#4DAF4A", linewidth=0.6)
    add_state_regions(ax)
    ax.set_ylabel("Acceleration (m/s²)", fontsize=9)
    ax.set_title("A — Accelerometer (3-axis)", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 0, Col 1: Gyroscope (3-axis) ---
    ax = fig.add_subplot(gs[0, 1])
    axes["gyro"] = ax
    ax.plot(t_sec, gyro_x, label="X", color="#E41A1C", linewidth=0.6)
    ax.plot(t_sec, gyro_y, label="Y", color="#377EB8", linewidth=0.6)
    ax.plot(t_sec, gyro_z, label="Z", color="#4DAF4A", linewidth=0.6)
    add_state_regions(ax)
    ax.set_ylabel("Angular Rate (rad/s)", fontsize=9)
    ax.set_title("B — Gyroscope (3-axis)", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 1, Col 0: G-Force & Acceleration Magnitude ---
    ax = fig.add_subplot(gs[1, 0])
    axes["gforce"] = ax
    ax.plot(t_sec, g_force, color="#D95F02", linewidth=0.8, label="G-Force")
    ax.axhline(y=1.0, color="gray", linestyle="--", alpha=0.5, linewidth=0.7, label="1G (gravity)")
    # Mark launch threshold if exceeded
    if np.max(g_force) > 2.0:
        launch_thresh = 20.0 / 9.81  # 20 m/s² → ~2.04 G
        ax.axhline(y=launch_thresh, color="red", linestyle=":", alpha=0.4, linewidth=0.7,
                   label=f"Launch ({launch_thresh:.1f}G)")
    add_state_regions(ax)
    ax.set_ylabel("G-Force (g)", fontsize=9)
    ax.set_title("C — G-Force / Accel Magnitude", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 1, Col 1: Altitude ---
    ax = fig.add_subplot(gs[1, 1])
    axes["alt"] = ax
    ax.plot(t_sec, altitude, color="#1B9E77", linewidth=1.0)
    # Mark apogee
    if len(altitude) > 0:
        i_max = np.argmax(altitude)
        ax.plot(t_sec[i_max], altitude[i_max], marker='v', color='red', markersize=8, zorder=5)
        ax.annotate(f"Apogee: {altitude[i_max]:.1f} m",
                     xy=(t_sec[i_max], altitude[i_max]),
                     xytext=(t_sec[i_max] + (t_end - t_start) * 0.02, altitude[i_max] * 0.8),
                     fontsize=7, color='red', fontweight='bold',
                     arrowprops=dict(arrowstyle="->", color='red', lw=0.8))
    add_state_regions(ax)
    ax.set_ylabel("Altitude MSL (m)", fontsize=9)
    ax.set_title("D — Altitude", fontsize=10, fontweight='bold', loc='left')
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 2, Col 0: Temperatures ---
    ax = fig.add_subplot(gs[2, 0])
    axes["temp"] = ax
    ax.plot(t_sec, core_temp, label="Core (RP2350)", color="#E7298A", linewidth=0.8)
    ax.plot(t_sec, baro_temp, label="Barometer (BMP390)", color="#66A61E", linewidth=0.8)
    add_state_regions(ax)
    ax.set_ylabel("Temperature (°C)", fontsize=9)
    ax.set_title("E — Temperatures", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 2, Col 1: Battery & Pyro Voltages ---
    ax = fig.add_subplot(gs[2, 1])
    axes["power"] = ax
    ax.plot(t_sec, bat_v, label="Battery", color="#E6AB02", linewidth=1.0)
    ax.plot(t_sec, pyro1_v, label="Pyro 1", color="#D95F02", linewidth=0.7, linestyle='--')
    ax.plot(t_sec, pyro2_v, label="Pyro 2", color="#7570B3", linewidth=0.7, linestyle='--')
    add_state_regions(ax)
    ax.set_ylabel("Voltage (V)", fontsize=9)
    ax.set_title("F — Battery & Pyro Channels", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 3, Col 0: Flash Usage ---
    ax = fig.add_subplot(gs[3, 0])
    axes["flash"] = ax
    ax.fill_between(t_sec, 0, flash_used, color="teal", alpha=0.3, step='mid')
    ax.plot(t_sec, flash_used, color="#006d5b", linewidth=0.8, drawstyle='steps-mid')
    add_state_regions(ax)
    ax.set_ylabel("Flash Used (%)", fontsize=9)
    ax.set_title("G — Flash Memory Usage", fontsize=10, fontweight='bold', loc='left')
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 3, Col 1: Descent Rate (derived from altitude) ---
    ax = fig.add_subplot(gs[3, 1])
    axes["descent"] = ax
    # Compute vertical velocity from altitude (smooth with moving average)
    if len(altitude) > 5:
        alt_arr = np.array(altitude, dtype=float)
        # Use unique timestamps to avoid divide-by-zero from duplicate ms
        t_uniq, inv = np.unique(t_sec, return_inverse=True)
        alt_uniq = np.array([np.mean(alt_arr[inv == i]) for i in range(len(t_uniq))])
        # Central difference on unique time points
        vz_uniq = np.gradient(alt_uniq) / np.gradient(t_uniq)
        # Map back to original (non-unique) time axis via nearest-neighbor
        vz = vz_uniq[inv]
        # Simple moving-average smoother
        window = max(3, len(vz) // 200)
        if window > 1:
            kernel = np.ones(window) / window
            vz_smooth = np.convolve(vz, kernel, mode='same')
        else:
            vz_smooth = vz
        ax.plot(t_sec, vz_smooth, color="#C51B7D", linewidth=0.8)
        ax.axhline(y=0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)
        # Threshold line for chute deployment decision
        ax.axhline(y=-5.0, color="red", linestyle=":", alpha=0.4, linewidth=0.7,
                   label="Chute threshold")
        ax.legend(loc="upper right", fontsize=7)
    add_state_regions(ax)
    ax.set_ylabel("Vertical Velocity (m/s)", fontsize=9)
    ax.set_title("H — Descent Rate (dAlt/dt)", fontsize=10, fontweight='bold', loc='left')
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 4: Full-width flight state legend + key stats ---
    ax_legend = fig.add_subplot(gs[4, :])
    ax_legend.axis('off')

    # State color patches
    states_present = sorted(set(states_list), key=lambda s: list(state_colors.keys()).index(s) if s in state_colors else 99)
    legend_patches = [Patch(facecolor=state_colors.get(s, "#CCCCCC"), alpha=0.7, label=s)
                      for s in states_present]

    # Key stats
    max_g = np.max(g_force)
    max_alt = np.max(altitude) if altitude else 0
    max_accel = np.max(accel_mag)
    avg_bat = np.mean(bat_v) if bat_v else 0
    duration = t_end - t_start

    stats_text = (
        f"Duration: {duration:.1f}s    "
        f"Max Altitude: {max_alt:.1f} m    "
        f"Max G-Force: {max_g:.1f} G    "
        f"Max Acceleration: {max_accel:.1f} m/s²    "
        f"Avg Battery: {avg_bat:.3f} V    "
        f"Total Frames: {len(timestamps)}"
    )

    ax_legend.legend(handles=legend_patches, loc='upper center', ncol=len(states_present),
                     fontsize=8, framealpha=0.9, edgecolor='gray')
    ax_legend.text(0.5, 0.05, stats_text, ha='center', va='bottom', fontsize=8,
                   family='monospace', transform=ax_legend.transAxes)

    # Share X on all data panels
    for key, ax in axes.items():
        ax.set_xlabel("")
        if key in ("descent", "flash"):
            ax.set_xlabel("Time (s)", fontsize=9)

    # Save outputs
    print(f"[CLI] Saving PNG: {output_png}")
    fig.savefig(output_png, dpi=150, bbox_inches="tight")
    print(f"[CLI] Saving PDF: {output_pdf}")
    fig.savefig(output_pdf, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"[CLI] Charts exported successfully:")
    print(f"       PNG: {output_png}")
    print(f"       PDF: {output_pdf}")
    print("> ", end="", flush=True)


def serial_reader(ser):
    global STATE, BUFFER
    while True:
        try:
            if STATE == "NORMAL":
                line = ser.readline()
                if b"DUMP_START" in line:
                    print("\n[CLI] Dump started. Downloading binary flash data...")
                    STATE = "DUMPING"
                    BUFFER = bytearray()
                elif line:
                    print(line.decode('utf-8', errors='replace'), end='')
            elif STATE == "DUMPING":
                chunk = ser.read(1024)
                if chunk:
                    BUFFER.extend(chunk)
                if b"\nDUMP_END" in BUFFER:
                    print("[CLI] Download finished. Parsing to CSV...")
                    parse_dump(BUFFER)
                    BUFFER = bytearray()
                    STATE = "NORMAL"
        except serial.SerialException:
            print("\n[CLI] Serial connection lost.")
            break
        except Exception as e:
            print(f"\n[CLI] Error in reader thread: {e}")
            break


def main():
    # Check for direct CLI commands that don't need a serial port
    if len(sys.argv) >= 2 and sys.argv[1].upper() == "EXPORT_CSV":
        if len(sys.argv) >= 3:
            export_csv_charts(sys.argv[2])
        else:
            print("Usage: python tool.py EXPORT_CSV <filename_core.csv>")
            print("Example: python tool.py EXPORT_CSV flight_data_1788270073_core.csv")
        sys.exit(0)

    if len(sys.argv) < 2:
        print("Usage: python tool.py <COM_PORT>")
        print("Example: python tool.py COM3")
        print("       python tool.py EXPORT_CSV <filename_core.csv>")
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port {port}: {e}")
        sys.exit(1)

    print(f"[CLI] Connected to {port}. Type commands and press Enter.")
    print("[CLI] Commands: STATUS, DUMP_FLASH, WIPE_FLASH, SIM_LAUNCH, P1_FIRE, RADIO_TEST, etc.")
    print("[CLI] Local commands: EXPORT_CSV <filename> — generate charts from a CSV file")

    reader_thread = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
    reader_thread.start()

    time.sleep(0.5)

    try:
        while True:
            cmd = input("> ").strip()
            if cmd:
                if cmd.upper().startswith("EXPORT_CSV"):
                    parts = cmd.split(maxsplit=1)
                    if len(parts) == 2:
                        export_csv_charts(parts[1])
                    else:
                        print("[CLI] Usage: EXPORT_CSV <filename_core.csv>")
                        print("> ", end="", flush=True)
                else:
                    ser.write(f"{cmd}\n".encode('utf-8'))
                    time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[CLI] Exiting.")
        ser.close()
        sys.exit(0)


if __name__ == '__main__':
    main()