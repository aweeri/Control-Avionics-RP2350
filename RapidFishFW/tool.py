# requires pyserial: pip install pyserial
# requires tqdm: pip install tqdm
# requires matplotlib: pip install matplotlib
import serial
import sys
import threading
import struct
import csv
import time
import os

STATE = "NORMAL"
BUFFER = bytearray()

def parse_dump(raw_bytes):
    end_idx = raw_bytes.find(b"\nDUMP_END")
    if end_idx != -1:
        raw_bytes = raw_bytes[:end_idx]

    sync_core = 0x1ACFFC1D
    states = ["BOOTING", "ERROR", "ARMED", "ACCELERATING", "COAST", "RECOVERY", "CHUTE", "GROUND"]

    filename = f"flight_data_{int(time.time())}.csv"
    
    total_chunks = len(raw_bytes) // 32
    parsed_count = 0

    try:
        from tqdm import tqdm
        progress = tqdm(total=total_chunks, desc="Parsing dump", unit="frames",
                        bar_format="{l_bar}{bar}| {n_fmt}/{total_fmt} frames [{elapsed}<{remaining}]")
        has_tqdm = True
    except ImportError:
        has_tqdm = False
        print(f"\n[CLI] Parsing {total_chunks} frames...")
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp_ms", "State", "Flash_Used_%", "Core_Temp_C", 
                         "Accel_X_G", "Accel_Y_G", "Accel_Z_G", 
                         "Gyro_X_rads", "Gyro_Y_rads", "Gyro_Z_rads", 
                         "Altitude_m", "Baro_Temp_C", 
                         "Bat_V", "Pyro1_V", "Pyro2_V"])

        for i in range(0, len(raw_bytes), 32):
            chunk = raw_bytes[i:i+32]
            if len(chunk) < 32:
                break
                
            sync, timestamp, apid, f_state, flash_u, core_t, ax, ay, az, gx, gy, gz, alt, baro_t, bat, p1, p2 = struct.unpack('<IIBBBbhhhhhhHbBBBxx', chunk)

            if sync == sync_core and apid == 0:
                state_str = states[f_state] if f_state < len(states) else str(f_state)
                writer.writerow([
                    timestamp, state_str, flash_u, core_t,
                    ax / 100.0, ay / 100.0, az / 100.0,
                    gx / 1000.0, gy / 1000.0, gz / 1000.0,
                    alt / 2.0, baro_t,
                    (bat * 9.9) / 255.0, (p1 * 9.9) / 255.0, (p2 * 9.9) / 255.0
                ])
                parsed_count += 1

            if has_tqdm:
                progress.update(1)

    if has_tqdm:
        progress.close()
    
    print(f"\n[CLI] Saved {parsed_count} frames to {filename}")
    print("> ", end="", flush=True)


def export_csv_charts(csv_path):
    """Read a CSV flight data file and generate a comprehensive set of matplotlib charts."""
    if not os.path.isfile(csv_path):
        print(f"[CLI] File not found: {csv_path}")
        return

    try:
        import matplotlib
        matplotlib.use('Agg')  # Non-interactive backend
        import matplotlib.pyplot as plt
        import numpy as np
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
        total_rows = sum(1 for _ in open(csv_path, "r")) - 1  # minus header
        f.seek(0)
        next(reader)  # skip header
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
            accel_x.append(float(row["Accel_X_G"]))
            accel_y.append(float(row["Accel_Y_G"]))
            accel_z.append(float(row["Accel_Z_G"]))
            gyro_x.append(float(row["Gyro_X_rads"]))
            gyro_y.append(float(row["Gyro_Y_rads"]))
            gyro_z.append(float(row["Gyro_Z_rads"]))
            altitude.append(float(row["Altitude_m"]))
            baro_temp.append(float(row["Baro_Temp_C"]))
            bat_v.append(float(row["Bat_V"]))
            pyro1_v.append(float(row["Pyro1_V"]))
            pyro2_v.append(float(row["Pyro2_V"]))

    # Convert to numpy arrays for efficiency
    t = np.array(timestamps)
    t_sec = (t - t[0]) / 1000.0  # convert to seconds relative to start

    # Normalize timestamps to seconds from start
    base_ts = timestamps[0]
    t_sec_list = [(ts - base_ts) / 1000.0 for ts in timestamps]

    # Generate output filename based on input
    base_name = os.path.splitext(os.path.basename(csv_path))[0]
    output_png = f"{base_name}_charts.png"
    output_pdf = f"{base_name}_charts.pdf"

    print(f"[CLI] Generating charts...")

    # Create figure with subplots
    fig, axes = plt.subplots(5, 2, figsize=(16, 18))
    fig.suptitle(f"Flight Data Analysis — {base_name}", fontsize=14, fontweight='bold')
    
    # Color map for flight states
    state_colors = {
        "BOOTING": "gray",
        "ERROR": "red",
        "ARMED": "orange",
        "ACCELERATING": "yellow",
        "COAST": "green",
        "RECOVERY": "blue",
        "CHUTE": "purple",
        "GROUND": "brown"
    }

    def add_state_background(ax, t_sec, states):
        """Add colored vertical spans for each flight state region."""
        if len(states) < 2:
            return
        current_state = states[0]
        start_t = t_sec[0]
        for i in range(1, len(states)):
            if states[i] != current_state or i == len(states) - 1:
                end_t = t_sec[i]
                color = state_colors.get(current_state, "lightgray")
                ax.axvspan(start_t, end_t, alpha=0.08, color=color)
                # Label the region at midpoint
                mid_t = (start_t + end_t) / 2
                ax.text(mid_t, ax.get_ylim()[1] * 0.95, current_state[:4],
                        ha='center', va='top', fontsize=6, alpha=0.6,
                        bbox=dict(boxstyle="round,pad=0.1", facecolor="white", alpha=0.5))
                current_state = states[i]
                start_t = t_sec[i]

    # 1. Acceleration (row 0, col 0)
    ax = axes[0, 0]
    ax.plot(t_sec_list, accel_x, label="X", alpha=0.8)
    ax.plot(t_sec_list, accel_y, label="Y", alpha=0.8)
    ax.plot(t_sec_list, accel_z, label="Z", alpha=0.8)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Acceleration (G)")
    ax.set_title("Accelerometer")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2. Gyroscope (row 0, col 1)
    ax = axes[0, 1]
    ax.plot(t_sec_list, gyro_x, label="X", alpha=0.8)
    ax.plot(t_sec_list, gyro_y, label="Y", alpha=0.8)
    ax.plot(t_sec_list, gyro_z, label="Z", alpha=0.8)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Angular Rate (rad/s)")
    ax.set_title("Gyroscope")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3. Altitude (row 1, col 0)
    ax = axes[1, 0]
    ax.plot(t_sec_list, altitude, color="darkgreen", linewidth=1.5)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude")
    ax.grid(True, alpha=0.3)

    # 4. Temperatures (row 1, col 1)
    ax = axes[1, 1]
    ax.plot(t_sec_list, core_temp, label="Core", color="red", alpha=0.8)
    ax.plot(t_sec_list, baro_temp, label="Barometer", color="blue", alpha=0.8)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Temperature (°C)")
    ax.set_title("Temperatures")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 5. Battery Voltage (row 2, col 0)
    ax = axes[2, 0]
    ax.plot(t_sec_list, bat_v, color="darkorange", linewidth=1.5)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Battery Voltage")
    ax.grid(True, alpha=0.3)

    # 6. Pyro Voltages (row 2, col 1)
    ax = axes[2, 1]
    ax.plot(t_sec_list, pyro1_v, label="Pyro 1", color="darkred", alpha=0.8)
    ax.plot(t_sec_list, pyro2_v, label="Pyro 2", color="darkviolet", alpha=0.8)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Pyro Channel Voltages")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 7. Flash Usage (row 3, col 0)
    ax = axes[3, 0]
    ax.plot(t_sec_list, flash_used, color="teal", linewidth=1.5)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Usage (%)")
    ax.set_title("Flash Memory Used")
    ax.grid(True, alpha=0.3)

    # 8. Core Temperature detail (row 3, col 1)
    ax = axes[3, 1]
    ax.plot(t_sec_list, core_temp, color="crimson", linewidth=1.5)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("Temperature (°C)")
    ax.set_title("Core Temperature Detail")
    ax.grid(True, alpha=0.3)

    # 9. State histogram (row 4, col 0)
    ax = axes[4, 0]
    state_counts = {}
    for s in states_list:
        state_counts[s] = state_counts.get(s, 0) + 1
    unique_states = list(state_counts.keys())
    counts = [state_counts[s] for s in unique_states]
    colors = [state_colors.get(s, "lightgray") for s in unique_states]
    bars = ax.bar(range(len(unique_states)), counts, color=colors, edgecolor="black", linewidth=0.5)
    ax.set_xticks(range(len(unique_states)))
    ax.set_xticklabels(unique_states, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Frame Count")
    ax.set_title("Flight State Distribution")
    ax.grid(True, alpha=0.3, axis="y")

    # 10. Acceleration magnitude (row 4, col 1)
    ax = axes[4, 1]
    accel_mag = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(accel_x, accel_y, accel_z)]
    ax.plot(t_sec_list, accel_mag, color="darkblue", linewidth=1.5)
    add_state_background(ax, t_sec_list, states_list)
    ax.set_ylabel("|A| (G)")
    ax.set_title("Acceleration Magnitude")
    ax.axhline(y=1.0, color="gray", linestyle="--", alpha=0.5, label="1G (gravity)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Common x-label for bottom row
    for ax in axes[4, :]:
        ax.set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0, 1, 0.97])

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
            print("Usage: python tool.py EXPORT_CSV <filename.csv>")
            print("Example: python tool.py EXPORT_CSV flight_data_1788270073.csv")
        sys.exit(0)

    if len(sys.argv) < 2:
        print("Usage: python cli.py <COM_PORT>")
        print("Example: python cli.py COM3")
        print("       python tool.py EXPORT_CSV <filename.csv>")
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
                        print("[CLI] Usage: EXPORT_CSV <filename.csv>")
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