#!/usr/bin/env python3
"""
parse_flight.py — Parse raw .bin flight data to CSV and generate charts.

Reads a binary dump from dump_flash.py (or any raw 32-byte frame stream),
extracts CORE (APID 0) and GPS (APID 1) frames, writes CSV files, and
produces a multi-panel chart figure.

Usage:
    python parse_flight.py <file.bin>
    python parse_flight.py flight_data_1788270073.bin
    python parse_flight.py flight_data_1788270073.bin --no-charts
"""

import sys
import os
import csv
import time
import struct
import argparse

import numpy as np

# ---------------------------------------------------------------------------
# Constants (must match RapidFish_v2.ino)
# ---------------------------------------------------------------------------
FRAME_SIZE = 32
SYNC_WORD = 0x1ACFFC1D

FLIGHT_STATES = [
    "BOOTING",       # 0
    "DISARMED",      # 1
    "ERROR",         # 2
    "ARMED",         # 3
    "ACCELERATING",  # 4
    "COAST",         # 5
    "RECOVERY",      # 6
    "CHUTE",         # 7
    "GROUND",        # 8
]

# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_bin(data: bytes):
    """Yield (sync, timestamp, apid, raw_chunk) for each valid frame.

    Scans for the first sync word to handle leading garbage bytes
    (e.g. stray newlines from serial framing), then walks in 32-byte
    steps from there. Skips any frame whose sync word doesn't match.
    """
    # Find the first sync word to establish alignment.
    # The sync word is the first 4 bytes of every 32-byte frame, so
    # the first occurrence tells us exactly where frame data begins.
    SYNC_BYTES = struct.pack('<I', SYNC_WORD)
    first_sync = data.find(SYNC_BYTES)
    if first_sync < 0:
        return  # no valid frames at all

    for i in range(first_sync, len(data), FRAME_SIZE):
        chunk = data[i:i + FRAME_SIZE]
        if len(chunk) < FRAME_SIZE:
            break
        sync, timestamp, apid = struct.unpack_from('<IIB', chunk, 0)
        if sync != SYNC_WORD:
            continue
        yield sync, timestamp, apid, chunk


def pressure_to_altitude(pressure_pa: float, ref_pressure_hpa: float = 1013.25) -> float:
    """Convert barometric pressure in Pa to altitude in meters using the
    international barometric formula."""
    if pressure_pa <= 0 or ref_pressure_hpa <= 0:
        return 0.0
    ratio = (pressure_pa / 100.0) / ref_pressure_hpa
    return 44330.0 * (1.0 - ratio ** (1.0 / 5.255))


def parse_core(chunk: bytes):
    """Unpack a CORE frame (APID 0). Returns dict.

    Matches LogFrameCore in RapidFish_v2.ino (dataframes.md).
    """
    # Format: IIBBBbhhhhhhIbBBB  = 32 bytes
    #   I  I  B B B b h h h h h h I  b B B B
    #   sy ts ap fl fl co ax ay az gx gy gz pr te ba p1 p2
    fields = struct.unpack_from('<IIBBBbhhhhhhIbBBB', chunk, 0)
    _, ts, _, f_state, flash_u, core_t, \
        ax, ay, az, gx, gy, gz, \
        pressure, baro_t, bat, p1, p2 = fields

    # Compute altitude from pressure using standard atmosphere
    alt_m = pressure_to_altitude(pressure)

    return {
        "Timestamp_ms": ts,
        "State": FLIGHT_STATES[f_state] if f_state < len(FLIGHT_STATES) else str(f_state),
        "Flash_Used_%": flash_u / 2.0,
        "Core_Temp_C": core_t,
        "Accel_X_ms2": ax / 100.0,
        "Accel_Y_ms2": ay / 100.0,
        "Accel_Z_ms2": az / 100.0,
        "Gyro_X_rads": gx / 1000.0,
        "Gyro_Y_rads": gy / 1000.0,
        "Gyro_Z_rads": gz / 1000.0,
        "Pressure_Pa": pressure,
        "Altitude_m": alt_m,
        "Baro_Temp_C": baro_t,
        "Bat_V": (bat * 9.9) / 255.0,
        "Pyro1_V": (p1 * 9.9) / 255.0,
        "Pyro2_V": (p2 * 9.9) / 255.0,
    }


def parse_gps(chunk: bytes):
    """Unpack a GPS frame (APID 1). Returns dict."""
    fields = struct.unpack_from('<IIBiiHBBIBhhh', chunk, 0)
    _, ts, _, lat, lon, gps_alt, fix_state, sats, \
        gps_time, hdop, mx, my, mz = fields

    return {
        "Timestamp_ms": ts,
        "Latitude": lat / 1e7,
        "Longitude": lon / 1e7,
        "GPS_Alt_m": gps_alt / 2.0,
        "Fix_State": fix_state,
        "Sats": sats,
        "GPS_Time_Unix": gps_time,
        "HDOP": hdop / 10.0,
        "Mag_X": mx,
        "Mag_Y": my,
        "Mag_Z": mz,
    }


# ---------------------------------------------------------------------------
# CSV export
# ---------------------------------------------------------------------------

CORE_HEADER = [
    "Timestamp_ms", "State", "Flash_Used_%", "Core_Temp_C",
    "Accel_X_ms2", "Accel_Y_ms2", "Accel_Z_ms2",
    "Gyro_X_rads", "Gyro_Y_rads", "Gyro_Z_rads",
    "Pressure_Pa", "Altitude_m", "Baro_Temp_C",
    "Bat_V", "Pyro1_V", "Pyro2_V",
]

GPS_HEADER = [
    "Timestamp_ms", "Latitude", "Longitude", "GPS_Alt_m",
    "Fix_State", "Sats", "GPS_Time_Unix", "HDOP",
    "Mag_X", "Mag_Y", "Mag_Z",
]


def write_csv(filename, header, rows):
    with open(filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r.get(h, "") for h in header])
    print(f"  Wrote {len(rows)} rows -> {filename}")


# ---------------------------------------------------------------------------
# Charting
# ---------------------------------------------------------------------------

STATE_COLORS = {
    "BOOTING":      "#888888",
    "DISARMED":     "#AAAAAA",
    "ERROR":        "#FF0000",
    "ARMED":        "#FFA500",
    "ACCELERATING": "#FFD700",
    "COAST":        "#228B22",
    "RECOVERY":     "#1E90FF",
    "CHUTE":        "#8B008B",
    "GROUND":       "#8B4513",
}


def generate_charts(core_csv_path):
    """Read a CORE CSV and produce a multi-panel chart figure."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except ImportError:
        print("[CHART] matplotlib not installed. Install: pip install matplotlib numpy")
        return

    if not os.path.isfile(core_csv_path):
        print(f"[CHART] File not found: {core_csv_path}")
        return

    print(f"[CHART] Loading {core_csv_path}...")

    # Read CSV into lists
    timestamps, states_list = [], []
    flash_used, core_temp = [], []
    accel_x, accel_y, accel_z = [], [], []
    gyro_x, gyro_y, gyro_z = [], [], []
    altitude, baro_temp = [], []
    bat_v, pyro1_v, pyro2_v = [], [], []

    with open(core_csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
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

    if not timestamps:
        print("[CHART] No data rows.")
        return

    # Time axis (seconds from start)
    t0 = timestamps[0]
    t_sec = np.array([(ts - t0) / 1000.0 for ts in timestamps])
    t_start, t_end = t_sec[0], t_sec[-1]

    # Derived quantities
    accel_mag = np.sqrt(np.array(accel_x)**2 + np.array(accel_y)**2 + np.array(accel_z)**2)
    g_force = accel_mag / 9.81

    base_name = os.path.splitext(os.path.basename(core_csv_path))[0]
    out_png = f"{base_name}_charts.png"
    out_pdf = f"{base_name}_charts.pdf"

    # -----------------------------------------------------------------------
    # State region helper
    # -----------------------------------------------------------------------
    state_alpha = 0.12

    def add_state_regions(ax, place_top=True):
        if len(states_list) < 2:
            return
        transitions = [0]
        for i in range(1, len(states_list)):
            if states_list[i] != states_list[i - 1]:
                transitions.append(i)
        transitions.append(len(states_list) - 1)

        for i in range(len(transitions) - 1):
            idx0, idx1 = transitions[i], transitions[i + 1]
            st = states_list[idx0]
            color = STATE_COLORS.get(st, "#CCCCCC")
            ax.axvspan(t_sec[idx0], t_sec[idx1], alpha=state_alpha, color=color, zorder=0)

        if place_top:
            ylo, yhi = ax.get_ylim()
            bh = (yhi - ylo) * 0.06
            bb = yhi - bh
            for i in range(len(transitions) - 1):
                idx0, idx1 = transitions[i], transitions[i + 1]
                st = states_list[idx0]
                color = STATE_COLORS.get(st, "#CCCCCC")
                ax.axvspan(t_sec[idx0], t_sec[idx1], alpha=0.35, color=color, zorder=3)
                mx = (t_sec[idx0] + t_sec[idx1]) / 2
                ax.text(mx, bb + bh / 2, st[:4], ha='center', va='center',
                        fontsize=5.5, fontweight='bold', color='black', zorder=4)

    # -----------------------------------------------------------------------
    # Build figure
    # -----------------------------------------------------------------------
    fig = plt.figure(figsize=(18, 20))
    fig.suptitle(f"Flight Data — {base_name}", fontsize=15, fontweight='bold', y=0.985)
    gs = fig.add_gridspec(5, 2, hspace=0.28, wspace=0.22,
                           left=0.07, right=0.97, top=0.96, bottom=0.06)
    axes = {}

    # --- Row 0, Col 0: Acceleration ---
    ax = fig.add_subplot(gs[0, 0])
    axes["accel"] = ax
    ax.plot(t_sec, accel_x, label="X", color="#E41A1C", lw=0.6)
    ax.plot(t_sec, accel_y, label="Y", color="#377EB8", lw=0.6)
    ax.plot(t_sec, accel_z, label="Z", color="#4DAF4A", lw=0.6)
    add_state_regions(ax)
    ax.set_ylabel("Acceleration (m/s²)", fontsize=9)
    ax.set_title("A — Accelerometer (3-axis)", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 0, Col 1: Gyroscope ---
    ax = fig.add_subplot(gs[0, 1])
    axes["gyro"] = ax
    ax.plot(t_sec, gyro_x, label="X", color="#E41A1C", lw=0.6)
    ax.plot(t_sec, gyro_y, label="Y", color="#377EB8", lw=0.6)
    ax.plot(t_sec, gyro_z, label="Z", color="#4DAF4A", lw=0.6)
    add_state_regions(ax)
    ax.set_ylabel("Angular Rate (rad/s)", fontsize=9)
    ax.set_title("B — Gyroscope (3-axis)", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7, ncol=3)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 1, Col 0: G-Force ---
    ax = fig.add_subplot(gs[1, 0])
    axes["gforce"] = ax
    ax.plot(t_sec, g_force, color="#D95F02", lw=0.8, label="G-Force")
    ax.axhline(y=1.0, color="gray", ls="--", alpha=0.5, lw=0.7, label="1G (gravity)")
    if np.max(g_force) > 2.0:
        ax.axhline(y=2.0, color="red", ls=":", alpha=0.4, lw=0.7, label="Launch threshold")
    add_state_regions(ax)
    ax.set_ylabel("G-Force (g)", fontsize=9)
    ax.set_title("C — G-Force / Accel Magnitude", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 1, Col 1: Altitude ---
    ax = fig.add_subplot(gs[1, 1])
    axes["alt"] = ax
    ax.plot(t_sec, altitude, color="#1B9E77", lw=1.0)
    if altitude:
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
    ax.plot(t_sec, core_temp, label="Core (RP2350)", color="#E7298A", lw=0.8)
    ax.plot(t_sec, baro_temp, label="Barometer (BMP390)", color="#66A61E", lw=0.8)
    add_state_regions(ax)
    ax.set_ylabel("Temperature (°C)", fontsize=9)
    ax.set_title("E — Temperatures", fontsize=10, fontweight='bold', loc='left')
    ax.legend(loc="upper right", fontsize=7)
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 2, Col 1: Battery & Pyro ---
    ax = fig.add_subplot(gs[2, 1])
    axes["power"] = ax
    ax.plot(t_sec, bat_v, label="Battery", color="#E6AB02", lw=1.0)
    ax.plot(t_sec, pyro1_v, label="Pyro 1", color="#D95F02", lw=0.7, ls='--')
    ax.plot(t_sec, pyro2_v, label="Pyro 2", color="#7570B3", lw=0.7, ls='--')
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
    ax.plot(t_sec, flash_used, color="#006d5b", lw=0.8, drawstyle='steps-mid')
    add_state_regions(ax)
    ax.set_ylabel("Flash Used (%)", fontsize=9)
    ax.set_title("G — Flash Memory Usage", fontsize=10, fontweight='bold', loc='left')
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 3, Col 1: Descent Rate ---
    ax = fig.add_subplot(gs[3, 1])
    axes["descent"] = ax
    if len(altitude) > 5:
        alt_arr = np.array(altitude, dtype=float)
        t_uniq, inv = np.unique(t_sec, return_inverse=True)
        alt_uniq = np.array([np.mean(alt_arr[inv == i]) for i in range(len(t_uniq))])
        vz_uniq = np.gradient(alt_uniq) / np.gradient(t_uniq)
        vz = vz_uniq[inv]
        window = max(3, len(vz) // 200)
        if window > 1:
            kernel = np.ones(window) / window
            vz = np.convolve(vz, kernel, mode='same')
        ax.plot(t_sec, vz, color="#C51B7D", lw=0.8)
        ax.axhline(y=0, color="gray", ls="-", alpha=0.3, lw=0.5)
        ax.axhline(y=-15.0, color="red", ls=":", alpha=0.4, lw=0.7, label="Chute threshold")
        ax.legend(loc="upper right", fontsize=7)
    add_state_regions(ax)
    ax.set_ylabel("Vertical Velocity (m/s)", fontsize=9)
    ax.set_title("H — Descent Rate (dAlt/dt)", fontsize=10, fontweight='bold', loc='left')
    ax.grid(True, alpha=0.25, linestyle=':')
    ax.set_xlim(t_start, t_end)

    # --- Row 4: Legend + stats ---
    ax_legend = fig.add_subplot(gs[4, :])
    ax_legend.axis('off')
    states_present = sorted(set(states_list),
                            key=lambda s: list(STATE_COLORS.keys()).index(s) if s in STATE_COLORS else 99)
    legend_patches = [Patch(facecolor=STATE_COLORS.get(s, "#CCCCCC"), alpha=0.7, label=s)
                      for s in states_present]
    max_g = np.max(g_force)
    max_alt = np.max(altitude) if altitude else 0
    avg_bat = np.mean(bat_v) if bat_v else 0
    duration = t_end - t_start
    stats_text = (
        f"Duration: {duration:.1f}s    "
        f"Max Altitude: {max_alt:.1f} m    "
        f"Max G-Force: {max_g:.1f} G    "
        f"Avg Battery: {avg_bat:.3f} V    "
        f"Total Frames: {len(timestamps)}"
    )
    ax_legend.legend(handles=legend_patches, loc='upper center', ncol=len(states_present),
                     fontsize=8, framealpha=0.9, edgecolor='gray')
    ax_legend.text(0.5, 0.05, stats_text, ha='center', va='bottom', fontsize=8,
                   family='monospace', transform=ax_legend.transAxes)

    for key, ax in axes.items():
        ax.set_xlabel("")
        if key in ("descent", "flash"):
            ax.set_xlabel("Time (s)", fontsize=9)

    print(f"[CHART] Saving {out_png}...")
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[CHART] Saving {out_pdf}...")
    fig.savefig(out_pdf, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[CHART] Done -> {out_png}, {out_pdf}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Parse raw .bin flight data from CARP avionics to CSV + charts."
    )
    parser.add_argument("file", help="Path to .bin file (raw 32-byte frames)")
    parser.add_argument("--no-charts", action="store_true",
                        help="Skip chart generation (CSV only)")
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print(f"File not found: {args.file}")
        sys.exit(1)

    with open(args.file, "rb") as f:
        data = f.read()

    print(f"Read {len(data)} bytes from {args.file}")

    core_rows = []
    gps_rows = []

    for sync, ts, apid, chunk in parse_bin(data):
        if apid == 0:
            core_rows.append(parse_core(chunk))
        elif apid == 1:
            gps_rows.append(parse_gps(chunk))

    print(f"Found {len(core_rows)} CORE frames, {len(gps_rows)} GPS frames")

    # Write CSVs
    ts_now = int(time.time())
    core_csv = f"flight_data_{ts_now}_core.csv"
    gps_csv = f"flight_data_{ts_now}_gps.csv"

    write_csv(core_csv, CORE_HEADER, core_rows)
    if gps_rows:
        write_csv(gps_csv, GPS_HEADER, gps_rows)
    else:
        print(f"  (no GPS frames, skipping GPS CSV)")

    # Charts
    if not args.no_charts and core_rows:
        generate_charts(core_csv)
    elif args.no_charts:
        print("[CHART] Skipped (--no-charts)")
    else:
        print("[CHART] No core data to chart.")

    print("Done.")


if __name__ == "__main__":
    main()