#!/usr/bin/env python3
"""
dump_flash.py — Download raw flight data from CARP avionics via serial.

Connects to the CARP board, sends DUMP_FLASH, captures the binary stream,
and saves it as a raw .bin file for offline parsing.

Usage:
    python dump_flash.py <COM_PORT>
    python dump_flash.py COM3
"""

import sys
import time
import serial


def main():
    if len(sys.argv) < 2:
        print("Usage: python dump_flash.py <COM_PORT>")
        print("Example: python dump_flash.py COM3")
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, 115200, timeout=2)
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        sys.exit(1)

    # Flush any stale input
    time.sleep(1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"[DUMP] Connected to {port}")
    print("[DUMP] Sending DUMP_FLASH...")
    ser.write(b"DUMP_FLASH\n")
    time.sleep(0.5)

    # Read until we see DUMP_START
    print("[DUMP] Waiting for DUMP_START...")
    buf = bytearray()
    while True:
        chunk = ser.read(1024)
        if not chunk:
            continue
        buf.extend(chunk)
        idx = buf.find(b"DUMP_START")
        if idx >= 0:
            # Discard everything up to and including "DUMP_START"
            tail = buf[idx + len(b"DUMP_START"):]
            # Strip the trailing CRLF (\r\n) that println() appends
            while tail and tail[0] in (b'\n'[0], b'\r'[0]):
                tail = tail[1:]
            buf = bytearray(tail)
            print("[DUMP] Download started...")
            break

    # Read until DUMP_END
    total = 0
    last_report = time.monotonic()
    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
            total += len(chunk)
            now = time.monotonic()
            if now - last_report >= 1.0:
                print(f"[DUMP] Received {total // 1024} KB...", end="\r")
                last_report = now
            idx = buf.find(b"DUMP_END")
            if idx >= 0:
                buf = buf[:idx]
                break

    print(f"\n[DUMP] Download complete: {len(buf)} bytes ({len(buf) / 1024:.1f} KB)")

    # Save to timestamped .bin file
    ts = int(time.time())
    filename = f"flight_data_{ts}.bin"
    with open(filename, "wb") as f:
        f.write(buf)
    print(f"[DUMP] Saved to {filename}")

    ser.close()
    print("[DUMP] Done.")


if __name__ == "__main__":
    main()