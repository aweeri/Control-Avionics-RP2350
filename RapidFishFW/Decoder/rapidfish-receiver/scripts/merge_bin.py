# Standalone helper to produce a single merged (websafe) firmware binary for
# web flashers (ESP Web Flasher / esptool-js). esptool's `merge_bin` combines
# bootloader.bin + partitions.bin + firmware.bin into one image ready to flash
# at offset 0x0.
#
# Usage (after `platformio run -e <env>`):
#   python scripts/merge_bin.py <build_dir> <chip>
#
# Examples:
#   python scripts/merge_bin.py .pio/build/ttgo_t3_lora32 esp32
#   python scripts/merge_bin.py .pio/build/heltec_wireless_tracker_stick esp32s3

import os
import subprocess
import sys

def merge_firmware(build_dir, chip):
    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    firmware = os.path.join(build_dir, "firmware.bin")
    merged = os.path.join(build_dir, "merged-firmware.bin")

    for f in (bootloader, partitions, firmware):
        if not os.path.exists(f):
            raise SystemExit("missing file: %s (run `platformio run -e <env>` first)" % f)

    if chip == "esp32":
        # classic ESP32: bootloader @0x1000, partitions @0x8000, app @0x10000
        offsets = ["0x1000", bootloader, "0x8000", partitions, "0x10000", firmware]
    else:
        # new SoC (esp32s2/s3/c3): bootloader @0x0, partitions @0x8000, app @0x10000
        offsets = ["0x0", bootloader, "0x8000", partitions, "0x10000", firmware]

    # Locate the esptool shipped with PlatformIO's tool-esptoolpy package.
    home = os.path.expanduser("~")
    esptool_py = os.path.join(
        home, ".platformio", "packages", "tool-esptoolpy", "esptool.py"
    )
    if not os.path.exists(esptool_py):
        raise SystemExit("esptool.py not found at %s" % esptool_py)

    cmd = [sys.executable, esptool_py, "--chip", chip, "merge_bin", "--output", merged] + offsets
    print(">>> Merging websafe firmware -> %s" % merged)
    print(">>> " + " ".join(cmd))
    subprocess.check_call(cmd)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    merge_firmware(sys.argv[1], sys.argv[2])