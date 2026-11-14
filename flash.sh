#!/usr/bin/env bash
# Flash MRRC Handheld onto the coolkit-xz-02 / xingzhi-cube-1.54tft-wifi board.
#
#   ./flash.sh            normal flash (app + bootloader + partition table)
#   ./flash.sh monitor    flash, then open the USB-JTAG console
#
# Restore the stock xiaozhi firmware (16 MB full-chip image) with:
#   ~/esp/bin/esptool.py -p /dev/ttyACM0 -b 921600 write_flash 0x0 \
#     ~/esp-backup-20261114-0151/esp32s3_flash_16MB.bin
set -euo pipefail

PORT="${MRRC_PORT:-$(ls /dev/serial/by-id/usb-Espressif_USB_JTAG* 2>/dev/null | head -1)}"
PORT="${PORT:-/dev/ttyACM0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$HOME/esp-idf/export.sh" >/dev/null 2>&1
cd "$HERE"

if [ ! -f build/mrrc_handheld.bin ]; then
    echo "no build yet — running idf.py build"
    idf.py build
fi

if [ "${1:-}" = "monitor" ]; then
    idf.py -p "$PORT" -b 921600 flash monitor
else
    idf.py -p "$PORT" -b 921600 flash
fi
