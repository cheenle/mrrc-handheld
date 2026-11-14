#!/usr/bin/env bash
# Build on the mac-mini (8 cores, ~90 s) and pull the artefacts back here.
#
#   ./tools/build_on_mac.sh          # sync + build + fetch
#   ./tools/build_on_mac.sh flash     # ... and flash the device
#
# The mac-mini needs esp-idf v5.4.2 in ~/esp-idf with its tools installed
# (./install.sh esp32s3 && idf_tools.py install cmake ninja).
set -euo pipefail

MAC=${MAC:-cheenle@192.168.1.228}
REMOTE=~/dev/mrrc-handheld
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT=${MRRC_PORT:-$(ls /dev/ttyACM* 2>/dev/null | head -1)}

echo "== sync source =="
rsync -a --delete \
      --exclude build --exclude build-mac --exclude managed_components \
      --exclude sdkconfig --exclude sdkconfig.old --exclude .git \
      "$HERE/" "$MAC:$REMOTE/"

echo "== build on mac-mini =="
ssh "$MAC" 'cd '"$REMOTE"' && zsh -c ". ~/esp-idf/export.sh >/dev/null 2>&1; idf.py build" 2>&1 | tail -20

echo "== fetch artefacts =="
mkdir -p "$HERE/build-mac"
rsync -a "$MAC:$REMOTE/build/mrrc_handheld.bin" \
         "$MAC:$REMOTE/build/bootloader/bootloader.bin" \
         "$MAC:$REMOTE/build/partition_table/partition-table.bin" \
         "$MAC:$REMOTE/build/flash_args" "$HERE/build-mac/"
ls -l "$HERE/build-mac/"

if [ "${1:-}" = "flash" ]; then
    echo "== flash $PORT =="
    for pid in $(pgrep -f '^python3 .*console\.py' || true); do kill "$pid" 2>/dev/null || true; done
    sleep 1
    cd "$HERE/build-mac"
    ~/esp/bin/esptool.py -p "$PORT" -b 921600 --after watchdog-reset write_flash \
        --flash_mode dio --flash_size 16MB --flash_freq 80m \
        0x0 bootloader.bin 0x8000 partition-table.bin 0x10000 mrrc_handheld.bin
fi
