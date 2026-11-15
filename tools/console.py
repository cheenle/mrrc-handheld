#!/usr/bin/env python3
"""Capture the board console across USB-CDC re-enumeration.

`cat /dev/ttyACM0` dies with the old fd when the chip resets (the CDC
interface re-enumerates); this reopens the port and keeps reading.

    ./console.py [seconds]
"""
import os, select, sys, time

import glob as _glob


def _find_port():
    """The USB-JTAG CDC reappears as ttyACM0/ttyACM1/... — always resolve by id."""
    cands = _glob.glob("/dev/serial/by-id/usb-Espressif_USB_JTAG*")
    if cands:
        return cands[0]
    acm = sorted(_glob.glob("/dev/ttyACM*"))
    return acm[0] if acm else "/dev/ttyACM0"


PORT = _find_port()
DEADLINE = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 30)
buf = b""
fd = None

while time.time() < DEADLINE:
    if fd is None:
        try:
            real = _find_port()
            fd = os.open(real, os.O_RDONLY | os.O_NONBLOCK)
            sys.stderr.write("[console] opened %s\n" % PORT)
        except OSError:
            time.sleep(0.2)
            continue
    try:
        r, _, _ = select.select([fd], [], [], 0.4)
        if r:
            chunk = os.read(fd, 4096)
            if chunk:
                buf += chunk
                sys.stdout.write(chunk.decode("utf-8", "replace"))
                sys.stdout.flush()
    except OSError:
        try:
            os.close(fd)
        except OSError:
            pass
        fd = None
        sys.stderr.write("[console] port lost, reopening\n")
        time.sleep(0.2)

if fd is not None:
    os.close(fd)
sys.stderr.write("[console] captured %d bytes\n" % len(buf))
