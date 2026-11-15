# tools

Host-side helpers.  None of this is part of the firmware image.

| file | what it does |
|---|---|
| `build_on_mac.sh` | cross-build on a second (faster) machine: rsync → `idf.py build` → pull artefacts back → optional flash. Also drops a stale remote `sdkconfig` when `sdkconfig.defaults` is newer, otherwise a new Kconfig option never reaches the build. |
| `gen_font.py` | bakes the anti-aliased fonts in `main/ui_fonts.c` from DejaVu (bold 44/19/15/12 px). |
| `host/render.c` | compiles the **real** `main/ui.c` on the host and dumps a PPM, so the layout and colours can be checked without flashing. `gcc -I tools/host -I main/include -I main tools/host/render.c main/ui_font.c main/ui_fonts.c -o /tmp/render -lm && /tmp/render [tx] [vol]` |
| `console.py` | opens the USB-Serial-JTAG console and survives the CDC re-enumeration after a reset (`python3 tools/console.py 120`). |
| `probe_ptt.py` | keys/unkeys the radio through the **server** (no radio hardware involved) and prints the state updates, so "device → server" and "server → radio" can be told apart. |
| `probe_ptt2.py` | same, but watches the TX meters (`power_meter`, `swr_meter`, `alc_meter`, `id_meter`) plus `active_freq`/`mode`, i.e. answers "did the radio actually transmit". |
| `fetch.sh` | download one file through `proxy.env`'s proxy, falling back to direct. Same release asset: 66 s direct vs 3.1 s proxied. |
| `probe_mrrc_ws.py` | read-only probe of all four sockets (login, `/WSradio`, `/WSspectrum`, `/WSaudioRX`) — the reference for what the firmware must send. |

`probe_*` take the server host as an optional first argument (default
`192.168.1.217`) and read the password from `PASSWORD` at the top of the file.
