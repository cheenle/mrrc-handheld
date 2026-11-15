#!/usr/bin/env python3
"""Key the radio via the MRRC server and watch the TX meters.

Answers: does the radio actually go into transmit (tx_status), and does it
produce RF (power_meter > 0) with a sane SWR?
"""
import asyncio, json, ssl, sys
import urllib.request
import websockets

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.217"
PORT = 8888
PASSWORD = "xy.cheenle"
BASE, WS = f"https://{HOST}:{PORT}", f"wss://{HOST}:{PORT}"
TX_KEYS = ("tx_status", "power_meter", "swr_meter", "alc_meter", "comp_meter",
           "vd_meter", "id_meter", "power_watts", "swr_ratio", "tuner_status",
           "rf_power", "mic_gain", "mode", "active_freq", "vox", "split")


def ctx():
    c = ssl.create_default_context()
    c.check_hostname = False
    c.verify_mode = ssl.CERT_NONE
    return c


async def main():
    body = json.dumps({"password": PASSWORD}).encode()
    req = urllib.request.Request(f"{BASE}/api/auth/login", data=body,
                                 headers={"Content-Type": "application/json"})
    token = json.loads(urllib.request.urlopen(req, timeout=10, context=ctx()).read())["token"]

    async with websockets.connect(f"{WS}/WSradio",
                                  additional_headers={"Authorization": f"Bearer {token}"},
                                  ssl=ctx(), open_timeout=10) as ws:
        state = {}
        for _ in range(8):
            m = json.loads(await asyncio.wait_for(ws.recv(), 10))
            if m.get("type") == "fullState":
                state.update(m["data"])
                break

        print("=== 发射前 ===")
        for k in TX_KEYS:
            if k in state:
                print(f"  {k:15} = {state[k]!r}")

        async def drain(seconds, tag):
            end = asyncio.get_event_loop().time() + seconds
            while asyncio.get_event_loop().time() < end:
                try:
                    m = json.loads(await asyncio.wait_for(ws.recv(), 0.7))
                except asyncio.TimeoutError:
                    continue
                f = m.get("fields", {})
                hit = {k: f[k] for k in f if k in TX_KEYS}
                if hit:
                    print(f"  [{tag}] {hit}")

        print("\n>>> PTT ON")
        await ws.send(json.dumps({"type": "set", "field": "ptt", "value": True}))
        await drain(3.0, "TX")

        print("\n>>> PTT OFF")
        await ws.send(json.dumps({"type": "set", "field": "ptt", "value": False}))
        await drain(1.0, "RX")
    print("完成")


asyncio.run(main())
