#!/usr/bin/env python3
"""Key the radio through the MRRC server, print the server's reply, unkey.

Runs from this machine (no radio hardware involved) so it separates
"device PTT button -> server" from "server -> radio".
"""
import asyncio, json, ssl, sys
import urllib.request
import websockets

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.217"
PORT = 8888
PASSWORD = "xy.cheenle"
BASE = f"https://{HOST}:{PORT}"
WS = f"wss://{HOST}:{PORT}"


async def main():
    body = json.dumps({"password": PASSWORD}).encode()
    req = urllib.request.Request(f"{BASE}/api/auth/login", data=body,
                                 headers={"Content-Type": "application/json"})
    import ssl as _ssl
    _ctx = _ssl.create_default_context()
    _ctx.check_hostname = False
    _ctx.verify_mode = _ssl.CERT_NONE
    with urllib.request.urlopen(req, timeout=10, context=_ctx) as resp:
        token = json.loads(resp.read().decode())["token"]
    hdr = {"Authorization": f"Bearer {token}"}
    sslctx = ssl.create_default_context()
    sslctx.check_hostname = False
    sslctx.verify_mode = ssl.CERT_NONE

    async with websockets.connect(f"{WS}/WSradio", additional_headers=hdr,
                                  ssl=sslctx, open_timeout=10) as ws:
        msg = None
        for _ in range(6):
            cand = json.loads(await asyncio.wait_for(ws.recv(), 10))
            print("  msg:", cand.get("type"))
            if cand.get("type") == "fullState":
                msg = cand
                break
        d = (msg or {}).get("data", {})
        d2 = dict(d)
        print("状态快照:")
        for k in ("active_freq", "mode_display", "mode", "tx_status", "serial_connected",
                  "power_on", "power_watts", "swr_ratio", "af_gain"):
            print(f"  {k:18} = {d2.get(k)!r}")

        if d2.get("serial_connected") is False:
            print("\n!! serial_connected = False —— 服务器连不上电台串口，PTT 不可能生效")
            return

        print("\n发送 PTT ON ...")
        await ws.send(json.dumps({"type": "set", "field": "ptt", "value": True}))
        await asyncio.sleep(1.5)
        # drain whatever the server pushed
        try:
            while True:
                u = json.loads(await asyncio.wait_for(ws.recv(), 1.0))
                fs = u.get("fields", {})
                print("  <-", u.get("type"), u.get("dirty", ""),
                      {k: fs[k] for k in ("tx_status", "power_watts", "swr_ratio")
                       if k in fs} or str(fs)[:80])
        except asyncio.TimeoutError:
            pass

        print("发送 PTT OFF ...")
        await ws.send(json.dumps({"type": "set", "field": "ptt", "value": False}))
        await asyncio.sleep(1.0)
        try:
            while True:
                u = json.loads(await asyncio.wait_for(ws.recv(), 1.0))
                fs = u.get("fields", {})
                print("  <-", u.get("type"), u.get("dirty", ""),
                      {k: fs[k] for k in ("tx_status", "power_watts", "swr_ratio")
                       if k in fs} or str(fs)[:80])
        except asyncio.TimeoutError:
            pass
    print("完成")


asyncio.run(main())
