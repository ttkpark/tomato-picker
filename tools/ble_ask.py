"""젯슨 IP를 **화면 없이** BLE로 묻는다 — SSH가 안 될 때(브라운아웃 뒤 IP가 바뀜).
    pip install bleak
    python tools/ble_ask.py              # ip, status
    python tools/ble_ask.py ip wifi sys  # 아무 명령이나 순서대로
`ble_handset_pc.py`(tkinter 화면)와 같은 프로토콜(NUS, auth 토큰, 0x04 종단)인데 화면이
없어서 에이전트/스크립트에서 쓴다. 토큰은 ~/.tomato_handset.json 또는 기본값.
⚠ 윈도우 BLE 스캔은 성겨서 40초까지 기다린다. "NOT FOUND"면 젯슨이 아직 부팅 중이거나
   ble-console이 죽은 것 — 30초 뒤 다시.
⚠ 젯슨 와이파이 MAC은 f0-68-e3-7f-48-d5(BLE는 …:D4) — ARP 표에서 찾는 길도 있다.
"""
import asyncio, json, os, sys
from bleak import BleakScanner, BleakClient

NUS_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
EOT = 0x04
NAME = "tomato-jetson"
cfg = os.path.join(os.path.expanduser("~"), ".tomato_handset.json")
TOKEN = "d270dba7"
try:
    TOKEN = json.load(open(cfg)).get("token") or TOKEN
except Exception:
    pass
CMDS = sys.argv[1:] or ["ip", "status"]


async def main():
    found = {}

    def cb(dev, adv):
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        if (adv.local_name or dev.name) == NAME or NUS_SVC in uuids:
            found[dev.address] = (dev, adv.rssi)

    scanner = BleakScanner(cb)
    await scanner.start()
    for _ in range(40):
        await asyncio.sleep(1)
        if found:
            break
    await scanner.stop()
    if not found:
        print("NOT FOUND (40s scan)")
        return 2
    addr, (dev, rssi) = next(iter(found.items()))
    print(f"found {addr} rssi={rssi}")
    buf = bytearray()
    lines = asyncio.Queue()

    def on_notify(_s, data):
        for b in data:
            if b == EOT:
                lines.put_nowait(buf.decode("utf-8", "replace"))
                buf.clear()
            else:
                buf.append(b)

    for attempt in range(3):
        try:
            async with BleakClient(dev, timeout=20) as client:
                await client.start_notify(NUS_TX, on_notify)

                async def send(line):
                    data = (line + "\n").encode()
                    for i in range(0, len(data), 20):
                        await client.write_gatt_char(NUS_RX, data[i:i + 20], response=True)
                    try:
                        return await asyncio.wait_for(lines.get(), 15)
                    except asyncio.TimeoutError:
                        return "<no reply>"

                print("auth:", await send("auth " + TOKEN))
                for c in CMDS:
                    print(f"> {c}\n{await send(c)}")
                return 0
        except Exception as e:
            print("attempt", attempt, "failed:", e)
            await asyncio.sleep(0.7)
    return 1


sys.exit(asyncio.run(main()))
