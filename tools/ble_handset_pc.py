"""젯슨 핸드셋 (PC판) — 블루투스로 젯슨에 붙어 IP를 묻고 망을 갈아탄다.

안드로이드 앱([`android/handset/`](../android/handset/))과 같은 일을 하는 데스크톱
판이다. 젯슨 쪽 서버는 [`tools/ble_console.py`](ble_console.py) 그대로 쓴다.

    python tools/ble_handset_pc.py

필요한 것: `pip install bleak` (윈도우에서는 WinRT, 맥은 CoreBluetooth,
리눅스는 BlueZ를 쓴다 — 코드는 같다). tkinter는 파이썬에 들어 있다.

왜 브라우저가 아닌가
--------------------
Web Bluetooth로 만들어 봤지만 샌드박스 iframe에서는 권한정책에 막힌다(실측:
`Access to the feature "bluetooth" is disallowed by permissions policy`). 최상위
탭에서 https로 열면 되긴 하나, 그러려면 페이지를 어딘가에 올려야 하고 — 그건
**네트워크가 필요하다**. 정작 이 도구가 필요한 순간은 네트워크가 없을 때다.
로컬 프로그램이면 그 모순이 없다.

스레드 구조 (여기가 이 파일에서 유일하게 까다로운 부분)
------------------------------------------------------
bleak는 asyncio이고 tkinter는 자기 스레드에서만 만질 수 있다. 그래서:

    [tk 메인 스레드]  ──명령──▶  run_coroutine_threadsafe  ──▶ [asyncio 스레드]
           ▲                                                        │
           └────────  queue.Queue  ◀── BLE 콜백 ─────────────────────┘
                      (50ms마다 after로 비움)

⚠ BLE 콜백에서 위젯을 직접 건드리면 안 된다. 조용히 멎거나 이상하게 죽는다.
  전부 큐를 거친다.
"""

from __future__ import annotations

import asyncio
import json
import os
import queue
import sys
import threading
import time
import tkinter as tk
import webbrowser
from tkinter import font as tkfont

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("bleak가 없습니다.  pip install bleak", file=sys.stderr)
    raise SystemExit(1)

# --- 젯슨 서버와 맞춘 값들 (ble_console.py와 같아야 한다) ---
NUS_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # PC → 젯슨 (write)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # 젯슨 → PC (notify)
EOT = 0x04
WRITE_CHUNK = 20
DEV_NAME = os.environ.get("BLE_NAME", "tomato-jetson")

# ⚠ **스캔은 길게 잡아야 한다.** 윈도우의 BLE 스캐너는 듀티 사이클이 낮아
#   (대략 1.3초마다 수십 ms만 열어 본다) 광고를 촘촘히 쏘아도 잡히는 건 성기다.
#   2026-08-22 실측(노트북↔젯슨 rssi -75): 15초에 **2번**. 능동/수동 모드 차이도
#   없었다. 12초로 끊으면 절반쯤 놓쳐서 "젯슨이 죽었나?" 하게 된다 — 30초를 준다.
#   (폰은 이런 문제가 없다. 안드로이드는 훨씬 공격적으로 훑는다.)
SCAN_SEC = 30.0
MAX_ATTEMPTS = 3          # ⚠ 1이면 안 된다 — 첫 시도가 깨지는 일이 흔하다
RETRY_DELAY = 0.7

CONFIG = os.path.join(os.path.expanduser("~"), ".tomato_handset.json")
# 이 로봇의 현재 토큰. 저장된 값이 있으면 그게 이긴다. 토큰을 바꿨으면(젯슨에서
# /etc/tomato-ble-token 삭제 후 재시작) 화면의 [접속 토큰] 칸에 새 값을 넣으면 된다.
DEFAULT_TOKEN = os.environ.get("BLE_TOKEN", "d270dba7")


# ----------------------------------------------------------------------
# 색 — 안드로이드 앱과 같은 팔레트(코스 테이프의 자주 + 장비 표시등 호박색)
# ----------------------------------------------------------------------

LIGHT = dict(ground="#F7F4F7", panel="#FFFFFF", sunk="#F1EDF2", edge="#D6CBD9",
             ink="#241C28", soft="#5C4F62", faint="#8B7E91", plum="#6B3E70",
             signal="#B4741A", on_signal="#FFFFFF", live="#2E7D52", alert="#B33A33")
DARK = dict(ground="#17121B", panel="#1F1926", sunk="#140F18", edge="#33283A",
            ink="#EFE7F1", soft="#B4A5BB", faint="#7D6E85", plum="#A874AE",
            signal="#E8A33D", on_signal="#1A1207", live="#4FB477", alert="#E2685F")


def windows_dark_mode() -> bool:
    """윈도우가 어두운 테마인가. 못 알아내면 밝은 쪽으로 (읽기라도 되게)."""
    try:
        import winreg
        k = winreg.OpenKey(winreg.HKEY_CURRENT_USER,
                           r"Software\Microsoft\Windows\CurrentVersion\Themes\Personalize")
        return winreg.QueryValueEx(k, "AppsUseLightTheme")[0] == 0
    except Exception:  # noqa: BLE001 - 맥/리눅스거나 키가 없다
        return False


# ----------------------------------------------------------------------
# BLE — asyncio 스레드에서만 돈다
# ----------------------------------------------------------------------

class BleWorker:
    """전용 asyncio 스레드에서 BLE를 돌리고, 결과를 큐로만 내보낸다."""

    def __init__(self, out: queue.Queue) -> None:
        self.out = out
        self.loop = asyncio.new_event_loop()
        self.client: BleakClient | None = None
        self._buf = bytearray()
        self._ready = False
        threading.Thread(target=self._run_loop, daemon=True, name="ble").start()

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # --- 메인 스레드에서 부르는 것들 ---

    def connect(self) -> None:
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def disconnect(self) -> None:
        asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)

    def send(self, line: str) -> None:
        asyncio.run_coroutine_threadsafe(self._send(line), self.loop)

    # --- 내부 ---

    def _emit(self, kind: str, *args) -> None:
        self.out.put((kind, *args))

    def _on_notify(self, _sender, data: bytearray) -> None:
        """0x04까지 모았다가 한 번에 디코드. 한글이 청크 경계에서 잘리기 때문이다."""
        for b in data:
            if b == EOT:
                text = self._buf.decode("utf-8", "replace")
                self._buf.clear()
                if text.strip():
                    self._emit("text", text)
            else:
                self._buf.append(b)

    async def _find(self):
        """젯슨을 찾을 때까지 계속 훑는다. 찾는 즉시 멈춘다.

        ⚠ 이름과 서비스 UUID를 **둘 다** 본다. 어떤 백엔드는 광고의 128비트 서비스
          UUID를 스캔 결과에 안 실어준다. 이름만 보면 이름을 바꿨을 때 못 찾고,
          UUID만 보면 그 백엔드에서 아예 못 찾는다.

        ⚠ find_device_by_filter 대신 직접 도는 이유: 몇 초에 한 번밖에 안 잡히는
          상황에서 사용자가 30초간 아무 소식도 없이 기다리면 멎은 줄 안다.
          몇 초마다 남은 시간을 알려준다.
        """
        hit: dict = {}
        found = asyncio.Event()

        def cb(dev, adv) -> None:
            if hit:
                return
            name = (dev.name or adv.local_name or "").strip()
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            if name == DEV_NAME or NUS_SVC in uuids:
                hit["dev"], hit["rssi"] = dev, adv.rssi
                found.set()

        scanner = BleakScanner(detection_callback=cb)
        await scanner.start()
        try:
            deadline = time.monotonic() + SCAN_SEC
            while not found.is_set():
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                try:
                    await asyncio.wait_for(found.wait(), min(5.0, left))
                except asyncio.TimeoutError:
                    self._emit("state", "wait", "찾는 중",
                               f"…{left:.0f}초 남음 (윈도우 BLE 스캔은 성깁니다)")
        finally:
            try:
                await scanner.stop()
            except Exception:  # noqa: BLE001
                pass
        return hit.get("dev"), hit.get("rssi")

    async def _connect(self) -> None:
        if self.client is not None and self.client.is_connected:
            return
        self._emit("state", "wait", "찾는 중", f"{DEV_NAME}을 찾고 있습니다…")
        try:
            device, rssi = await self._find()
        except Exception as exc:  # noqa: BLE001
            self._emit("state", "off", "끊김", f"스캔 실패: {exc}")
            return
        if device is None:
            self._emit("state", "off", "끊김",
                       f"{SCAN_SEC:.0f}초 동안 찾지 못했습니다.\n"
                       "· 젯슨에 더 가까이 가세요(노트북은 폰보다 훨씬 둔합니다)\n"
                       "· 다른 기기가 이미 붙어 있으면 안 보입니다 — 연결 중에는"
                       " 광고가 멈춥니다(폰 앱에서 [연결 끊기])\n"
                       "· 젯슨에서: systemctl status ble-console")
            return

        name = device.name or str(device)
        self._emit("state", "wait", "연결 중",
                   f"찾았습니다: {name}" + (f" (rssi {rssi})" if rssi is not None else ""))

        # ⚠ 한 번 실패했다고 포기하지 않는다. BLE 첫 연결은 흔히 깨지고 두세 번째에
        #   붙는다(컨트롤러 슬롯 경합). 안드로이드판과 같은 이유, 같은 대책이다.
        last = None
        for attempt in range(1, MAX_ATTEMPTS + 1):
            if attempt > 1:
                self._emit("state", "wait", f"재시도 {attempt}/{MAX_ATTEMPTS}",
                           f"연결 실패({last}) — 다시 시도합니다")
                await asyncio.sleep(RETRY_DELAY)
            try:
                client = BleakClient(device, disconnected_callback=self._on_dropped)
                await client.connect()
                await client.start_notify(NUS_TX, self._on_notify)
                self.client = client
                self._ready = True
                self._emit("state", "live", "연결됨", None)
                return
            except Exception as exc:  # noqa: BLE001 - 어떤 실패든 재시도 대상
                last = type(exc).__name__
                try:
                    await client.disconnect()
                except Exception:  # noqa: BLE001
                    pass
        self._emit("state", "off", "끊김",
                   f"연결하지 못했습니다 — {last} ({MAX_ATTEMPTS}번 시도)\n"
                   "PC 블루투스를 껐다 켜보세요. 그래도 안 되면 젯슨에서:"
                   " sudo systemctl restart ble-console")

    def _on_dropped(self, _client) -> None:
        """상대가 끊었거나 범위를 벗어났다. bleak가 자기 스레드에서 부른다."""
        was_ready, self._ready = self._ready, False
        self.client = None
        self._buf.clear()
        if was_ready:
            self._emit("state", "off", "끊김",
                       "연결이 끊겼습니다. 다시 연결하면 자동으로 재인증합니다.")

    async def _disconnect(self) -> None:
        self._ready = False
        c, self.client = self.client, None
        if c is not None:
            try:
                await c.disconnect()
            except Exception:  # noqa: BLE001
                pass
        self._emit("state", "off", "끊김", None)

    async def _send(self, line: str) -> None:
        if self.client is None or not self.client.is_connected:
            self._emit("text", "연결되어 있지 않습니다")
            return
        data = (line + "\n").encode("utf-8")
        try:
            for i in range(0, len(data), WRITE_CHUNK):
                await self.client.write_gatt_char(NUS_RX, data[i:i + WRITE_CHUNK],
                                                  response=True)
        except Exception as exc:  # noqa: BLE001
            self._emit("text", f"전송 실패: {exc}")


# ----------------------------------------------------------------------
# 화면
# ----------------------------------------------------------------------

class Handset(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.C = DARK if windows_dark_mode() else LIGHT
        c = self.C
        self.title("젯슨 핸드셋")
        self.configure(bg=c["ground"])
        # ⚠ 900은 넉넉해서가 아니라 **필요해서**다. 아래쪽 WiFi·토큰 칸까지 넣으면
        #   내용이 880px쯤 된다 — 760으로 두었더니 그 둘이 창 밖으로 잘려서
        #   "WiFi 갈아타기가 없다"가 된다.
        self.geometry("720x900")
        self.minsize(560, 620)

        self.ui_font = tkfont.Font(family="Malgun Gothic", size=10)
        self.bold = tkfont.Font(family="Malgun Gothic", size=10, weight="bold")
        self.h1 = tkfont.Font(family="Malgun Gothic", size=15, weight="bold")
        self.mono = tkfont.Font(family="Consolas", size=10)
        self.ip_font = tkfont.Font(family="Consolas", size=22, weight="bold")
        self.tiny = tkfont.Font(family="Consolas", size=8)

        self.q: queue.Queue = queue.Queue()
        self.ble = BleWorker(self.q)
        self.live = False
        self.ip: str | None = None

        self._build()
        self._load_token()
        self.after(50, self._drain)
        self.protocol("WM_DELETE_WINDOW", self._quit)
        self._print("[블루투스 연결]을 누르면 tomato-jetson을 찾습니다.", "faint")

    # --- 위젯 만들기 --------------------------------------------------

    def _btn(self, parent, text, cmd, primary=False):
        c = self.C
        b = tk.Button(parent, text=text, command=cmd, font=self.bold, relief="flat",
                      bd=0, padx=14, pady=7, cursor="hand2",
                      activebackground=c["signal"], activeforeground=c["on_signal"],
                      bg=c["signal"] if primary else c["panel"],
                      fg=c["on_signal"] if primary else c["ink"],
                      highlightthickness=1, highlightbackground=c["edge"])
        return b

    def _build(self) -> None:
        c = self.C
        pad = dict(padx=14)

        # 명패
        head = tk.Frame(self, bg=c["ground"])
        head.pack(fill="x", pady=(12, 0), **pad)
        left = tk.Frame(head, bg=c["ground"])
        left.pack(side="left", anchor="w")
        tk.Label(left, text="TOMATO-PICKER · BLE LINK", font=self.tiny,
                 bg=c["ground"], fg=c["faint"]).pack(anchor="w")
        tk.Label(left, text="젯슨 핸드셋", font=self.h1,
                 bg=c["ground"], fg=c["ink"]).pack(anchor="w")
        self.lamp = tk.Label(head, text="● 끊김", font=self.mono,
                             bg=c["ground"], fg=c["faint"])
        self.lamp.pack(side="right", anchor="e")
        tk.Frame(self, bg=c["plum"], height=2).pack(fill="x", pady=(8, 12), **pad)

        # 주소 카드 — 이 앱의 존재 이유
        card = tk.Frame(self, bg=c["panel"], highlightthickness=1,
                        highlightbackground=c["edge"])
        card.pack(fill="x", **pad)
        strip = tk.Frame(card, bg=c["signal"], width=3)
        strip.pack(side="left", fill="y")
        inner = tk.Frame(card, bg=c["panel"])
        inner.pack(side="left", fill="both", expand=True, padx=12, pady=10)
        tk.Label(inner, text="젯슨 주소", font=self.tiny,
                 bg=c["panel"], fg=c["faint"]).pack(anchor="w")
        self.ip_label = tk.Label(inner, text="아직 모릅니다 — 연결한 뒤 [ip]",
                                 font=self.ui_font, bg=c["panel"], fg=c["faint"])
        self.ip_label.pack(anchor="w", pady=(2, 0))
        self.net_label = tk.Label(inner, text="", font=self.mono,
                                  bg=c["panel"], fg=c["soft"])
        self.net_label.pack(anchor="w")
        acts = tk.Frame(inner, bg=c["panel"])
        acts.pack(anchor="w", pady=(8, 0))
        self.btn_conn = self._btn(acts, "블루투스 연결", self._toggle, primary=True)
        self.btn_conn.pack(side="left")
        self.btn_copy = self._btn(acts, "주소 복사", self._copy)
        self.btn_copy.pack(side="left", padx=(8, 0))
        self.btn_dash = self._btn(acts, "대시보드 열기", self._open_dash)
        self.btn_dash.pack(side="left", padx=(8, 0))

        # 콘솔
        wrap = tk.Frame(self, bg=c["sunk"], highlightthickness=1,
                        highlightbackground=c["edge"])
        wrap.pack(fill="both", expand=True, pady=12, **pad)
        # height=14: Text의 기본 24줄은 자리를 너무 많이 요구해 아래 칸들을 밀어낸다.
        # 창을 키우면 expand로 알아서 늘어난다.
        self.console = tk.Text(wrap, bg=c["sunk"], fg=c["ink"], font=self.mono,
                               relief="flat", wrap="word", padx=10, pady=8,
                               height=14, state="disabled", insertbackground=c["ink"])
        sb = tk.Scrollbar(wrap, command=self.console.yview, relief="flat", bd=0)
        self.console.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.console.pack(side="left", fill="both", expand=True)
        for tag, col in (("cmd", c["signal"]), ("err", c["alert"]),
                         ("faint", c["faint"]), ("ink", c["ink"])):
            self.console.tag_configure(tag, foreground=col)
        self.console.tag_configure("meta", foreground=c["faint"], font=self.tiny)

        # 빠른 명령
        chips = tk.Frame(self, bg=c["ground"])
        chips.pack(fill="x", **pad)
        self.chips = []
        for cmd in ("ip", "wifi", "status", "sys", "help"):
            b = self._btn(chips, cmd, lambda x=cmd: self._send(x))
            b.configure(font=self.mono, padx=11, pady=5)
            b.pack(side="left", padx=(0, 6))
            self.chips.append(b)

        # 명령 입력
        row = tk.Frame(self, bg=c["ground"])
        row.pack(fill="x", pady=(10, 0), **pad)
        self.entry = tk.Entry(row, font=self.mono, bg=c["panel"], fg=c["ink"],
                              relief="flat", insertbackground=c["ink"],
                              highlightthickness=1, highlightbackground=c["edge"],
                              highlightcolor=c["signal"])
        self.entry.pack(side="left", fill="x", expand=True, ipady=6)
        self.entry.bind("<Return>", lambda _e: self._submit())
        self.btn_send = self._btn(row, "보내기", self._submit)
        self.btn_send.pack(side="left", padx=(8, 0))

        # WiFi 갈아타기 + 토큰
        low = tk.Frame(self, bg=c["ground"])
        low.pack(fill="x", pady=12, **pad)
        tk.Label(low, text="WiFi 갈아타기", font=self.bold,
                 bg=c["ground"], fg=c["ink"]).grid(row=0, column=0, sticky="w",
                                                   columnspan=3, pady=(0, 4))
        self.ssid = tk.Entry(low, font=self.mono, bg=c["panel"], fg=c["ink"],
                             relief="flat", insertbackground=c["ink"],
                             highlightthickness=1, highlightbackground=c["edge"])
        self.ssid.grid(row=1, column=0, sticky="ew", ipady=4)
        self.psk = tk.Entry(low, font=self.mono, bg=c["panel"], fg=c["ink"],
                            relief="flat", show="•", insertbackground=c["ink"],
                            highlightthickness=1, highlightbackground=c["edge"])
        self.psk.grid(row=1, column=1, sticky="ew", padx=6, ipady=4)
        self.btn_join = self._btn(low, "이 망으로 접속", self._join)
        self.btn_join.grid(row=1, column=2, sticky="w")
        tk.Label(low, text="SSID(공백 가능)", font=self.tiny,
                 bg=c["ground"], fg=c["faint"]).grid(row=2, column=0, sticky="w")
        tk.Label(low, text="비밀번호", font=self.tiny,
                 bg=c["ground"], fg=c["faint"]).grid(row=2, column=1, sticky="w", padx=6)

        tk.Label(low, text="접속 토큰", font=self.bold, bg=c["ground"],
                 fg=c["ink"]).grid(row=3, column=0, sticky="w", pady=(10, 4))
        self.token = tk.Entry(low, font=self.mono, bg=c["panel"], fg=c["ink"],
                              relief="flat", insertbackground=c["ink"],
                              highlightthickness=1, highlightbackground=c["edge"])
        self.token.grid(row=4, column=0, sticky="ew", ipady=4)
        tk.Label(low, text="젯슨의 /etc/tomato-ble-token · 이 PC에 저장됩니다",
                 font=self.tiny, bg=c["ground"], fg=c["faint"]) \
            .grid(row=4, column=1, columnspan=2, sticky="w", padx=6)
        low.columnconfigure(0, weight=3)
        low.columnconfigure(1, weight=2)

        self._set_live(False)

    # --- 동작 ---------------------------------------------------------

    def _print(self, text: str, tag: str = "ink") -> None:
        self.console.configure(state="normal")
        self.console.insert("end", time.strftime("%H:%M:%S") +
                            ("  보냄\n" if tag == "cmd" else "\n"), "meta")
        self.console.insert("end", text.rstrip() + "\n\n", tag)
        self.console.see("end")
        self.console.configure(state="disabled")

    def _set_live(self, on: bool) -> None:
        self.live = on
        st = "normal" if on else "disabled"
        for b in (*self.chips, self.btn_send, self.btn_join):
            b.configure(state=st)
        self.entry.configure(state=st)
        self.btn_conn.configure(text="연결 끊기" if on else "블루투스 연결")

    def _toggle(self) -> None:
        if self.live:
            self.ble.disconnect()
        else:
            self.ble.connect()

    def _send(self, line: str) -> None:
        # ⚠ 토큰은 화면에 남기지 않는다 — 어깨너머로도, 스크린샷으로도.
        shown = "auth ***" if line.lower().startswith("auth ") else line
        self._print(shown, "cmd")
        self.ble.send(line)

    def _submit(self) -> None:
        v = self.entry.get().strip()
        if v:
            self._send(v)
            self.entry.delete(0, "end")

    def _join(self) -> None:
        s, p = self.ssid.get().strip(), self.psk.get()
        if not s or not p:
            self._print("SSID와 비밀번호를 모두 넣어 주세요.", "err")
            return
        # 젯슨이 마지막 토큰을 비밀번호로 자르므로 공백 있는 SSID도 그대로 보낸다
        self._print(f"join {s} ***", "cmd")
        self.ble.send(f"join {s} {p}")

    def _copy(self) -> None:
        if not self.ip:
            return
        self.clipboard_clear()
        self.clipboard_append(self.ip)
        self._print(f"복사됨: {self.ip}", "faint")

    def _open_dash(self) -> None:
        if self.ip:
            webbrowser.open(f"http://{self.ip}:8090")

    def _harvest(self, text: str) -> None:
        """응답에서 주소를 뽑아 카드에 상주시킨다 — 스크롤백에 묻히면 의미가 없다."""
        import re
        m = re.search(r"http://(\d{1,3}(?:\.\d{1,3}){3}):8090", text)
        if m:
            self.ip = m.group(1)
            self.ip_label.configure(text=self.ip, font=self.ip_font, fg=self.C["ink"])
        w = re.search(r"^WiFi:\s*(.+)$", text, re.M)
        if w:
            self.net_label.configure(text="WiFi · " + w.group(1).strip())

    # --- 큐 비우기 (BLE 스레드 → 화면) --------------------------------

    def _drain(self) -> None:
        try:
            while True:
                kind, *rest = self.q.get_nowait()
                if kind == "state":
                    state, label, note = rest
                    dot = {"live": self.C["live"], "wait": self.C["signal"]}.get(
                        state, self.C["faint"])
                    self.lamp.configure(text=f"● {label}", fg=dot)
                    self._set_live(state == "live")
                    if note:
                        self._print(note, "err" if state == "off" else "faint")
                    if state == "live":
                        tok = self.token.get().strip()
                        if tok:
                            self._save_token(tok)
                            self._send("auth " + tok)
                        else:
                            self._print("토큰이 없습니다 — 아래 [접속 토큰]에 넣고"
                                        " 다시 연결하세요.", "err")
                elif kind == "text":
                    text = rest[0]
                    bad = any(k in text for k in ("실패", "오류", "틀렸", "없습니다"))
                    self._print(text, "err" if bad else "ink")
                    self._harvest(text)
        except queue.Empty:
            pass
        self.after(50, self._drain)

    # --- 토큰 저장 ----------------------------------------------------

    def _load_token(self) -> None:
        tok = ""
        try:
            with open(CONFIG, encoding="utf-8") as f:
                tok = (json.load(f).get("token") or "").strip()
        except (OSError, ValueError):
            pass
        self.token.insert(0, tok or DEFAULT_TOKEN)

    def _save_token(self, tok: str) -> None:
        try:
            with open(CONFIG, "w", encoding="utf-8") as f:
                json.dump({"token": tok}, f)
        except OSError:
            pass

    def _quit(self) -> None:
        try:
            self.ble.disconnect()
        finally:
            self.destroy()


if __name__ == "__main__":
    Handset().mainloop()
