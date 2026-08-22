"""블루투스(BLE)로 젯슨에 붙어 IP를 묻고 망을 갈아타는 콘솔 서버.

왜 이게 필요한가
-----------------
이 로봇은 현장을 옮길 때마다 WiFi가 바뀌고, 그때마다 젯슨의 IP가 바뀐다. 그런데
IP를 알아내려면 SSH가 필요하고, SSH를 하려면 IP를 알아야 한다 — **닭과 달걀**이다.
지금까지는 ARP 스윕으로 매번 찾아냈지만, 새 망에 아예 못 붙은 상태(예: 폰 핫스팟이
꺼진 뒤)에서는 그 방법도 통하지 않는다. 그때 남는 건 모니터·키보드를 물리는 것뿐이다.

블루투스는 **IP 망과 완전히 무관한 경로**라 이 고리를 끊는다. WiFi가 어떻든,
젯슨이 어느 망에도 못 붙어 있든, 반경 10m 안이면 붙어서 물어볼 수 있다.
그리고 여기서 `join`으로 새 WiFi에 붙이면 된다 — **BLE 링크는 WiFi를 갈아타도
끊기지 않으므로**, 전환 결과를 그 자리에서 확인할 수 있다(SSH로 하면 자기가
탄 가지를 자르는 셈이라 매번 조마조마했다).

프로토콜 — Nordic UART Service (NUS)
------------------------------------
BLE에 "시리얼"은 없지만, NUS가 사실상의 표준이다. 두 개의 캐릭터리스틱으로
양방향 텍스트를 흘린다. 이걸 고른 이유는 **크롬의 Web Bluetooth가 그대로 붙기
때문**이다 — 안드로이드든 윈도우든 링크만 열면 되고, 앱 설치가 필요 없다.
(클래식 SPP/RFCOMM은 Web Bluetooth가 지원하지 않는다.)

    RX 6e400002-… (write)  폰 → 젯슨   명령 한 줄
    TX 6e400003-… (notify) 젯슨 → 폰   응답. 20바이트씩 쪼개 보내고 0x04로 끝냄

⚠ **20바이트 청크는 타협이 아니라 안전판이다.** BLE 기본 MTU는 23바이트(페이로드
20)이고, 협상으로 늘어나는지는 상대 스택에 달렸다. 크롬은 보통 517로 올려주지만
그걸 가정하면 안 늘려주는 조합에서 응답이 잘린 채 조용히 끝난다. 20으로 쪼개면
어디서든 맞고, wifi 스캔(~1KB)도 0.3초면 다 나간다 — 사람이 못 느낀다.

⚠ 청크는 **바이트 단위로** 자른다. 한글은 UTF-8로 3바이트라 글자 중간에서 잘리는데,
클라이언트가 0x04까지 모아서 한 번에 디코드하므로 문제되지 않는다. 청크마다
디코드하면 깨진다.

보안
----
이 서버는 WiFi를 갈아타고 서비스를 재시작한다 — 반경 안의 누구나 만질 수 있으면
안 된다. 그래서 **토큰**을 먼저 받는다(`auth <토큰>`). 토큰은 아래 파일에 있고,
없으면 처음 뜰 때 만들어 로그에 찍는다. 연결이 끊기면 인증도 같이 풀린다.

⚠ **임의 셸을 열지 않는다.** 명령은 아래 표에 있는 것만 돈다. 블루투스 너머로
`rm -rf`가 들어올 구멍을 만들 이유가 없다.
"""

from __future__ import annotations

import json
import os
import re
import secrets
import shlex
import socket
import subprocess
import sys
import time

import dbus
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

# --- NUS 표준 UUID (바꾸면 크롬 앱도 같이 바꿔야 한다) ---
NUS_SVC = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # write  (폰 → 젯슨)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # notify (젯슨 → 폰)

ADV_NAME = os.environ.get("BLE_NAME", "tomato-jetson")
TOKEN_FILE = os.environ.get("BLE_TOKEN_FILE", "/etc/tomato-ble-token")
CHUNK = int(os.environ.get("BLE_CHUNK", "20"))
EOT = b"\x04"                     # 응답 끝 표식
CHUNK_GAP_MS = int(os.environ.get("BLE_CHUNK_GAP_MS", "6"))
# 광고 간격(ms). 촘촘할수록 멀리서도 빨리 잡힌다 — 자세한 근거는 Advertisement 참고.
ADV_MIN_MS = int(os.environ.get("BLE_ADV_MIN_MS", "150"))
ADV_MAX_MS = int(os.environ.get("BLE_ADV_MAX_MS", "250"))

# 재시작을 허용할 서비스 — 여기 없는 이름은 거부한다.
ALLOWED_SERVICES = ("tomato-voice", "line-cam", "line-follow", "controller-drive")

BLUEZ = "org.bluez"
GATT_MGR = "org.bluez.GattManager1"
ADV_MGR = "org.bluez.LEAdvertisingManager1"
DBUS_PROPS = "org.freedesktop.DBus.Properties"
DBUS_OM = "org.freedesktop.DBus.ObjectManager"


def log(msg: str) -> None:
    print(f"[ble] {msg}", flush=True)


def run(cmd: list[str], timeout: float = 20.0) -> tuple[int, str]:
    """명령 하나 실행 → (종료코드, 출력). 예외를 밖으로 내지 않는다."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, f"시간 초과({timeout:.0f}s): {' '.join(cmd)}"
    except OSError as exc:
        return 127, f"실행 실패: {exc}"


def load_token() -> str:
    """토큰을 읽고, 없으면 만든다. 파일 권한은 소유자만."""
    try:
        with open(TOKEN_FILE) as f:
            tok = f.read().strip()
        if tok:
            return tok
    except OSError:
        pass
    tok = secrets.token_hex(4)          # 8자 — 손으로 칠 만하면서 추측은 어렵다
    try:
        with open(TOKEN_FILE, "w") as f:
            f.write(tok + "\n")
        os.chmod(TOKEN_FILE, 0o600)
    except OSError as exc:
        log(f"토큰 파일을 못 썼습니다({exc}) — 이번 실행에만 쓰는 토큰입니다")
    return tok


# ----------------------------------------------------------------------
# 명령들
# ----------------------------------------------------------------------

def cmd_ip() -> str:
    """이 서버의 존재 이유. 어디에 붙어 있고 대시보드 주소가 무엇인지."""
    out = []
    _rc, txt = run(["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show", "--active"])
    active_wifi = None
    for line in txt.splitlines():
        parts = line.split(":")
        if len(parts) >= 2 and parts[-1] == "802-11-wireless":
            active_wifi = ":".join(parts[:-1])
    _rc, txt = run(["ip", "-4", "-o", "addr", "show"])
    primary = None
    for line in txt.splitlines():
        m = re.search(r"^\d+:\s+(\S+)\s+inet\s+(\S+)", line)
        if not m:
            continue
        dev, cidr = m.group(1), m.group(2)
        if dev == "lo":
            continue
        # l4tbr0(192.168.55.1)·docker0은 젯슨 고유 표식이지만 접속용은 아니다
        tag = ""
        if dev.startswith("l4tbr0"):
            tag = "  (USB 디바이스모드)"
        elif dev.startswith("docker"):
            tag = "  (도커 내부)"
        elif primary is None:
            primary = cidr.split("/")[0]
            tag = "  <= 이걸로 접속"
        out.append(f"{dev:<12} {cidr}{tag}")
    if active_wifi:
        out.insert(0, f"WiFi: {active_wifi}")
    if primary:
        out.append("")
        out.append(f"대시보드  http://{primary}:8090")
        out.append(f"SSH       ssh server@{primary}")
    else:
        out.append("")
        out.append("[!] 붙어 있는 망이 없습니다 — `wifi`로 목록을 보고 `join`으로 붙이세요")
    return "\n".join(out) or "주소 없음"


def cmd_wifi() -> str:
    out = []
    _rc, txt = run(["nmcli", "device", "wifi", "rescan"], timeout=25)
    rc, txt = run(["nmcli", "-t", "-f", "ACTIVE,SIGNAL,SECURITY,SSID",
                   "device", "wifi", "list", "--rescan", "no"], timeout=25)
    if rc != 0:
        return f"스캔 실패:\n{txt}"
    seen = set()
    cur, others = [], []
    for line in txt.splitlines():
        # SSID에 ':'가 들어갈 수 있다. 앞의 세 칸은 고정이므로 3번만 자른다.
        parts = line.split(":", 3)
        if len(parts) < 4:
            continue
        active, signal, sec, ssid = parts
        if not ssid or ssid in seen:
            continue
        seen.add(ssid)
        try:
            sig = int(signal)
        except ValueError:
            sig = 0
        row = (sig, f"{sig:>3}  {ssid}  [{sec or '개방'}]")
        (cur if active == "yes" else others).append(row)
    if cur:
        out.append("[접속 중]")
        out.extend("  " + r for _s, r in cur)
    out.append("[보이는 망 · 신호 순]")
    out.extend("  " + r for _s, r in sorted(others, key=lambda t: -t[0])[:12])
    out.append("")
    out.append("붙이기:  join <SSID> <비밀번호>     (SSID에 공백이 있어도 됩니다)")
    return "\n".join(out)


def cmd_join(arg: str) -> str:
    """WiFi 접속. 마지막 토큰이 비밀번호, 그 앞이 전부 SSID.

    ⚠ SSID에 공백이 흔하다("Next door 오피스 5G"). 그래서 앞에서 자르지 않고
      **뒤에서** 자른다. 비밀번호에 공백이 있으면 따옴표로 감싸면 된다.
    """
    arg = arg.strip()
    if not arg:
        return "쓰기: join <SSID> <비밀번호>"
    try:
        parts = shlex.split(arg)
    except ValueError:
        parts = arg.split()
    if len(parts) < 2:
        return "비밀번호가 없습니다 — join <SSID> <비밀번호>"
    ssid, psk = " ".join(parts[:-1]), parts[-1]

    # 같은 SSID의 낡은 프로필은 지우고 새로 만든다(비번이 바뀌었을 수 있다)
    _rc, txt = run(["nmcli", "-t", "-f", "NAME", "connection", "show"])
    for name in txt.splitlines():
        if name == ssid:
            run(["nmcli", "connection", "delete", name])
    rc, txt = run(["nmcli", "device", "wifi", "connect", ssid, "password", psk], timeout=60)
    if rc != 0:
        return f"접속 실패:\n{txt}\n\n(비밀번호·SSID를 확인하세요. `wifi`로 목록을 다시 보세요)"
    time.sleep(3)
    return f"접속됨: {ssid}\n\n{cmd_ip()}"


def cmd_status() -> str:
    out = []
    for svc in ALLOWED_SERVICES:
        _rc, txt = run(["systemctl", "is-active", f"{svc}.service"], timeout=8)
        out.append(f"{svc:<17} {txt}")
    # 팔이 붙어 있나 — 시리얼 by-id 심볼릭 링크가 곧 답이다
    arm = "없음"
    try:
        for n in os.listdir("/dev/serial/by-id"):
            if "Single_Serial" in n:
                arm = n.split("_")[-1].replace("-if00", "")
    except OSError:
        pass
    out.append(f"{'팔(USB)':<15} {arm}")
    # 라인 검출 상태
    try:
        with open("/dev/shm/line_status") as f:
            st = json.load(f)
        out.append("{:<15} found={} dy={} 각도={} 마커={}".format(
            "라인", st.get("found"), st.get("offset_y_px"),
            st.get("angle_deg"), len(st.get("markers") or [])))
    except (OSError, ValueError):
        out.append(f"{'라인':<16} 상태 파일 없음")
    return "\n".join(out)


def cmd_restart(arg: str) -> str:
    svc = arg.strip()
    if svc not in ALLOWED_SERVICES:
        return f"허용되지 않은 서비스: {svc!r}\n허용: {', '.join(ALLOWED_SERVICES)}"
    rc, txt = run(["systemctl", "restart", f"{svc}.service"], timeout=45)
    if rc != 0:
        return f"재시작 실패:\n{txt}"
    time.sleep(2)
    _rc, txt = run(["systemctl", "is-active", f"{svc}.service"], timeout=8)
    return f"{svc} -> {txt}"


def cmd_sys() -> str:
    out = []
    _rc, txt = run(["uptime", "-p"])
    out.append(f"가동     {txt}")
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            out.append("온도     {:.1f}C".format(int(f.read().strip()) / 1000))
    except (OSError, ValueError):
        pass
    _rc, txt = run(["df", "-h", "/"])
    for line in txt.splitlines()[1:2]:
        c = line.split()
        if len(c) >= 5:
            out.append(f"디스크   {c[2]} 사용 / {c[1]} ({c[4]})")
    try:
        with open("/proc/loadavg") as f:
            out.append(f"부하     {' '.join(f.read().split()[:3])}")
    except OSError:
        pass
    out.append(f"호스트   {socket.gethostname()}")
    return "\n".join(out)


HELP = """명령
  ip                     주소와 대시보드 URL          <= 이게 주력
  wifi                   접속 중인 망 + 보이는 망 목록
  join <SSID> <비번>     그 WiFi로 갈아타기 (BLE는 안 끊깁니다)
  status                 서비스·팔·라인 검출 상태
  restart <서비스>       tomato-voice | line-cam | line-follow | controller-drive
  sys                    가동시간·온도·디스크·부하
  help                   이 도움말"""


def handle(line: str, state: dict) -> str:
    line = line.strip()
    if not line:
        return ""
    parts = line.split(None, 1)
    cmd, arg = parts[0].lower(), (parts[1] if len(parts) > 1 else "")

    if cmd == "help":
        return HELP
    if cmd == "auth":
        if secrets.compare_digest(arg.strip(), state["token"]):
            state["authed"] = True
            return f"인증됨 · {ADV_NAME}\n\n{cmd_ip()}"
        # 무차별 대입을 굼뜨게 — 사람은 못 느끼고 스크립트는 느려진다
        time.sleep(1.0)
        return "토큰이 틀렸습니다"
    if not state["authed"]:
        return "먼저 인증하세요:  auth <토큰>   (토큰은 젯슨의 /etc/tomato-ble-token)"

    if cmd == "ip":
        return cmd_ip()
    if cmd == "wifi":
        return cmd_wifi()
    if cmd == "join":
        return cmd_join(arg)
    if cmd == "status":
        return cmd_status()
    if cmd == "restart":
        return cmd_restart(arg)
    if cmd == "sys":
        return cmd_sys()
    return f"모르는 명령: {cmd!r}\n\n{HELP}"


# ----------------------------------------------------------------------
# BlueZ GATT 껍데기 (D-Bus 객체들)
# ----------------------------------------------------------------------

class Application(dbus.service.Object):
    PATH = "/com/tomatopicker/ble"

    def __init__(self, bus):
        self.services = []
        dbus.service.Object.__init__(self, bus, self.PATH)

    def get_path(self):
        return dbus.ObjectPath(self.PATH)

    def add_service(self, svc):
        self.services.append(svc)

    @dbus.service.method(DBUS_OM, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        out = {}
        for s in self.services:
            out[s.get_path()] = s.get_properties()
            for c in s.chars:
                out[c.get_path()] = c.get_properties()
        return out


class Service(dbus.service.Object):
    def __init__(self, bus, index, uuid):
        self.path = f"{Application.PATH}/service{index}"
        self.uuid = uuid
        self.chars = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {"org.bluez.GattService1": {
            "UUID": self.uuid,
            "Primary": dbus.Boolean(True),
            "Characteristics": dbus.Array(
                [c.get_path() for c in self.chars], signature="o"),
        }}

    @dbus.service.method(DBUS_PROPS, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != "org.bluez.GattService1":
            raise dbus.exceptions.DBusException("org.bluez.Error.InvalidArguments")
        return self.get_properties()["org.bluez.GattService1"]


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = f"{service.path}/char{index}"
        self.uuid, self.flags, self.service = uuid, flags, service
        self.notifying = False
        self.value = []
        service.chars.append(self)
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {"org.bluez.GattCharacteristic1": {
            "Service": self.service.get_path(),
            "UUID": self.uuid,
            "Flags": dbus.Array(self.flags, signature="s"),
        }}

    @dbus.service.method(DBUS_PROPS, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != "org.bluez.GattCharacteristic1":
            raise dbus.exceptions.DBusException("org.bluez.Error.InvalidArguments")
        return self.get_properties()["org.bluez.GattCharacteristic1"]

    @dbus.service.signal(DBUS_PROPS, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return self.value

    @dbus.service.method("org.bluez.GattCharacteristic1", in_signature="aya{sv}")
    def WriteValue(self, value, options):
        pass

    @dbus.service.method("org.bluez.GattCharacteristic1")
    def StartNotify(self):
        self.notifying = True

    @dbus.service.method("org.bluez.GattCharacteristic1")
    def StopNotify(self):
        self.notifying = False


class TxCharacteristic(Characteristic):
    """젯슨 → 폰. 20바이트씩 끊어 흘리고 마지막에 0x04."""

    def __init__(self, bus, index, service):
        super().__init__(bus, index, NUS_TX, ["notify"], service)
        self._queue = []
        self._pumping = False

    def send(self, text: str) -> None:
        data = text.encode("utf-8") + EOT
        self._queue.extend(data[i:i + CHUNK] for i in range(0, len(data), CHUNK))
        if not self._pumping:
            self._pumping = True
            GLib.timeout_add(CHUNK_GAP_MS, self._pump)

    def _pump(self) -> bool:
        if not self._queue or not self.notifying:
            self._pumping = False
            self._queue.clear()
            return False
        chunk = self._queue.pop(0)
        self.PropertiesChanged(
            "org.bluez.GattCharacteristic1",
            {"Value": dbus.Array([dbus.Byte(b) for b in chunk], signature="y")}, [])
        if not self._queue:
            self._pumping = False
            return False
        return True


class RxCharacteristic(Characteristic):
    """폰 → 젯슨. 줄 단위로 모아 명령으로 넘긴다."""

    def __init__(self, bus, index, service, tx, state):
        super().__init__(bus, index, NUS_RX,
                         ["write", "write-without-response"], service)
        self._tx, self._state, self._buf = tx, state, b""

    def WriteValue(self, value, options):
        self._buf += bytes(bytearray(value))
        while b"\n" in self._buf:
            raw, self._buf = self._buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            # 토큰은 로그에 남기지 않는다
            shown = "auth ***" if line.lower().startswith("auth") else line
            log(f"<- {shown}")
            try:
                reply = handle(line, self._state)
            except Exception as exc:  # noqa: BLE001 - 어떤 명령이 터져도 서버는 산다
                reply = f"오류: {exc}"
                log(f"명령 처리 중 예외: {exc}")
            if reply:
                self._tx.send(reply)


class Advertisement(dbus.service.Object):
    PATH = "/com/tomatopicker/adv0"

    def get_path(self):
        return dbus.ObjectPath(self.PATH)

    # 광고 간격(ms). ⚠ 기본값(BlueZ ~1.28초)은 **멀리 있는 상대에게 너무 성기다** —
    #   2026-08-22 실측: 노트북(rssi -83)에서 20초 스캔에 단 2번 잡혔고, 12초짜리
    #   탐색은 절반쯤 놓쳤다. 촘촘히 쏘면 같은 거리에서도 확률이 올라간다.
    #   대가는 전력인데, 이 로봇은 벽 전원이거나 큰 배터리를 지고 다닌다.
    #   ⚠ 이 속성은 BlueZ 5.56+에서만 있다. 없으면 등록이 통째로 실패하므로
    #   main()에서 실패 시 이 둘을 빼고 다시 등록한다(intervals=False).
    def __init__(self, bus, intervals: bool = True):
        self.intervals = intervals
        dbus.service.Object.__init__(self, bus, self.PATH)

    def get_properties(self):
        props = {
            "Type": "peripheral",
            "ServiceUUIDs": dbus.Array([NUS_SVC], signature="s"),
            "LocalName": dbus.String(ADV_NAME),
            "Includes": dbus.Array(["tx-power"], signature="s"),
        }
        if self.intervals:
            props["MinInterval"] = dbus.UInt32(ADV_MIN_MS)
            props["MaxInterval"] = dbus.UInt32(ADV_MAX_MS)
        return {"org.bluez.LEAdvertisement1": props}

    @dbus.service.method(DBUS_PROPS, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != "org.bluez.LEAdvertisement1":
            raise dbus.exceptions.DBusException("org.bluez.Error.InvalidArguments")
        return self.get_properties()["org.bluez.LEAdvertisement1"]

    @dbus.service.method("org.bluez.LEAdvertisement1")
    def Release(self):
        log("광고 해제됨")


def find_adapter(bus) -> str | None:
    om = dbus.Interface(bus.get_object(BLUEZ, "/"), DBUS_OM)
    for path, ifaces in om.GetManagedObjects().items():
        if GATT_MGR in ifaces and ADV_MGR in ifaces:
            return path
    return None


def main() -> None:
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter = find_adapter(bus)
    if adapter is None:
        log("BLE 어댑터를 찾지 못했습니다 (bluetooth.service가 떠 있나요?)")
        sys.exit(1)

    props = dbus.Interface(bus.get_object(BLUEZ, adapter), DBUS_PROPS)
    props.Set("org.bluez.Adapter1", "Powered", dbus.Boolean(True))
    props.Set("org.bluez.Adapter1", "Alias", dbus.String(ADV_NAME))

    state = {"token": load_token(), "authed": False}

    app = Application(bus)
    svc = Service(bus, 0, NUS_SVC)
    tx = TxCharacteristic(bus, 0, svc)
    RxCharacteristic(bus, 1, svc, tx, state)
    app.add_service(svc)

    gatt = dbus.Interface(bus.get_object(BLUEZ, adapter), GATT_MGR)
    adv_mgr = dbus.Interface(bus.get_object(BLUEZ, adapter), ADV_MGR)
    adv = Advertisement(bus)

    def die(err):
        log(f"등록 실패: {err}")
        sys.exit(1)

    def on_first_error(err):
        """첫 등록이 실패하면 **간격 속성 탓인지부터 의심한다.**

        MinInterval/MaxInterval은 BlueZ 5.56+에만 있다. 오래된 BlueZ에서는 이것
        하나 때문에 등록 전체가 거절되는데, 그러면 광고가 아예 안 나가 서비스가
        무용지물이 된다. 간격은 있으면 좋은 것이지 필수가 아니므로 빼고 다시 건다.
        """
        if adv.intervals:
            log(f"광고 등록 실패({err}) — 간격 속성을 빼고 다시 시도합니다")
            adv.intervals = False
            start_advertising(first=True)
            return
        die(err)

    def start_advertising(first: bool = False) -> bool:
        adv_mgr.RegisterAdvertisement(
            adv.get_path(), {},
            reply_handler=lambda: log(
                f"광고 시작: '{ADV_NAME}'"
                + (f" (간격 {ADV_MIN_MS}~{ADV_MAX_MS}ms)" if adv.intervals else "")),
            error_handler=(on_first_error if first
                           else lambda e: log(f"광고 재개 실패: {e}")))
        return False        # GLib.timeout_add용 — 한 번만 돈다

    def readvertise() -> bool:
        """⚠ **이게 이 파일에서 제일 중요한 다섯 줄이다.**

        BLE 규격상 **연결 가능 광고는 연결이 성립되는 순간 컨트롤러가 끈다.** 그건
        정상이다. 문제는 **BlueZ가 끊긴 뒤에도 다시 켜주지 않는다**는 것 —
        그래서 손님이 한 번 다녀가면 젯슨은 **영영 안 보이는 상태가 된다.**

        2026-08-22 실측으로 확정: 폰이 붙었다 끊긴 뒤 PC에서 12초 전체 스캔을 해도
        47개 기기 중 젯슨만 없었다. 서비스를 재시작하니 즉시 잡혔다(rssi -84).
        그날 "연결이 안 된다"던 증상이 전부 이것이었다 — 폰의 첫 시도가 링크 계층
        에서 붙어 광고를 끄고 GATT에서 깨졌고, 그 뒤로는 찾을 수조차 없었다.
        재시작만이 살렸고, 그래서 원인이 컨트롤러 문제처럼 보였다.
        """
        try:
            adv_mgr.UnregisterAdvertisement(adv.get_path())
        except Exception as exc:  # noqa: BLE001 - 이미 풀려 있으면 그게 정상이다
            log(f"(광고 해제 생략: {exc})")
        return start_advertising()

    def on_conn_change(iface, changed, _invalidated, path=None):
        """연결이 끊기면 인증을 풀고 **광고를 되살린다**."""
        if iface != "org.bluez.Device1" or "Connected" not in changed:
            return
        if bool(changed["Connected"]):
            log(f"연결됨: {path}")
        else:
            log(f"끊김: {path} — 인증 해제, 광고 재개")
            state["authed"] = False
            tx.notifying = False
            # 끊김 직후 바로 걸면 BlueZ가 아직 정리 중이라 거절한다. 한 박자 쉰다.
            GLib.timeout_add(500, readvertise)

    bus.add_signal_receiver(on_conn_change, dbus_interface=DBUS_PROPS,
                            signal_name="PropertiesChanged", path_keyword="path")

    gatt.RegisterApplication(app.get_path(), {},
                             reply_handler=lambda: log("GATT 등록됨"),
                             error_handler=die)
    start_advertising(first=True)

    log(f"토큰: {state['token']}   (파일: {TOKEN_FILE})")
    log(f"크롬에서 웹앱을 열고 이 이름을 고르세요: {ADV_NAME}")
    try:
        GLib.MainLoop().run()
    except KeyboardInterrupt:
        log("종료")


if __name__ == "__main__":
    main()
