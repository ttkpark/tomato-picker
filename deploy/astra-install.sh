#!/usr/bin/env bash
# Orbbec Astra Pro 드라이버 설치 (젯슨에서 한 번만).
#
#   bash deploy/astra-install.sh
#
# ⚠ **왜 스크립트가 필요한가** — 이 카메라는 apt로 안 된다.
#   우분투의 `libopenni2-0`이 들고 있는 PS1080 드라이버는 PrimeSense
#   VID(0x1d27)만 찾는다. Orbbec은 0x2bc5라 **장치가 하나도 안 잡히는데,
#   에러가 아니라 "장치 0개"로 조용히 끝난다.** 카메라가 고장난 것처럼 보인다.
#   Orbbec이 배포하는 `liborbbec.so`(+ 같이 빌드된 libOpenNI2.so)를 따로 깔아야 한다.
#
# ⚠ **ini의 기본 해상도가 640x400인데 그러면 프레임이 통째로 깨진다.**
#   매 프레임 `Depth buffer is corrupt. Size is 511856 (!= 512000)` — 144바이트가
#   모자란다. 640x480으로 바꾸면 멀쩡하다. usbfs 버퍼 문제가 아니었다
#   (usbfs_memory_mb를 16→1000으로 올려도 그대로였다). 그래서 여기서 고쳐 둔다.
#
# 근거와 실측은 docs/depth-camera.md §9.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOME_DIR="${HOME}"
ONI="${ASTRA_ONI_HOME:-$HOME_DIR/openni2}"
# ros_astra_camera가 재배포하는 arm64 OpenNI2 런타임. 소스 빌드가 필요 없다.
BASE="https://raw.githubusercontent.com/orbbec/ros_astra_camera/main/include/openni2_redist/arm64"

echo "== ① Orbbec OpenNI2 런타임 → $ONI"
mkdir -p "$ONI/OpenNI2/Drivers"
for f in libOpenNI2.so OpenNI.ini SimpleRead; do
  curl -fsSL -o "$ONI/$f" "$BASE/$f"
done
for f in liborbbec.so libOniFile.so orbbec.ini OniFile.ini; do
  curl -fsSL -o "$ONI/OpenNI2/Drivers/$f" "$BASE/OpenNI2/Drivers/$f"
done
chmod +x "$ONI/SimpleRead"

echo "== ② 깊이 기본 모드를 640x480@30으로 (640x400은 프레임이 깨진다)"
python3 - "$ONI/OpenNI2/Drivers/orbbec.ini" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p, encoding="utf-8", errors="replace").read()
# [Depth] 절 안의 Resolution/FPS만 바꾼다 — [IR]·[Image]는 건드리지 않는다.
s = re.sub(r"(\[Depth\][\s\S]*?)^Resolution=\d+", r"\g<1>Resolution=1", s, count=1, flags=re.M)
s = re.sub(r"(\[Depth\][\s\S]*?)^FPS=\d+", r"\g<1>FPS=30", s, count=1, flags=re.M)
open(p, "w", encoding="utf-8").write(s)
print("   orbbec.ini [Depth] Resolution=1(640x480) FPS=30")
PY

echo "== ③ udev 규칙 (USB 장치를 video 그룹에 연다)"
curl -fsSL https://raw.githubusercontent.com/orbbec/ros_astra_camera/main/56-orbbec-usb.rules \
  | sudo tee /etc/udev/rules.d/56-orbbec-usb.rules > /dev/null
sudo udevadm control --reload
sudo udevadm trigger

echo "== ④ 서비스"
sudo cp "$REPO/deploy/astra-cam.service" /etc/systemd/system/
sudo cp "$REPO/deploy/50-tomato-services.rules" /etc/polkit-1/rules.d/ 2>/dev/null || true
sudo systemctl daemon-reload
sudo systemctl enable --now astra-cam

echo "== ⑤ 확인"
sleep 4
systemctl is-active astra-cam || true
python3 - <<'PY'
import json, time
try:
    m = json.load(open("/dev/shm/astra_meta.json"))
except OSError:
    raise SystemExit("   ✗ /dev/shm/astra_meta.json 이 없다 — journalctl -u astra-cam -n 40")
print(f"   ✓ {m.get('model')} {m.get('serial')} {m['width']}x{m['height']} "
      f"유효 {m['valid_frac']:.0%} 중앙값 {m.get('median_mm')}mm "
      f"(유효범위 안 {m.get('near_frac', 0):.0%})")
if not m.get("near_frac"):
    print("   ⚠ 유효범위(60~250cm) 안에 아무것도 없다. 카메라가 벽에 너무 붙어 있거나"
          "\n     장면이 너무 멀다 — 60cm보다 가까운 것은 이 카메라가 **못 본다**.")
PY
