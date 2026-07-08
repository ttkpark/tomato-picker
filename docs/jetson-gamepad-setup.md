# 젯슨 게임패드 주행 + 팔 프리셋 통합 (설정 가이드)

토마토피커: **Switch 프로콘 호환 게임패드(조이트론 Switch Duocon)** 를 젯슨이 evdev로 읽어
- 메카넘 베이스 주행([`firmware/mecanum_stable`](../firmware/mecanum_stable/) 펌웨어에 `V` 지령)
- SO-101 팔로워 **프리셋 재생**(리더 없이, 저장 자세로 보간)

을 하나의 드라이버 [`tools/controller_drive.py`](../tools/controller_drive.py)에서 처리하고,
**systemd로 부팅 시 자동 실행**한다.

대상: Jetson Orin Nano, JetPack/L4T 커널 `6.8.12-1021-tegra`, 계정 `server`.

---

## 1. 하드웨어 연결
| 장치 | 포트 | 비고 |
|---|---|---|
| 메카넘 모터보드(Arduino/CH340) | `/dev/ttyUSB0` (`1a86:7523`) | `mecanum_stable` 펌웨어 업로드 |
| SO-101 팔로워(집게) | `/dev/ttyACM0` (`1a86:55d3`) | 리더 없음 |
| 게임패드 | USB (권장) 또는 BT | 057e:2009 Pro Controller로 잡히는 모드 |

## 2. hid-nintendo 커널모듈 빌드 (이 커널엔 기본 미포함)
Switch/Pro 컨트롤러는 호스트의 **입력리포트 핸드셰이크**가 있어야 데이터를 흘린다.
그걸 하는 `hid-nintendo`가 이 tegra 커널엔 없어서 out-of-tree 빌드가 필요하다(ch341과 동일 패턴).
```bash
mkdir -p ~/hidnin && cd ~/hidnin
curl -sSL -o hid-nintendo.c https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/hid/hid-nintendo.c
curl -sSL -o hid-ids.h      https://raw.githubusercontent.com/torvalds/linux/v6.8/drivers/hid/hid-ids.h   # .c가 로컬 include
printf 'obj-m := hid-nintendo.o\n' > Makefile
make -C /lib/modules/$(uname -r)/build M=$PWD modules          # → hid-nintendo.ko (gcc 버전 경고 무해)
sudo cp hid-nintendo.ko /lib/modules/$(uname -r)/kernel/drivers/hid/
sudo depmod -a
echo hid-nintendo | sudo tee /etc/modules-load.d/hid-nintendo.conf   # 부팅 자동로드
sudo modprobe hid-nintendo
```
⚠️ **커널 업데이트 시 재빌드**(DKMS 미설정). 테이블에 `HID_BLUETOOTH_DEVICE(...PROCON)` 있어 USB/BT 프로콘 모두 바인딩.
단 컨트롤러가 **클론ID(1949:0402)** 로 붙는 모드면 매칭 안 됨 → **프로콘(057e:2009) 모드**로 쓸 것.

## 3. 권한
```bash
sudo usermod -aG input server     # /dev/input/event* 는 root:input 660 → server를 input 그룹에
# (새 로그인 세션부터 적용. 안 하면 evdev list_devices()가 빈 리스트)
```
서비스 유닛에도 `SupplementaryGroups=input dialout` 포함됨.

## 4. venv 의존성
드라이버는 lerobot venv에서 실행(팔 제어 위해). evdev 추가:
```bash
~/lerobot/.venv/bin/pip install evdev     # pyserial·lerobot은 이미 있음
```

## 5. 펌웨어 업로드
```bash
cd ~/tomato-picker   # (repo)
arduino-cli compile --fqbn arduino:avr:uno firmware/mecanum_stable
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno firmware/mecanum_stable
```

## 6. systemd 서비스 (부팅 자동 실행)
```bash
sudo cp deploy/controller-drive.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now controller-drive
# 관리: sudo systemctl {start|stop|restart} controller-drive ; journalctl -u controller-drive -f
```
장치가 아직 없으면 스크립트가 종료→3초 뒤 자동 재시도(`Restart=always`), 붙으면 자동 동작.

---

## 조작 (controller_drive.py)
- **데드맨**: **LT(ZL) 누르는 동안만** 주행. 놓으면 즉시 정지 → 스틱 드리프트/latch 폭주 불가.
- LT 누른 채: 왼스틱=전진/평행, 오른스틱 X=회전, D패드=디지털 이동, RT(ZR)=부스트, B=즉시정지.
- **팔 프리셋**: **LB=이전, RB=다음** 프리셋 재생(팔로워만, limp로 시작). 프리셋은 `~/arm_presets.json`.
- Switch(hid-nintendo, 디지털 ZL/ZR=BTN_TL2/TR2) / Xbox360(xpad, 아날로그 LT/RT=ABS_Z/ABS_RZ) **양쪽 지원**.
- 시작 시 스틱 중립값 자동보정(손 떼고 실행), 데드존 7000.
- **모터 시리얼 자동재연결**: CH340이 끊겼다 재인식돼도 쓰기 실패 감지→재오픈으로 스스로 복구.

## ⚠️ 알려진 이슈 (USB 안정성)
- **CH340(모터보드) 반복 disconnect/재인식** → 낡은 시리얼 핸들로 주행만 멈추던 문제.
  드라이버 자동재연결로 완화. 근본은 **양품 데이터케이블 + 안정 전원(BAT/굵은선)**.
- **게임패드 USB 접촉 불량**(`dmesg: error -71, unable to enumerate`, 포트 1-2.1): 케이블·커넥터
  손상/헐거움. 증상 시 **데이터 케이블 교체 + 다른 USB 포트**, 또는 **블루투스로 전환**(케이블 문제 회피).
- 전원: 모터 구동 시 전류 급증 — 벤치 파워 **전류 제한을 넉넉히(≥5~10A)**. 낮으면 CC진입→전압 새그.

## 진단 도구
- [`tools/jetson_pad_probe.py`](../tools/jetson_pad_probe.py): 게임패드 축/버튼 실시간 매핑 확인(evdev).
- [`tools/serial_bridge.py`](../tools/serial_bridge.py): 시리얼을 로그파일로 실시간 미러 + 명령파일 주입(개발 중 시리얼 훔쳐보기).
- [`firmware/diagnostics/motor_test/`](../firmware/diagnostics/motor_test/): PS2 없이 시리얼로 모터 직접 구동(모터경로 격리 테스트).
