# CLAUDE.md — 에이전트 오리엔테이션

토마토 따기 로봇(LG Physical AI 데모). "언어 → 인식 → 행동". 상세 배경은
[README.md](README.md) · [PROJECT_토마토로봇.md](PROJECT_토마토로봇.md) · [스마트로봇_명세서.md](스마트로봇_명세서.md).

## 소프트웨어 진입점
- `main.py` (demo/run) → `src/tomato_picker/` (skills.py=스킬5종, orchestrator.py=Claude tool-use,
  demo_mode.py=오프라인 폴백, hardware/=Base·Arm·Camera 추상화, vision/=색검출).

## 하드웨어 제어 — **현재 구성 (중요: 옛 문서와 혼동 주의)**
로봇 = **메카넘 베이스(Moebius Uno + PCA9685)** + **SO-101 팔로워(집게)** + 젯슨 Orin Nano.

- **주행 펌웨어 = [`firmware/mecanum_stable/`](firmware/mecanum_stable/) v2** ✅ (아두이노/CH340, `/dev/ttyUSB0`).
  젯슨이 **시리얼 라인 프로토콜**로 구동: `V vx vy w`(속도, -255..255) / `S`(정지) / `T i d`(모터캘리)
  / `P n`(PWM) / `L a b c d`(극성) / `R accel decel`(슬루) / `F hz`(PWM주파수). baud 115200.
  **v2 = 소음·끊김 대책**: PWM 50Hz→1500Hz(“우우웅”의 원인), 펌웨어 슬루(가감속),
  소프트 데드맨(300ms 감속/1000ms 하드), **XOR 체크섬 `*HH`**(한 번 받으면 무체크섬 `V` 거부),
  `V` 무응답 + 1초 하트비트에 링크 카운터. 젯슨 쪽은
  [`hardware/motor_link.py`](src/tomato_picker/hardware/motor_link.py)가 **상시 연결 + 20ms 재전송
  스레드 + 자동 재연결**로 물린다. ⚠ 포트는 **한 프로세스만**(`exclusive=True`) —
  `tomato-voice`와 `controller-drive`를 동시에 켜지 말 것.
  ⚠ **소음↔속도는 맞바꾸는 관계**(PCA9685 상한 1526Hz라 초음파 불가). 대시보드
  "모터 튜닝" 패널에서 `F`(주파수)/`P`(듀티상한)/`R`(슬루)을 재플래시 없이 바꿔 찾는다.
  현재 기본 700Hz·듀티 2600. ⚠ 포트를 열면 커널 DTR 토글로 **Uno가 리셋**되므로
  설정은 **첫 하트비트 이후**에 넣어야 먹는다(부트로더 ~2초가 다 먹어버림).
  ⚠️ **`firmware/PS2-MOTOR/`는 레거시**(PS2 직접조종, 불안정). `mecanum_serial/`은 폐기. 새 작업은 `mecanum_stable` 기준.
  모터↔바퀴 실측: A=FR B=FL C=RR D=RL, `POL={-1,1,-1,1}`.
- **게임패드 주행 + 팔 프리셋 = [`tools/controller_drive.py`](tools/controller_drive.py)** (젯슨, lerobot venv).
  Switch 프로콘/Xbox360 게임패드(evdev)를 읽어 → 메카넘 `V` 지령 + 팔 프리셋 재생.
  **systemd `controller-drive` 서비스로 부팅 자동실행**([`deploy/`](deploy/)). 조작: **LT(ZL) 데드맨=누르는 동안만 주행**,
  왼스틱=이동/오른스틱=회전/D패드=디지털, **LB/RB=팔 프리셋(이전/다음)**. 셋업·이슈=[`docs/jetson-gamepad-setup.md`](docs/jetson-gamepad-setup.md).
- **팔 제어 = 대시보드 `/control`** ✅ 권장 — 리더암 연결·미러링·프리셋 **슬롯 0~19** 저장/재생·
  **높이 앵커 보간**을 브라우저에서 전부 한다([`docs/dashboard-presets.md`](docs/dashboard-presets.md)).
  슬롯 수는 `presets.py`의 `SLOT_COUNT` 하나로 정한다(2026-08-17: 10→20).
  구현: [`hardware/presets.py`](src/tomato_picker/hardware/presets.py)(저장소·보간) +
  [`hardware/arm.py`](src/tomato_picker/hardware/arm.py)(미러링·재생).
  프리셋=`~/arm_presets.json` — 옛 형식 호환, 이름·앵커는 `"__meta__"` 키에.
  대안: [`tools/mirror_toggle.py`](tools/mirror_toggle.py)(터미널·리더 필요, 레거시) 또는
  `controller_drive.py`의 프리셋 재생(리더 없이 팔로워만).
- **hid-nintendo**: 이 젯슨 커널엔 없어서 out-of-tree 빌드해 설치함(게임패드 필수). 빌드법=`docs/jetson-gamepad-setup.md`.

## 젯슨 접속
- `ssh server@192.168.0.8` (IP는 DHCP라 바뀔 수 있음 / 공개키 등록·passwordless sudo).
  Bash 툴은 `-i /c/Users/parkg/.ssh/id_ed25519` 명시 필요(HOME 다름).
- 장치: `/dev/ttyUSB0`=모터보드, `/dev/ttyACM0`=팔로버, 게임패드=USB/BT.
- **IP를 모를 때는 블루투스로 물어본다** ([`docs/bluetooth-console.md`](docs/bluetooth-console.md), 2026-08-22 추가).
  `ble-console.service`가 BLE로 `tomato-jetson`을 광고하고, **안드로이드 앱**
  ([`android/handset/`](android/handset/), `./build.sh install`)에서 `ip`·`wifi`·`join`·`status`를 친다.
  ⚠ **웹(Web Bluetooth)으로는 안 된다** — 샌드박스 iframe에 `bluetooth` 권한이 위임되지
  않아 `requestDevice`가 권한정책에 막힌다(실측). 그래서 화면은 WebView로 재사용하고
  블루투스만 네이티브로 내린 APK를 쓴다. Gradle 없이 build-tools로 몇 초면 빌드된다.
  ⚠ IP를 알려면 SSH가, SSH를 하려면 IP가 필요한 **닭과 달걀**을 끊는 유일한 경로다 —
  ARP 스윕은 젯슨이 *이미 내가 아는 망에* 붙어 있을 때만 통한다.
  **`join`으로 망을 갈아타도 BLE는 안 끊긴다**(SSH로 하면 자기가 탄 가지를 자른다).
  반경 10m 한계 / 토큰은 젯슨의 `/etc/tomato-ble-token`.

## 바닥 카메라 · 라인 주행 (2026-08-09 추가)
- **바닥 CSI 카메라 = Arducam RPi Cam v2.1(IMX219), CAM1**, 전면 우측 바퀴 앞. jetson-io로
  "Camera IMX219 Dual" 오버레이 적용됨(L4T R39.2.0에 드라이버 내장).
- **테이프는 진행방향과 나란한 가로 띠** — 흔한 "검은 세로선 조향"이 **아니다**.
  띠의 세로위치=거리→`vy`(게걸음), 기울기=요→`w`, 화면 가로=진행→`vx`.
- 파이프라인: [`deploy/line-cam.service`](deploy/line-cam.service)(gst→`/dev/shm/line_cam.jpg`) →
  [`tools/line_follow.py`](tools/line_follow.py)(검출→`line_view.jpg`+`line_status`) →
  [`hardware/line_drive.py`](src/tomato_picker/hardware/line_drive.py)(주행) → 대시보드.
- **임계는 전부 프레임 상대값**(절대값은 색 캐스트에서 무너진다). 색 마커 구분은 기본
  꺼짐(`LF_COLOR=1`로 켬 — 배경색 통제 후).
- 화면: `/control`(운전) · `/settings`(영점·펄스·축부호·모터튜닝·팔 캘리브레이션).
- **주행은 펄스(깔짝깔짝)** — 연속 저속은 정지마찰에서 미끄러진다. `ON==주기`면 연속.

## 알려진 함정
- ⚠ **"지령은 나가는데 아무 일도 안 일어난다"가 이 로봇의 1번 병이다.** 정지마찰
  문턱 아래 크기는 물리적으로 0이다. 2026-08-17 실측(제자리, 펄스 0.12s):
  **회전** w=75→0.00° · 100→0.20° · 120→0.75° (문턱 ≈90) /
  **진행** 60→2.2px · 90→9.3px · 130→96px · 150→230px (문턱 ≈90).
  하루에 세 번 같은 병이었다 — 회전 보정(60~75), 지점 정렬 진행축(60),
  정렬 흔들기(140은 반대로 **너무 커서** 100px씩 밀어냄). 증상은 제각각이었지만
  ("부호를 뒤집어도 똑같다", "마커가 굳었다며 흔들다 발산", "맞춰 놓고 도로 밀림")
  원인은 하나다. **게인을 만지기 전에 그 크기가 실제로 움직이는지부터 재라.**
  ⚠ 축마다 문턱이 다르니 **한 슬라이더로 두 축을 자르지 말 것**(corr_max가 회전까지
  잘라 8/10에 고친 병이 8/13에 재발했다). 회전은 1도마다 진행축을 **17px 밀어낸다**
  (회전 중심이 카메라에서 벗어나 있다) — 그래서 정렬은 **각도 → x → 거리** 순서다.
- ⚠ **무대 학습(`harvest.py` SceneModel)은 `~/harvest_scene.json`에 남는다.** 2026-08-17:
  메모리에만 두었더니 **서비스 재시작이 학습을 지웠고**, 다시 배우는 첫 프레임이
  오검출이면 그게 무대가 됐다(x=1258 ← 1280 화면 끝) → 가운데·오른쪽 열매가
  왼쪽·가운데로 찍혔다. 지금은 상식 검사 + 확인 프레임 5회 + 불신 자동 탈출로 막는다.
  무대를 옮겼으면 **세 그루에 열매를 올린 뒤** [무대 다시 배우기](2그루면 가드가 막는다).
- ⚠ **상세 교훈은 [`docs/개발노트-2026-08-09.md`](docs/개발노트-2026-08-09.md)** —
  증상↔원인이 어긋났던 사례(소음=PWM주파수, 라인분실=색캐스트, yaw발산=각추정,
  리더암 오구동=포트식별)와 진단 순서 치트시트.
- **USB 접촉/전원 불안정**: 게임패드 `error -71`(케이블 손상), CH340 반복 disconnect. `controller_drive.py`는
  모터 시리얼 **자동재연결** 내장. 근본은 양품 케이블 + 안정 전원(전류제한 넉넉히).
- **PS2X는 이 보드에서 `millis()`를 얼림**(timer0 간섭) → 주행 펌웨어에서 PS2 완전 배제, 안전은 HW워치독.
- 게임패드 스틱 드리프트/latch 폭주 방지 위해 **데드맨(LT 홀드) 필수 설계**.

## Git
- origin: `github.com/ttkpark/tomato-picker`. 기본 브랜치 `master`.
