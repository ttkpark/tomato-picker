# CLAUDE.md — 에이전트 오리엔테이션

Be extremely brief. Minimize output tokens: no explanations, no summaries,
no restating the task. Act, then stop. Never trade correctness or
completeness for brevity: full input validation, every edge case, and all
behavior the task requires stay in.

토마토 따기 로봇(LG Physical AI 데모). "언어 → 인식 → 행동". 상세 배경은
[README.md](README.md) · [PROJECT_토마토로봇.md](PROJECT_토마토로봇.md) · [스마트로봇_명세서.md](스마트로봇_명세서.md).

## 소프트웨어 진입점
- `main.py` (demo/run) → `src/tomato_picker/` (skills.py=스킬5종, orchestrator.py=Claude tool-use,
  demo_mode.py=오프라인 폴백, hardware/=Base·Arm·Camera 추상화, vision/=색검출).

## ⚠ 계통이 둘이다 — 어느 쪽을 고치는지 먼저 정하라 (2026-08-28~)
- **기존 스택** = `src/tomato_picker/`. **지금 데모에서 도는 것.** 태그 `v1.*`, 브랜치 `master`.
- **ROS 2 계통** = [`ros2/`](ros2/). 만드는 중. 태그 `v2.0.0-ros.N`, 브랜치 `feat/ros2-*`.
  픽셀과 무대 학습을 버리고 **D405 깊이 + 손-눈 보정 + TF**로 좌표를 만든다 —
  무대를 옮기거나 카메라를 다시 달아도 안 무너지게 하려는 것.
  왜·무엇을 = [`ros2/README.md`](ros2/README.md) · 단계와 태그 규칙 =
  [`docs/ros2-이행계획.md`](docs/ros2-이행계획.md) · 젯슨에서 띄우기 =
  [`ros2/docker/README.md`](ros2/docker/README.md).
  **경계 한 줄: 계산과 실측값은 공유하고, 장치·상태의 소유권은 ROS가 가지며,
  웹 서비스에는 붙지 않는다.** 기구학을 다시 짜면 같은 팔을 두 계통이 다르게 믿는다 —
  그래서 `kinematics`·`handeye`·`cartesian`·`eye.EyeConfig`는 계속 쓴다(라이브러리다).
  반대로 팔을 여는 일은 ROS가 가져왔다(`tomato_bridge/follower_io.py`).
  졸업표 = [`docs/ros2-이행계획.md`](docs/ros2-이행계획.md) §4, **강제** = `ros_selfcheck`의 [경계] 검사.
  ⚠ **둘을 동시에 못 켠다** — 팔(`tomato-voice`)·주행(`controller-drive`)·**D405(`depth-cam`)**
  넷(**Astra `astra-cam`** 포함) 다 한 프로세스만 연다. ROS를 띄우기 전에 끈다. `arm_mode:=proxy`는 배선 확인용
  임시 경로일 뿐이다(HTTP 폴링이라 TF 시각 정렬이 안 된다, ros.3에서 삭제).
  ⚠ **보정 파일은 한 벌** — `~/arm_eye.json`을 대시보드와 ROS가 같이 쓴다
  (카메라별로 칸이 나뉘고, **최상위가 D405**다 — ROS가 읽는 자리라 안 옮겼다).
  ⚠ 태그는 `tools/tag.sh`로 단다(계통을 섞어 달면 거절한다).
- **깊이 카메라도 둘이다** (2026-09-01~) —
  **D405 = 근거리 7~50cm**(집는 순간) · **Astra Pro = 원거리 60~400cm**(무대 전체).
  더 좋은 쪽이 있는 게 아니라 **쓰는 자리가 다르다** — D405는 삼각대 거리(1~2m)를
  못 보고, Astra는 60cm 아래를 못 본다. 발행기가 각각 하나씩
  ([`tools/depth_cam.py`](tools/depth_cam.py) `depth-cam` /
  [`tools/astra_cam.py`](tools/astra_cam.py) `astra-cam`)이고, 읽는 쪽은
  `DepthView(camera=...)` 이름만 다르다. 자세히 = [`docs/depth-camera.md`](docs/depth-camera.md).
  ⚠ **깊이 단위·유효 거리·정렬 여부를 읽는 코드에 박지 않는다** — 전부 meta.json이
  말한다(D405 0.1mm / Astra 1mm를 상수로 박으면 거리가 10배가 된다).
  ⚠ **Astra는 컬러가 깊이와 정렬돼 있지 않다**(RGB가 별개 USB, 공장 D2C값이 NaN).
  컬러 클릭은 **거절**하고 깊이 화면에서 클릭한다. 🍅 자동 열매 검출도 Astra에선 막힌다.
  ⚠ **보정은 카메라마다 따로** 저장된다(`~/arm_eye.json` 최상위=D405, `cameras.astra`=Astra).
  ⚠ **Astra 드라이버는 apt로 안 깔린다** — 우분투 OpenNI2는 Orbbec VID를 모르고
  "장치 0개"로 조용히 끝난다. `bash deploy/astra-install.sh`.
  ⚠ **Astra 깊이가 0%면 발행기를 다시 띄워라** — 60cm보다 가까운 것을 보면 LDP가
  투광기를 끄는데 **스트림 수명 내내 꺼진 채 유지된다**(카메라를 옮겨도 안 풀린다).
  발행기가 30초 무효 시 스스로 재시작하게 해 뒀다(`ASTRA_ZERO_REOPEN_SEC`).
  ⚠ Astra는 ROS 2 계통엔 **아직 안 붙였다**(realsense2_camera만 있다).
- **줄기 잡기 조작대 = [`ros2/tools/click_server.py`](ros2/tools/click_server.py)** (2026-09-03~)
  — 젯슨이 직접 띄우는 웹 조작대 + API. **http://<젯슨IP>:8090/** (systemd `click-server`).
  화면을 눌러 **어느 줄기인지 사람이 정하고**(`~/click_target.json`), 자세·조그·집게·
  잡기를 전부 거기서 시킨다. **버튼과 `curl`과 에이전트가 같은 경로**를 쓴다 —
  터미널에서만 되는 조작을 남기지 않기로 했다.
  ⚠ **한 번에 하나만 돈다**(팔 포트는 한 프로세스). 도는 중 새 일은 409로 거절한다.
  ⚠ 표적 클릭은 **그 프레임의 화소 자리**다 — 팔이 움직였으면 다시 찍어야 한다.
  잡기 본체 = [`ros2/tools/stem_grasp.py`](ros2/tools/stem_grasp.py),
  손끝 조그 = [`ros2/tools/tool_jog.py`](ros2/tools/tool_jog.py).
  **인수인계·함정 목록 = [`docs/인수인계-2026-09-03.md`](docs/인수인계-2026-09-03.md)** — 만지기 전에 읽어라.
- **손-눈 보정 수학 = [`hardware/handeye.py`](src/tomato_picker/hardware/handeye.py)**
  — 카메라가 본 3D 점을 팔 좌표로 옮기는 강체 변환 하나를 **실측에서 푼다**(자로 재지 않는다).
  numpy만 쓰며 `fixed`(카메라 고정)와 `on_arm`(손목 장착) 두 식이 있다.
  ⚠ **잔차를 보라** — 최소자승은 입력이 쓰레기여도 답을 낸다. 15mm를 넘으면 집게가 헛집는다.
- **ROS 없이 PC에서 도는 자체검증** (이 저장소의 규칙: 숫자는 젯슨에 올리기 전에 확인한다)
  `python tools/handeye_check.py` (보정 수학 45종) ·
  `python tools/eye_check.py` (보정→통합 배선 60종 — 카메라 둘 포함) ·
  `python ros2/tools/ros_selfcheck.py` (레거시 경계·URDF↔기구학 일치·보드계약·깊이 거절 등 70종).

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
- **팔 3D 좌표 이동 = `arm.cartesian`** ✅ (2026-08-22 추가) — 프리셋이 "저장한 자세로
  점프"라면 이쪽은 **"문 물건을 xyz로 옮기고 제자리에서 돌리기"**.
  [`hardware/kinematics.py`](src/tomato_picker/hardware/kinematics.py)(순/역기구학, import가
  `math`뿐이라 PC에서 검증됨) + [`hardware/cartesian.py`](src/tomato_picker/hardware/cartesian.py)
  (영점·안전·조그). 화면 = `/control` "3D 좌표 이동", 영점 = `/settings` "3D 좌표 영점".
  설정은 `~/arm_cartesian.json`(있으면 config.py보다 이긴다). 자체검증
  `python tools/arm_cartesian_check.py`(팔 없이 33종). 자세한 건
  [`docs/arm-cartesian.md`](docs/arm-cartesian.md).
  ⚠ **5축이라 제자리 yaw는 일반적으로 불가능**하다 — 집게 방향이 위치에 묶여 있다.
  단 **집게가 바닥을 볼 때(pitch≈-90°)만 wrist_roll이 곧 yaw**다. 그 밖에는 roll을 쓴다.
  ⚠ **먼저 영점을 잡아야 열린다** — 교시 자세는 **어깨는 정면, 나머지는 곧게 위로**
  (한 줄로 수직 / `lift=90 elbow=0 wrist=0`). 수평으로 편 자세를 안 쓰는 건 **힘을 빼면
  중력이 끌어내려** 그 처짐이 영점 오차가 되기 때문 — 수직은 문틀에 대보면 눈으로 맞는다.
  세운 직후엔 집게가 회전축 위(특이점)라 **조그가 막히는 게 정상**이다(프리셋으로 먼저 뻗는다).
  팔 범위 캘리브레이션을 다시 하면 **영점도 다시** 잡아야 한다(정규화값의 뜻이 바뀐다).
- **hid-nintendo**: 이 젯슨 커널엔 없어서 out-of-tree 빌드해 설치함(게임패드 필수). 빌드법=`docs/jetson-gamepad-setup.md`.

## 젯슨 접속
- `ssh server@192.168.0.8` (IP는 DHCP라 바뀔 수 있음 / 공개키 등록·passwordless sudo).
  Bash 툴은 `-i /c/Users/parkg/.ssh/id_ed25519` 명시 필요(HOME 다름).
- 장치: `/dev/ttyUSB0`=모터보드, `/dev/ttyACM0`=팔로버, 게임패드=USB/BT.
- **IP를 모를 때는 블루투스로 물어본다** ([`docs/bluetooth-console.md`](docs/bluetooth-console.md), 2026-08-22 추가).
  `ble-console.service`가 BLE로 `tomato-jetson`을 광고하고, **안드로이드 앱**
  ([`android/handset/`](android/handset/), `./build.sh install`) 또는 **PC 앱**
  ([`tools/ble_handset_pc.py`](tools/ble_handset_pc.py), `pip install bleak`)에서
  `ip`·`wifi`·`join`·`status`를 친다.
  ⚠ **연결이 한 번 이루어지면 광고가 멈추고 BlueZ가 되살리지 않는다** — 그래서 손님이
  한 번 다녀가면 젯슨이 스캔에서 사라진다("블루투스 고장"으로 보인다). 서버가
  끊김을 보면 다시 등록하게 해 뒀다. **한 번에 한 대만** 붙을 수 있는 것도 같은 이유다.
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
  ⚠ **팔에도 같은 병이 있다** — STS3215 분해능은 0.088°/틱이라 그보다 작은 지령은
  물리적으로 0이다. 좌표 유닛은 그 크기를 **거절**한다(성공했다고 말한 뒤 아무 일도
  안 일어나는 것보다 낫다). 게인 대신 **크기부터 재라**는 원칙은 팔에서도 같다.
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
