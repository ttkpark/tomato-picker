# 젯슨에서 띄우기

## 0. 먼저 자리를 비운다

포트도 카메라도 **한 프로세스만** 연다. 이건 설정이 아니라 물리다.

```bash
sudo systemctl stop tomato-voice              # 대시보드가 /dev/ttyACM0(팔)을 잡고 있다
sudo systemctl stop depth-cam                 # 발행기가 D405를 잡고 있다
sudo systemctl stop controller-drive          # 게임패드가 /dev/ttyUSB0을 잡고 있다
```

⚠ **D405를 빠뜨리기 쉽다.** 팔과 바퀴만 끄고 카메라를 놔두면 `realsense2_camera`가
조용히 못 연다.

`arm_mode:=proxy`(레거시 대시보드 경유)는 **배선 확인용**이다 — 관절값이 HTTP
폴링이라 TF 시각 정렬이 안 되므로 보정·수확에는 쓰지 마라.

## 1. 빌드

이 젯슨은 **Ubuntu 24.04 noble / arm64**라 Jazzy가 배포판 기본이다(그래서
`ros:jazzy-ros-base`가 그대로 맞는다). `server` 계정은 **docker 그룹이 아니라**
`sudo`로 돌린다.

```bash
cd ~/tomato-picker/ros2/docker
sudo HOME="$HOME" docker compose build     # 처음 한 번. 젯슨에서 10~20분
sudo HOME="$HOME" docker compose run --rm ros
```

⚠ **`HOME="$HOME"`을 빠뜨리지 마라.** `sudo`는 HOME을 `/root`로 바꾸는데,
compose가 `${HOME}`을 그대로 마운트에 쓴다(`- ${HOME}:/host-home`). 빠뜨리면
팔 영점(`~/arm_cartesian.json`)과 손-눈 보정(`~/arm_eye.json`)을 **엉뚱한 홈에서**
찾게 되고, 증상은 "보정을 방금 했는데 컨테이너가 없다고 한다"로 나타난다.
(귀찮으면 `sudo usermod -aG docker server` 후 재로그인 — 그 뒤로는 sudo가 필요 없다.)

⚠ **RAM이 빠듯하다** — 7.4Gi 중 `tomato_vision.py`(YOLO)가 2.7Gi를 쓴다. 빌드가
OOM으로 죽으면 잠깐 비운다:

```bash
sudo systemctl stop tomato-vision      # 2.7Gi 확보 (검출만 멈춘다)
colcon build --symlink-install --parallel-workers 1
sudo systemctl start tomato-vision
```

컨테이너 안에서:

```bash
cd /ws && colcon build --symlink-install && source install/setup.bash
```

`--symlink-install`을 쓰는 이유 — 파이썬 노드를 고칠 때마다 다시 빌드하지 않아도
된다. 메시지(`tomato_msgs`)를 고쳤을 때만 다시 빌드하면 된다.

## 2. 하나씩 붙인다

```bash
# ① TF만 — 팔도 카메라도 없이 URDF가 맞는지 본다
ros2 launch tomato_bringup stage1.launch.py arm:=false handeye:=false
ros2 run tf2_tools view_frames        # 프레임 트리를 PDF로

# ② 팔 (ROS가 포트를 직접 잡는다 — 기본값)
ros2 launch tomato_bringup stage1.launch.py
ros2 topic echo /joint_states --once

# ③ 카메라 + 검출
ros2 launch tomato_bringup stage1.launch.py camera:=true perception:=true
ros2 topic echo /fruits --once

# ④ 주행 (⚠ controller-drive를 먼저 끌 것)
ros2 launch tomato_bringup stage1.launch.py base:=true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

## 3. 손-눈 보정 (한 번만, 카메라를 옮기면 다시)

```bash
# 팔을 옮긴다 → 멈춘다 → 찍는다. 8번 이상, 서로 멀리 떨어뜨려서.
ros2 service call /handeye/capture_sample tomato_msgs/srv/CaptureSample "{label: '왼쪽 아래'}"
...
ros2 service call /handeye/solve tomato_msgs/srv/SolveHandEye "{save: true}"
```

저장 위치는 **`~/arm_eye.json`** — 레거시 대시보드와 **같은 파일**이다. 어느 쪽에서
잡든 보정은 한 벌이고, 다른 쪽이 그대로 쓴다.

잔차 RMS가 15mm를 넘으면 저장되지 않는다. `worst_index`가 가리키는 표본을 지우고
다시 풀어라:

```bash
ros2 service call /handeye/drop_worst std_srvs/srv/Trigger
ros2 service call /handeye/solve tomato_msgs/srv/SolveHandEye "{save: false}"
```

## 4. 좌표로 집으러 보내기

```bash
# 먼저 dry_run으로 IK가 풀리는지 본다 (팔은 안 움직인다)
ros2 service call /arm/move_to_point tomato_msgs/srv/MoveToPoint \
  "{target: {header: {frame_id: 'arm_base'}, point: {x: 0.22, y: 0.0, z: 0.10}},
    approach_pitch_deg: -20.0, standoff_m: 0.05, dry_run: true}"
```

---

## 자주 겪는 것들

| 증상 | 원인 |
|---|---|
| 노드는 떴는데 다른 기계에서 토픽이 안 보인다 | `network_mode: host`가 빠졌거나 `ROS_DOMAIN_ID`가 다르다 |
| `/joint_states`가 안 온다 | proxy 모드에서 대시보드가 안 떠 있다 (`curl localhost:8090/status`) |
| rviz에서 팔이 안 그려진다 | `/joint_states`가 없으면 robot_state_publisher가 TF를 못 만든다 |
| TF 조회가 오락가락한다 | 카메라 광학 프레임에 부모가 둘 — `store.retarget` 주석 참고 |
| 열매가 **하나도** 안 나온다 | 깊이가 컬러에 정렬됐는지(`align_depth.enable`), `~/annotated`를 봐라 |
| 팔을 못 연다 | `tomato-voice`가 살아 있다. `systemctl stop` 후 다시 (proxy로 도망가지 말 것) |
| 카메라를 못 연다 | `depth-cam.service`가 D405를 잡고 있다 |
| 바퀴가 안 돈다 | `/cmd_vel`이 끊기면 0.3초 뒤 데드맨이 세운다. 계속 발행해야 한다 |
| 보정 파일이 재시작마다 사라진다 | `HOME=/host-home` 마운트가 빠졌다 (compose 주석 참고) |
| `bash /ws/tools/*.sh`가 `set: pipefail: invalid option name`으로 죽는다 | 그 파일이 CRLF다 — Windows 작업트리에서 그대로 scp됐다. 고치는 법은 아래 §줄끝 |
| 시험기록이 **어제 날짜** 파일에 붙는다 | 컨테이너가 UTC였다 — `/etc/localtime` 마운트가 빠졌다. 아래 §시각대 |

## 시각대 — 컨테이너는 UTC다, 기록 이름은 날짜다 (2026-09-18)

`move5_check.py`는 매 시도를 `docs/시험기록/move-to-point-<날짜>.jsonl`에 남긴다.
그런데 도커 기본 시각대는 UTC라서 **한국 자정~09시에 돌리면 어제 파일에 붙는다**
(실측: 호스트 `Fri Sep 18 03:10 KST` ↔ 컨테이너 `Thu Sep 17 18:10 UTC`). 다음
사이클은 파일 이름을 사실로 믿으므로, 하루치 실기가 어제 것으로 읽힌다.

`TZ=Asia/Seoul`을 compose에 적는 대신 **호스트의 것을 물렸다**
(`/etc/localtime`·`/etc/timezone` 읽기전용) — 젯슨이 이미 자기 시각대를 안다.
존 이름을 베껴 두면 젯슨을 다른 곳으로 옮겼을 때 두 곳이 어긋난다. 그리고
`docker run`으로 직접 띄우면 마운트가 없어 다시 UTC이므로, **실행이 스스로
시각대를 말한다** — `1개 표적, 기록: …/move-to-point-2026-09-18.jsonl  [KST UTC+9]`.

강제 검사 = `python ros2/tools/ros_selfcheck.py` **[시각]** 4종.

## 줄끝 — 컨테이너에 CRLF를 들이면 셸이 통째로 죽는다 (2026-09-18)

이 저장소는 Windows에서 고쳐 **젯슨으로 scp**한다. `core.autocrlf=true`면 작업트리에
CRLF로 체크아웃되고 scp는 바이트를 그대로 보내므로, 리눅스에 CRLF 셸 스크립트가
도착한다. bash는 그걸 한 줄도 못 읽는다(`set: pipefail: invalid option name` —
2026-09-18 사이클25에 `bringup_check.sh`가 이렇게 막혀 졸업기준 1번 계측이 하루
미뤄졌다). systemd 유닛도 같은 병을 앓는다(`ExecStart` 값 끝에 CR이 붙어 실행 파일
이름이 틀린다). `entrypoint.sh`는 이미지에 `COPY`되므로 **CRLF로 빌드하면 컨테이너가
아예 안 뜬다.**

고친 자리는 배포 경로의 **입구**다 — 저장소 루트 `.gitattributes`가 배포되는 종류
(`*.sh` `*.py` `*.service` `*.rules` `*.yml` `*.yaml` `*.xacro` `*.urdf` `Dockerfile`)를
`text eol=lf`로 못 박는다. 그래서 Windows에서 체크아웃해도 LF다. 강제 검사 =
`python ros2/tools/ros_selfcheck.py` **[줄끝]** 13종(패턴이 지워지거나 작업트리에
CR이 섞이면 실패한다 — 돌연변이로 확인했다).

받는 쪽에서 고치는 것(`sed -i 's/\r$//'`)은 **임시 처치**다. 다음 scp가 되돌려 놓는다.

## GPU가 필요해지면

지금 1단계는 HSV 색검출이라 CPU로 충분하다. YOLO를 넣는 순간 베이스 이미지를
`dustynv/l4t-pytorch` 계열로 바꾸고 그 위에 ROS를 얹어야 한다 — `--runtime nvidia`도
같이. 그 전까지는 이 이미지가 가볍고 **PC에서도 그대로 빌드된다**(시뮬레이션·rviz용).
