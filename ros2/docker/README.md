# 젯슨에서 띄우기

## 0. 먼저 자리를 비운다

포트는 **한 프로세스만** 연다. 이건 설정이 아니라 물리다.

```bash
sudo systemctl stop controller-drive          # 게임패드가 /dev/ttyUSB0을 잡고 있다
sudo systemctl stop tomato-voice              # 대시보드가 /dev/ttyACM0을 잡고 있다
```

`tomato-voice`를 **끄고 싶지 않다면** 팔을 `arm_mode:=proxy`로 쓴다 — 그 프로세스를
통해 팔을 읽고 움직이므로 포트를 안 잡는다. 대신 주기가 ~10Hz로 떨어진다.

## 1. 빌드

```bash
cd ~/tomato-picker/ros2/docker
docker compose build                 # 처음 한 번. 젯슨에서 10~20분
docker compose run --rm ros          # 셸
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

# ② 팔
ros2 launch tomato_bringup stage1.launch.py arm_mode:=proxy
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
| 팔을 못 연다 (`direct`) | `tomato-voice`가 살아 있다. `systemctl stop` 후 다시 |
| 바퀴가 안 돈다 | `/cmd_vel`이 끊기면 0.3초 뒤 데드맨이 세운다. 계속 발행해야 한다 |
| 보정 파일이 재시작마다 사라진다 | `HOME=/host-home` 마운트가 빠졌다 (compose 주석 참고) |

## GPU가 필요해지면

지금 1단계는 HSV 색검출이라 CPU로 충분하다. YOLO를 넣는 순간 베이스 이미지를
`dustynv/l4t-pytorch` 계열로 바꾸고 그 위에 ROS를 얹어야 한다 — `--runtime nvidia`도
같이. 그 전까지는 이 이미지가 가볍고 **PC에서도 그대로 빌드된다**(시뮬레이션·rviz용).
