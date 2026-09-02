#!/usr/bin/env bash
# 5~6단 · 카메라와 검출. **팔은 안 쓴다**(arm:=false) — tomato-voice를 안 내려도 된다.
#
#   sudo systemctl stop depth-cam            # ← D405도 한 프로세스만 연다
#   sudo HOME="$HOME" docker compose run --rm ros bash /ws/tools/cam_check.sh
#   sudo systemctl start depth-cam           # ← 끝나면 반드시
set -o pipefail
source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /ws/install/setup.bash

ros2 launch tomato_bringup stage1.launch.py \
     arm:=false handeye:=false camera:=true perception:=true > /tmp/cam.log 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT $LAUNCH 2>/dev/null
  for _ in $(seq 1 12); do kill -0 $LAUNCH 2>/dev/null || break; sleep 0.5; done
  kill -9 $LAUNCH 2>/dev/null
}
trap cleanup EXIT

echo "카메라 기동 대기 (D405 열거에 시간이 걸린다)..."
sleep 15
echo "--- 런치 로그 ---"
grep -E "RealSense|Device|error|ERROR|WARN|검출|내부" /tmp/cam.log | tail -12 | sed 's/^/  /'
echo
python3 /ws/tools/cam_check.py
RC=$?
[ "$RC" -ne 0 ] && { echo; echo "--- 로그 꼬리 ---"; tail -30 /tmp/cam.log | sed 's/^/  /'; }
exit "$RC"
