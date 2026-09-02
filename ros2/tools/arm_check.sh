#!/usr/bin/env bash
# 3~4단 · 팔 검증을 컨테이너 안에서 처음부터 끝까지.
#
#   sudo systemctl stop tomato-voice          # ← 먼저. 포트는 한 프로세스만
#   sudo HOME="$HOME" docker compose run --rm ros bash /ws/tools/arm_check.sh
#   sudo systemctl start tomato-voice         # ← 끝나면 반드시
#
# 런치를 띄우고 → 검사하고 → 반드시 내린다. trap을 거는 이유는 검사가 중간에
# 죽어도 **팔 포트를 붙잡은 채 남으면 안 되기** 때문이다. 그 상태로 두면
# tomato-voice를 되살려도 대시보드가 팔을 못 잡고, 원인이 안 보인다.
set -o pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /ws/install/setup.bash

ros2 launch tomato_bringup stage1.launch.py \
     camera:=false perception:=false > /tmp/stage1.log 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT $LAUNCH 2>/dev/null
  # SIGINT로 안 죽으면 확실히 죽인다 — 포트를 쥔 채 남는 것이 최악이다.
  for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 $LAUNCH 2>/dev/null || break; sleep 0.5; done
  kill -9 $LAUNCH 2>/dev/null
}
trap cleanup EXIT

echo "런치 기동 대기 (팔 연결에 몇 초 걸린다)..."
sleep 12

echo "--- arm_node 로그 ---"
grep -E "팔 |arm|ERROR|error" /tmp/stage1.log | tail -12 | sed 's/^/  /'
echo

python3 /ws/tools/arm_check.py
RC=$?

if [ "$RC" -ne 0 ]; then
  echo
  echo "--- 런치 로그 전체 꼬리 (원인 찾기) ---"
  tail -25 /tmp/stage1.log | sed 's/^/  /'
fi
exit "$RC"
