#!/usr/bin/env bash
# ROS 환경 + 저장소 경로를 세우고 넘긴다.
#
# ⚠ `set -e`도 `set -u`도 쓰지 않는다.
#
#   · `set -e` — 여기서 죽으면 셸조차 안 뜨고, 컨테이너에서 무엇이 잘못됐는지
#     볼 방법이 사라진다. 대신 문제를 **말하고** 계속 간다.
#   · `set -u` — ROS의 `/opt/ros/$ROS_DISTRO/setup.bash`가 `set -u` 안전하지 않다.
#     (실측: `line 8: AMENT_TRACE_SETUP_FILES: unbound variable`로 엔트리포인트가
#      통째로 죽어 컨테이너가 즉시 종료됐다. colcon은 시작조차 못 했다.)
#     ROS 환경 스크립트는 미정의 변수를 참조하는 것을 정상으로 삼는다.

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
else
  echo "⚠ /ws/install 이 없다 — 아직 빌드하지 않았다. 컨테이너 안에서:"
  echo "     cd /ws && colcon build --symlink-install && source install/setup.bash"
fi

if [ ! -d "${TOMATO_REPO}/src/tomato_picker" ]; then
  echo "⚠ ${TOMATO_REPO}/src/tomato_picker 가 없다 — 저장소를 마운트하지 않았다."
  echo "   기구학·손눈보정·모터링크를 그 저장소에서 그대로 쓰므로 반드시 필요하다."
fi

python3 -c "import cv2, cv2.aruco" 2>/dev/null \
  || echo "⚠ cv2.aruco 가 없다 — 손-눈 보정의 마커 인식이 안 된다 (opencv-contrib 필요)."

exec "$@"
