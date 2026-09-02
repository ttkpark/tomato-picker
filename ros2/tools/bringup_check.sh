#!/usr/bin/env bash
# 컨테이너 안에서 도는 브링업 점검 — **팔도 카메라도 없이** 0~2단을 확인한다.
#
#     sudo HOME="$HOME" docker compose run --rm ros bash /ws/tools/bringup_check.sh
#
# 0단  패키지와 메시지가 실재하는가            (빌드가 진짜였는가)
# 1단  URDF가 펴지고 TF 트리가 서는가          (xacro·런치가 도는가)
# 2단  그 TF가 kinematics.forward()와 같은가   (ROS가 이해한 로봇 == 기구학의 로봇)
#
# 왜 스크립트로 두나 — 이걸 SSH 한 줄로 치면 따옴표가 세 겹(ssh → bash -lc →
# docker run)이 되어 sed 하나에 통째로 깨진다. 실제로 그랬다.
#
# 종료코드 0이면 전부 통과. 손이 필요한 3단 이후(팔·카메라·보정)는 여기 없다.
set -o pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /ws/install/setup.bash

FAIL=0
step() { printf '\n===== %s =====\n' "$1"; }
note() { printf '  %s\n' "$1"; }

step "0단 · 패키지"
PKGS=$(ros2 pkg list | grep '^tomato_')
echo "$PKGS" | sed 's/^/  /'
COUNT=$(echo "$PKGS" | grep -c .)
if [ "$COUNT" -eq 6 ]; then note "ok   6개 전부"; else note "FAIL $COUNT개뿐"; FAIL=1; fi

step "0단 · 메시지·서비스가 생성됐는가"
ros2 interface list 2>/dev/null | grep tomato_msgs | sed 's/^/  /'
if ros2 interface show tomato_msgs/msg/Fruit3D > /dev/null 2>&1; then
  note "ok   Fruit3D 정의를 읽을 수 있다"
else
  note "FAIL Fruit3D를 못 읽는다 — rosidl 생성이 안 됐다"; FAIL=1
fi

step "1단 · URDF와 TF 트리"
ros2 launch tomato_description description.launch.py > /tmp/rsp.log 2>&1 &
RSP=$!
trap 'kill $RSP 2>/dev/null' EXIT
sleep 6

if ros2 topic list 2>/dev/null | grep -q '^/robot_description$'; then
  note "ok   /robot_description 있음"
else
  note "FAIL /robot_description 없음 — xacro가 안 펴졌다:"
  tail -15 /tmp/rsp.log | sed 's/^/       /'
  FAIL=1
fi

LINKS=$(ros2 param get /robot_state_publisher robot_description 2>/dev/null \
        | grep -o '<link name="[^"]*"' | wc -l)
note "링크 $LINKS개"

step "2단 · TF ↔ 기구학 대조"
python3 /ws/tools/tf_check.py || FAIL=1

printf '\n'
if [ "$FAIL" -eq 0 ]; then
  echo "✅ 0~2단 전부 통과 — 손이 필요한 3단(팔)부터는 서비스를 비워야 한다."
else
  echo "❌ 실패한 단이 있다. 위 로그를 보라."
fi
exit "$FAIL"
