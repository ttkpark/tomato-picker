# ROS 2 Jazzy — 젯슨 L4T 커널 위에 얹는 컨테이너.
#
# 왜 컨테이너인가 (도커를 좋아해서가 아니다):
#   · 젯슨의 시스템 파이썬에는 **lerobot venv**가 얹혀 있다. ROS를 apt로 깔면
#     그 위에 또 하나의 파이썬 세계가 생기고, 둘이 numpy/opencv 버전을 두고
#     싸운다. 지금 도는 데모를 깨뜨리지 않는 게 첫 번째 조건이다.
#   · L4T는 우분투 배포판이 커널에 묶여 있어 되돌리기가 어렵다. 컨테이너는
#     `docker rm` 한 줄로 되돌아간다.
#   · 새 젯슨/새 PC에 올릴 때 "무엇을 깔았더라"가 이 파일 하나로 남는다.
#
# ⚠ **GPU가 필요해지면 베이스를 바꿔야 한다.** 지금 1단계는 HSV 색검출이라
#    CPU로 충분하다. YOLO를 넣는 순간 dustynv/l4t-pytorch 계열 이미지 + nvidia
#    런타임으로 옮겨야 하고, 그때는 ROS를 그 위에 얹는 형태가 된다.
#    (그 전에는 이 이미지가 가볍고 어디서나 빌드된다 — PC에서도 돈다.)
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-numpy python3-yaml python3-serial python3-opencv \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-realsense2-camera \
      ros-${ROS_DISTRO}-realsense2-description \
 && rm -rf /var/lib/apt/lists/*

# ArUco는 opencv **contrib**에 있다. 배포판 python3-opencv에 빠져 있는 경우가
# 있어서, 없으면 그때만 휠로 채운다(있으면 건드리지 않는다 — 두 벌의 cv2가
# 깔리면 import 순서에 따라 다른 게 잡혀 진단이 지옥이 된다).
RUN python3 -c "import cv2; cv2.aruco" 2>/dev/null \
 || pip3 install --no-cache-dir --break-system-packages opencv-contrib-python

WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 워크스페이스와 저장소는 **볼륨으로 마운트**한다(COPY하지 않는다).
# 코드를 고칠 때마다 이미지를 다시 굽지 않으려는 것이고, 젯슨에서 고친 값이
# 컨테이너 밖에도 남게 하려는 것이다.
ENV TOMATO_REPO=/repo
ENV PYTHONPATH=/repo/src

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
