"""1단계 전부 — URDF/TF · 팔 · 주행 · 인식 · 손눈보정.

**켜기 전에 자리를 비운다.** 장치는 한 프로세스만 연다 — 팔(`/dev/ttyACM0`),
주행 보드(`/dev/ttyUSB0`), 그리고 **D405도 마찬가지다**(`depth-cam.service`가
잡고 있으면 realsense2_camera가 못 연다).

    sudo systemctl stop tomato-voice depth-cam       # 팔 + 카메라
    sudo systemctl stop controller-drive             # 주행까지 쓸 때

그다음 하나씩 켜 가며 붙인다.

    # ① TF만 — 팔도 카메라도 없이 URDF가 맞는지 rviz로 본다
    ros2 launch tomato_bringup stage1.launch.py arm:=false handeye:=false

    # ② 팔 (ROS가 포트를 직접 잡는다)
    ros2 launch tomato_bringup stage1.launch.py

    # ③ 카메라 + 검출까지
    ros2 launch tomato_bringup stage1.launch.py camera:=true perception:=true

    # ④ 주행까지
    ros2 launch tomato_bringup stage1.launch.py base:=true

⚠ `arm_mode:=proxy`는 **레거시 대시보드를 켜 둔 채 배선만 확인하는** 임시 경로다.
   관절값이 HTTP 폴링이라 TF 시각 정렬이 안 되므로 보정·수확에는 쓰지 마라
   (v2.0.0-ros.3에서 삭제). 포트가 막히면 도망갈 곳이 아니라 **끌 서비스**를 봐라.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAMS = os.path.join(get_package_share_directory("tomato_bringup"),
                      "config", "stage1.yaml")


def generate_launch_description() -> LaunchDescription:
    description_launch = os.path.join(
        get_package_share_directory("tomato_description"), "launch",
        "description.launch.py")

    args = [
        DeclareLaunchArgument("arm", default_value="true"),
        DeclareLaunchArgument(
            "arm_mode", default_value="direct",
            description="direct=ROS가 포트를 직접(기본) / proxy=레거시 대시보드 경유(브링업 전용)"),
        DeclareLaunchArgument("base", default_value="false",
                              description="⚠ controller-drive를 먼저 끌 것"),
        DeclareLaunchArgument("camera", default_value="false",
                              description="realsense2_camera를 여기서 띄운다"),
        DeclareLaunchArgument("perception", default_value="false"),
        DeclareLaunchArgument("handeye", default_value="true",
                              description="저장된 보정이 있으면 TF로 되살린다"),
    ]

    return LaunchDescription(args + [
        # TF 트리는 **언제나** 띄운다. 이게 없으면 나머지가 전부 의미가 없다.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch)),

        Node(package="tomato_bridge", executable="arm_node", name="tomato_arm",
             output="screen", parameters=[PARAMS, {
                 "arm_mode": LaunchConfiguration("arm_mode")}],
             condition=IfCondition(LaunchConfiguration("arm"))),

        Node(package="tomato_bridge", executable="cmd_vel_node", name="tomato_base",
             output="screen", parameters=[PARAMS],
             condition=IfCondition(LaunchConfiguration("base"))),

        # D405. align_depth가 **필수**다 — 깊이와 컬러가 어긋나면 열매 중심의
        # 깊이가 옆 잎의 깊이가 되고, 아무 에러도 안 난다.
        Node(package="realsense2_camera", executable="realsense2_camera_node",
             name="camera", namespace="camera", output="screen",
             parameters=[{"align_depth.enable": True,
                          "enable_color": True,
                          "enable_depth": True,
                          "pointcloud.enable": False,
                          "rgb_camera.color_profile": "848x480x30",
                          "depth_module.depth_profile": "848x480x30"}],
             condition=IfCondition(LaunchConfiguration("camera"))),

        Node(package="tomato_perception", executable="detect_node",
             name="tomato_detect", output="screen", parameters=[PARAMS],
             condition=IfCondition(LaunchConfiguration("perception"))),

        Node(package="tomato_handeye", executable="handeye_node",
             name="tomato_handeye", output="screen", parameters=[PARAMS],
             condition=IfCondition(LaunchConfiguration("handeye"))),
    ])
