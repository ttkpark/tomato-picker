"""1단계 전부 — URDF/TF · 팔 · 주행 · 인식 · 손눈보정.

기본값은 **가장 안전한 조합**이다: 팔은 proxy(포트를 안 잡는다), 주행은 꺼짐,
카메라 드라이버는 꺼짐(별도로 띄우는 경우가 많아서). 하나씩 켜 가며 붙인다.

    # ① TF만 — 팔도 카메라도 없이 URDF가 맞는지 rviz로 본다
    ros2 launch tomato_bringup stage1.launch.py arm:=false

    # ② 팔 붙이기 (대시보드를 켜 둔 채로)
    ros2 launch tomato_bringup stage1.launch.py arm_mode:=proxy

    # ③ 카메라 + 검출까지
    ros2 launch tomato_bringup stage1.launch.py camera:=true perception:=true

    # ④ 주행까지 (⚠ controller-drive.service를 먼저 끌 것)
    ros2 launch tomato_bringup stage1.launch.py base:=true

⚠ 켜는 순서가 아니라 **끄는 순서**가 중요하다. 포트를 한 프로세스만 열 수 있으니
   `arm_mode:=direct`나 `base:=true`를 쓰기 전에 기존 서비스를 먼저 세워라:
       sudo systemctl stop tomato-voice controller-drive
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
            "arm_mode", default_value="proxy",
            description="proxy=기존 대시보드를 통해(포트 안 잡음) / direct=포트를 직접"),
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
