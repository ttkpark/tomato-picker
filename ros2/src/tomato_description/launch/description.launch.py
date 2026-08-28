"""robot_state_publisher — URDF를 TF 트리로 만든다.

이 런치가 하는 일은 하나다: `config/so101_geometry.yaml`(mm)을 읽어 xacro에
인자로 넘기고, 그 결과를 `/robot_description`으로 올린다. **숫자를 두 곳에 적지
않기 위한** 우회로다 — yaml이 정본이고 xacro의 default는 yaml을 못 읽었을 때의
비상용일 뿐이다.

joint_state_publisher_gui는 기본 꺼짐. 실물 팔이 `/joint_states`를 발행하는데
GUI도 같이 발행하면 **두 발행자가 번갈아 이겨서 TF가 떨린다.** 팔 없이 URDF만
볼 때(`gui:=true`)만 켠다.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

PKG = "tomato_description"


def _mappings(path: str) -> dict:
    """yaml → xacro 인자. 키가 없으면 **조용히 기본값으로 넘어가지 않고** 빠뜨린다
    (xacro의 default가 받는다). 값이 있는데 형이 이상하면 여기서 터지는 게 낫다."""
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    arm = cfg.get("arm", {})
    mount = cfg.get("mount", {})
    base = cfg.get("base", {})
    lim = arm.get("limits_deg", {})

    out = {}
    for key in ("z0", "d0", "l1", "l2", "l3"):
        if key in arm:
            out[f"{key}_mm"] = str(float(arm[key]))
    for key, arg in (("x", "mount_x_mm"), ("y", "mount_y_mm"),
                     ("z", "mount_z_mm"), ("yaw_deg", "mount_yaw_deg")):
        if key in mount:
            out[arg] = str(float(mount[key]))
    for key, arg in (("length", "base_length_mm"), ("width", "base_width_mm"),
                     ("height", "base_height_mm"), ("wheel_radius", "wheel_radius_mm")):
        if key in base:
            out[arg] = str(float(base[key]))
    for joint, short in (("shoulder_pan", "pan"), ("shoulder_lift", "lift"),
                         ("elbow_flex", "elbow"), ("wrist_flex", "wflex"),
                         ("wrist_roll", "wroll"), ("gripper", "grip")):
        pair = lim.get(joint)
        if pair:
            out[f"{short}_min_deg"] = str(float(pair[0]))
            out[f"{short}_max_deg"] = str(float(pair[1]))
    return out


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PKG)
    xacro_file = os.path.join(share, "urdf", "tomato_robot.urdf.xacro")
    geometry = os.path.join(share, "config", "so101_geometry.yaml")

    args = " ".join(f"{k}:={v}" for k, v in _mappings(geometry).items())
    robot_description = Command(["xacro ", xacro_file, " ", args])

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="joint_state_publisher_gui로 관절을 손으로 돌려 본다. "
                        "⚠ 실물 팔과 동시에 켜면 TF가 떨린다."),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
            output="screen",
        ),
    ])
