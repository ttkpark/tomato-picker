from glob import glob

from setuptools import find_packages, setup

package_name = "tomato_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giho Park",
    maintainer_email="juhyung1021@gmail.com",
    description="기존 하드웨어를 ROS 2에 물리는 다리",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_node = tomato_bridge.arm_node:main",
            "cmd_vel_node = tomato_bridge.cmd_vel_node:main",
        ],
    },
)
