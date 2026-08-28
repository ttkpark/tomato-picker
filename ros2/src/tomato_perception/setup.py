from glob import glob

from setuptools import find_packages, setup

package_name = "tomato_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giho Park",
    maintainer_email="juhyung1021@gmail.com",
    description="D405에서 열매 3D 좌표를 뽑는다",
    license="MIT",
    entry_points={
        "console_scripts": [
            "detect_node = tomato_perception.detect_node:main",
        ],
    },
)
