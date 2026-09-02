from glob import glob

from setuptools import find_packages, setup

package_name = "tomato_handeye"

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
    description="손-눈 보정",
    license="MIT",
    entry_points={
        "console_scripts": [
            "handeye_node = tomato_handeye.handeye_node:main",
        ],
    },
)
