from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mavlink_sensor_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [
            os.path.join("resource", package_name)
        ]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "pymavlink",
    ],
    zip_safe=True,
    maintainer="samuel",
    maintainer_email="samuel16.gendre@gmail.com",
    description="ROS 2 bridge that reads MAVLink IMU and GPS data from a Cube Orange over UART.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mavlink_sensor_bridge_node = mavlink_sensor_bridge.mavlink_sensor_bridge_node:main",
        ],
    },
)
