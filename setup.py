import os
from glob import glob
from setuptools import setup

package_name = "icm_20948"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz2"), glob("rviz2/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alpaca-zip",
    maintainer_email="zip@todo.todo",
    description="ROS2 driver for the ICM-20948 with Seeeduino XIAO",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["imu_node = icm_20948.imu_node:main"],
    },
)
