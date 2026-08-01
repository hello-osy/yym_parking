from glob import glob

from setuptools import find_packages, setup

package_name = "parking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/parking.yaml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="team",
    maintainer_email="team@example.com",
    description="LiDAR-primary ROS 2 reverse perpendicular parking",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "parking_node = parking.parking_node:main",
            "parking_node_osy = parking.parking_node_osy:main",
            "parking_node_yym = parking.parking_node_yym:main",
        ],
    },
)
