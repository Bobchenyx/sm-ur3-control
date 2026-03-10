from setuptools import setup
import os
from glob import glob

package_name = "sm_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "twist_relay_node = sm_teleop.twist_relay_node:main",
        ],
    },
)
