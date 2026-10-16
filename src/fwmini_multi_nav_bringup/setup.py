from glob import glob
from os.path import join
from setuptools import setup

name = "fwmini_multi_nav_bringup"
setup(
    name=name, version="0.1.0", packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        (join("share", name, "launch"), glob("launch/*.launch.py")),
        (join("share", name, "config", "cartographer"), glob("config/cartographer/*.lua")),
        (join("share", name, "config", "slam_toolbox"), glob("config/slam_toolbox/*.yaml")),
        (join("share", name, "config"), glob("config/*.json")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="FW-mini integration", maintainer_email="maintainer@example.invalid",
    description="Multi-backend SLAM selector for ROS 2 Humble FW-mini",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "fwmini_preflight = fwmini_multi_nav_bringup.preflight:main",
    ]},
)
