1: from glob import glob
2: from os.path import join
3: from setuptools import setup
4: 
5: name = "fwmini_multi_nav_bringup"
6: setup(
7:     name=name, version="0.1.0", packages=[name],
8:     data_files=[
9:         ("share/ament_index/resource_index/packages", ["resource/" + name]),
10:         ("share/" + name, ["package.xml"]),
11:         (join("share", name, "launch"), glob("launch/*.launch.py")),
12:         (join("share", name, "config", "cartographer"), glob("config/cartographer/*.lua")),
13:         (join("share", name, "config", "slam_toolbox"), glob("config/slam_toolbox/*.yaml")),
14:         (join("share", name, "config"), glob("config/*.json")),
15:     ],
16:     install_requires=["setuptools"], zip_safe=True,
17:     maintainer="FW-mini integration", maintainer_email="maintainer@example.invalid",
18:     description="Multi-backend SLAM selector for ROS 2 Humble FW-mini",
19:     license="Apache-2.0",
20:     entry_points={"console_scripts": [
21:         "fwmini_preflight = fwmini_multi_nav_bringup.preflight:main",
22:     ]},
23: )