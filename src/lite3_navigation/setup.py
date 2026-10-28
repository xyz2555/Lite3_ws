import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lite3_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lite3-team",
    maintainer_email="todo@example.com",
    description="Closed-loop go-to-point for the Lite3 (RViz2 2D Goal Pose, odom TF).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go_to_point = lite3_navigation.go_to_point_node:main",
            "fake_lite3 = lite3_navigation.fake_lite3:main",
        ],
    },
)
