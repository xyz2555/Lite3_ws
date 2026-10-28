import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lite3_safety"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agus-darmawan",
    maintainer_email="darmawandeveloper@gmail.com",
    description="Obstacle guard for the Lite3 (ultrasound and depth camera).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "obstacle_guard = lite3_safety.obstacle_guard:main",
        ],
    },
)
