import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lite3_motion"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "waypoints"),
         glob("waypoints/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agus-darmawan",
    maintainer_email="darmawandeveloper@gmail.com",
    description="Timed relative moves and JSON waypoint plans for the Lite3.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_runner = lite3_motion.waypoint_runner:main",
            "move_cli = lite3_motion.move_cli:main",
        ],
    },
)
