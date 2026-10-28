import os
from glob import glob

from setuptools import find_packages, setup

package_name = "lite3_workshop"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agus-darmawan",
    maintainer_email="darmawandeveloper@gmail.com",
    description="Workshop labs: first publisher, closed loop with turtlesim, square walk.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hello_pub = lite3_workshop.hello_pub:main",
            "go_to_goal = lite3_workshop.go_to_goal:main",
            "square_walk = lite3_workshop.square_walk:main",
        ],
    },
)
