from setuptools import find_packages, setup

package_name = "lite3_driver"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agus-darmawan",
    maintainer_email="darmawandeveloper@gmail.com",
    description="UDP bridge between ROS 2 and the DEEP Robotics Lite3 motion host.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver_node = lite3_driver.driver_node:main",
        ],
    },
)
