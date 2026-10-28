from setuptools import find_packages, setup

package_name = "robot_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andymuji",
    maintainer_email="andymuji@users.noreply.github.com",
    description=(
        "Driver for the mecanum base: the safety gate's /cmd_vel to the "
        "Pico's wheels, and encoders to /odom."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_node = robot_base.base_node:main",
        ],
    },
)
