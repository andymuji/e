from glob import glob

from setuptools import find_packages, setup

package_name = "robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        # navigation.launch.py defaults to maps/test_room.yaml in the share
        # directory, so the map has to be installed as well as committed.
        ("share/" + package_name + "/maps", glob("maps/*.yaml") + glob("maps/*.pgm")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "world_to_map = robot_bringup.world_to_map:main",
        ],
    },
    maintainer="andymuji",
    maintainer_email="andymuji@users.noreply.github.com",
    description="Launch files, parameters, and simulation worlds for the robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
