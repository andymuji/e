from glob import glob

from setuptools import find_packages, setup

package_name = "robot_telemetry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andymuji",
    maintainer_email="andymuji@users.noreply.github.com",
    description=(
        "Recording of safety-relevant runs, and analysis of the recordings "
        "against the safety invariants."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "analyse_run = robot_telemetry.analyse:main",
            "graph_probe = robot_telemetry.graph_probe:main",
        ],
    },
)
