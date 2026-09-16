from glob import glob

from setuptools import find_packages, setup

package_name = "robot_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andymuji",
    maintainer_email="andymuji@users.noreply.github.com",
    description="URDF, meshes, and transforms for the robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
