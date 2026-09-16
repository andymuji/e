"""Checks on the URDF that do not need ROS, Gazebo, or a running robot.

They exist because the safety behaviour is derived from this geometry: a
silently broken frame or a rewired velocity topic is a safety regression, not
just a modelling mistake.
"""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

URDF_DIR = Path(__file__).resolve().parents[1] / "urdf"
XACRO_NS = "http://www.ros.org/wiki/xacro"


def parse(name: str) -> ET.Element:
    return ET.parse(URDF_DIR / name).getroot()


class RobotDescriptionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.robot = parse("robot.urdf.xacro")
        self.gazebo = parse("robot.gazebo.xacro")

    def test_every_urdf_file_is_well_formed(self) -> None:
        for path in sorted(URDF_DIR.glob("*.xacro")):
            with self.subTest(path=path.name):
                ET.parse(path)

    def test_declares_the_frames_navigation_depends_on(self) -> None:
        links = {link.get("name") for link in self.robot.iter("link")}

        for frame in ("base_footprint", "base_link", "lidar_link", "imu_link"):
            self.assertIn(frame, links)

    def test_every_joint_connects_declared_links(self) -> None:
        links = {link.get("name") for link in self.robot.iter("link")}
        macro_links = {"${prefix}_wheel_link"}

        for joint in self.robot.iter("joint"):
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            with self.subTest(joint=joint.get("name")):
                self.assertIn(parent, links | macro_links)
                self.assertIn(child, links | macro_links)

    def test_drive_wheels_are_the_only_movable_joints(self) -> None:
        movable = {
            joint.get("name")
            for joint in self.robot.iter("joint")
            if joint.get("type") != "fixed"
        }

        self.assertEqual(movable, {"${prefix}_wheel_joint"})

    def test_every_link_has_inertia(self) -> None:
        # A link without inertia is silently dropped or destabilised by the
        # physics engine, which makes simulator results meaningless.
        #
        # These files are read unexpanded, because xacro is not importable
        # without ROS. Inertia therefore arrives either as a literal element
        # or as a call to the box_inertial macro; this catches a link with
        # neither, not a macro that computes the wrong numbers.
        sources = ("inertial", f"{{{XACRO_NS}}}box_inertial")

        for link in self.robot.iter("link"):
            if link.get("name") == "base_footprint":
                continue  # A massless frame, by convention.
            with self.subTest(link=link.get("name")):
                self.assertTrue(
                    any(link.find(source) is not None for source in sources),
                    f"{link.get('name')} declares no inertia",
                )

    def test_drive_plugin_listens_to_the_gated_topic(self) -> None:
        plugins = {
            plugin.get("name"): plugin for plugin in self.gazebo.iter("plugin")
        }
        diff_drive = plugins["gz::sim::systems::DiffDrive"]

        # robot_safety publishes cmd_vel. If this ever becomes
        # cmd_vel_requested, every request reaches the wheels ungated.
        self.assertEqual(diff_drive.find("topic").text, "cmd_vel")

    def test_drive_plugin_uses_the_declared_wheel_geometry(self) -> None:
        properties = {
            element.get("name"): element.get("value")
            for element in self.robot.iter(f"{{{XACRO_NS}}}property")
        }
        diff_drive = next(
            plugin
            for plugin in self.gazebo.iter("plugin")
            if plugin.get("name") == "gz::sim::systems::DiffDrive"
        )

        # Odometry drifts quietly if these disagree with the URDF.
        self.assertEqual(diff_drive.find("wheel_radius").text, "${wheel_radius}")
        self.assertEqual(
            diff_drive.find("wheel_separation").text, "${wheel_separation}"
        )
        self.assertIn("wheel_radius", properties)
        self.assertIn("wheel_separation", properties)

    def test_placeholder_geometry_is_physically_consistent(self) -> None:
        # These numbers are placeholders meant to be replaced with measured
        # ones. That edit is exactly when a robot whose chassis scrapes the
        # floor, or whose wheels are buried inside its own body, gets built
        # without anyone noticing until it is in the simulator.
        properties = {
            element.get("name"): float(element.get("value"))
            for element in self.robot.iter(f"{{{XACRO_NS}}}property")
        }

        chassis_bottom = properties["base_clearance"] - properties["base_height"] / 2
        self.assertGreater(chassis_bottom, 0.0, "chassis intersects the ground")

        wheel_inner_face = (
            properties["wheel_separation"] / 2 - properties["wheel_width"] / 2
        )
        self.assertGreaterEqual(
            wheel_inner_face,
            properties["base_width"] / 2,
            "wheels intersect the chassis",
        )

        self.assertGreater(
            properties["base_clearance"],
            properties["caster_radius"],
            "caster cannot reach the ground",
        )

    def test_lidar_publishes_the_topic_the_safety_gate_reads(self) -> None:
        lidar = next(
            sensor
            for sensor in self.gazebo.iter("sensor")
            if sensor.get("type") == "gpu_lidar"
        )

        self.assertEqual(lidar.find("topic").text, "scan")
        self.assertGreater(float(lidar.find("update_rate").text), 0.0)

    def test_lidar_sees_closer_than_the_safety_stop_distance(self) -> None:
        from robot_safety import SafetyController

        lidar = next(
            sensor
            for sensor in self.gazebo.iter("sensor")
            if sensor.get("type") == "gpu_lidar"
        )
        range_min = float(lidar.find(".//range/min").text)
        range_max = float(lidar.find(".//range/max").text)
        controller = SafetyController()

        # Readings outside the lidar's range window are discarded, so a
        # range_min above the stop distance would make close obstacles
        # invisible at exactly the distance that matters most.
        self.assertLess(range_min, controller.stop_distance)
        self.assertGreater(range_max, controller.caution_distance)


if __name__ == "__main__":
    unittest.main()
