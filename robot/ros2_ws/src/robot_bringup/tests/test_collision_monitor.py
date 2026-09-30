"""Checks on the Collision Monitor's zones and on where its output goes.

Two separate worries, and they fail for different reasons.

The geometry tests exist because the zones are the robot's shape and the
gate's braking distance restated in a third file. Hand-written, they would
let the robot be resized without the monitor noticing - the same failure the
Nav2 footprint tests guard against, one layer further out. Nothing here
accepts a number that was typed in; every distance is recomputed from the
URDF and base_dynamics.yaml and compared.

The topology tests exist because Nav2 ships this node publishing straight to
/cmd_vel. That is correct upstream, where the Collision Monitor is the last
thing before the wheels, and it is wrong here, where the safety gate is. The
upstream default is one word away at all times, so it is asserted against by
name.

None of this needs ROS, Gazebo, or nav2_collision_monitor to be installed.
"""

import ast
from pathlib import Path
import unittest

import yaml

SHARE = Path(__file__).resolve().parents[1]
CONFIG = SHARE / "config"
LAUNCH = SHARE / "launch"

CONFIG_FILE = CONFIG / "collision_monitor.yaml"
LAUNCH_FILE = LAUNCH / "collision_monitor.launch.py"

# Where the monitor's output must go, and the one place it must never go.
REQUEST_TOPIC = "cmd_vel_requested"
WHEEL_TOPIC = "cmd_vel"


def zone_corners(points: str) -> list[tuple[float, float]]:
    """The polygon as pairs of numbers.

    Nav2 Jazzy takes this parameter as a string of [x, y] pairs, in the same
    form as the costmap footprint, so it is parsed rather than read as YAML.
    """
    return [tuple(corner) for corner in ast.literal_eval(points)]


def urdf_dimensions() -> dict[str, float]:
    """The numeric xacro properties from the URDF, without expanding it."""
    import xml.etree.ElementTree as ET

    namespace = "http://www.ros.org/wiki/xacro"
    urdf = SHARE.parent / "robot_description" / "urdf" / "robot.urdf.xacro"
    robot = ET.parse(urdf).getroot()

    dimensions = {}
    for element in robot.iter(f"{{{namespace}}}property"):
        try:
            dimensions[element.get("name")] = float(element.get("value"))
        except ValueError:
            continue  # A non-numeric property, such as a material name.
    return dimensions


class CollisionMonitorGeometryTests(unittest.TestCase):
    """The zones must be the robot's real shape and the gate's real distances.

    A zone that is too short stops too late. A zone that is too narrow misses
    what the robot is about to hit with its shoulder. Both are the kind of
    mistake that survives review and shows up on a person, so neither number
    is allowed to be independent of the URDF.
    """

    def setUp(self) -> None:
        from robot_navigation import BaseFootprint
        from robot_safety import BaseDynamics
        from robot_safety.distances import derive_safety_distances

        self.params = yaml.safe_load(CONFIG_FILE.read_text())[
            "collision_monitor"
        ]["ros__parameters"]

        self.dynamics = BaseDynamics.from_mapping(
            yaml.safe_load((CONFIG / "base_dynamics.yaml").read_text())
        )
        self.distances = derive_safety_distances(self.dynamics)

        # The same footprint the Nav2 costmaps use, from the same source.
        # Nav2GeometryTests in test_bringup.py builds it identically; the
        # clearance margin is the planner's and does not enter these zones.
        dimensions = urdf_dimensions()
        self.footprint = BaseFootprint(
            length=dimensions["base_length"],
            width=dimensions["base_width"],
            clearance_margin=0.25,
        )

    def reach_ahead(self, gate_distance: float) -> float:
        """How far ahead of base_link a zone must reach for a gate distance.

        The gate's distances are measured from the lidar; the monitor works in
        base_footprint. `sensor_to_front_edge` is the gap between the two, so
        subtracting it converts one into the other. Written out here rather
        than inlined because getting this conversion backwards would make
        every zone 0.10 m wrong in the same direction.
        """
        return self.footprint.length / 2.0 + (
            gate_distance - self.dynamics.sensor_to_front_edge
        )

    def test_the_stop_zone_reaches_as_far_as_the_gate_would_need_to_brake(
        self,
    ) -> None:
        corners = zone_corners(self.params["StopZone"]["points"])
        expected = self.reach_ahead(self.distances.stop_distance)

        self.assertAlmostEqual(max(x for x, _ in corners), expected, places=6)

    def test_the_slowdown_zone_reaches_as_far_as_the_gates_caution_distance(
        self,
    ) -> None:
        corners = zone_corners(self.params["SlowdownZone"]["points"])
        expected = self.reach_ahead(self.distances.caution_distance)

        self.assertAlmostEqual(max(x for x, _ in corners), expected, places=6)

    def test_both_zones_are_exactly_as_wide_as_the_robot(self) -> None:
        # Wider, and the robot stops for walls it is driving past, which is
        # the behaviour this layer exists to avoid repeating. Narrower, and it
        # drives its own corner into something the zone did not cover.
        half_width = self.footprint.width / 2.0

        for name in ("StopZone", "SlowdownZone"):
            with self.subTest(zone=name):
                corners = zone_corners(self.params[name]["points"])

                self.assertAlmostEqual(max(y for _, y in corners), half_width, 6)
                self.assertAlmostEqual(min(y for _, y in corners), -half_width, 6)

    def test_both_zones_start_at_the_back_of_the_robot(self) -> None:
        # The zone contains the robot, so an obstacle that is already
        # alongside the chassis is inside it rather than behind its near edge.
        expected = -self.footprint.length / 2.0

        for name in ("StopZone", "SlowdownZone"):
            with self.subTest(zone=name):
                corners = zone_corners(self.params[name]["points"])

                self.assertAlmostEqual(min(x for x, _ in corners), expected, 6)

    def test_the_slowdown_zone_contains_the_stop_zone(self) -> None:
        # Reversed, the robot would be told to stop before it was ever told to
        # slow down, and would brake from full speed every time.
        stop = zone_corners(self.params["StopZone"]["points"])
        slowdown = zone_corners(self.params["SlowdownZone"]["points"])

        self.assertGreater(max(x for x, _ in slowdown), max(x for x, _ in stop))

    def test_the_slowdown_speed_matches_the_gates_caution_speed(self) -> None:
        # Two layers disagreeing about what "go slowly" means would make the
        # robot's speed depend on which one happened to fire.
        self.assertAlmostEqual(
            self.params["SlowdownZone"]["slowdown_ratio"],
            self.dynamics.caution_speed_scale,
            places=6,
        )

    def test_every_zone_is_a_closed_polygon_of_at_least_three_corners(
        self,
    ) -> None:
        for name in self.params["polygons"]:
            with self.subTest(zone=name):
                corners = zone_corners(self.params[name]["points"])

                self.assertGreaterEqual(len(corners), 3)
                for corner in corners:
                    self.assertEqual(len(corner), 2)

    def test_the_monitor_does_not_tolerate_staler_scans_than_the_gate(
        self,
    ) -> None:
        # A second layer working from older data than the layer behind it
        # would report clear while the gate was already stopping.
        gate = yaml.safe_load((CONFIG / "safety.yaml").read_text())
        gate_timeout = gate["safety_controller"]["ros__parameters"][
            "sensor_timeout"
        ]

        self.assertLessEqual(self.params["source_timeout"], gate_timeout)

    def test_the_declared_zones_all_exist(self) -> None:
        # A name in `polygons` with no matching block is a zone Nav2 will
        # refuse to start with, and a protection nobody gets.
        for name in self.params["polygons"]:
            with self.subTest(zone=name):
                self.assertIn(name, self.params)
                self.assertIn("points", self.params[name])

    def test_there_is_a_stop_zone_and_a_slowdown_zone(self) -> None:
        actions = {
            name: self.params[name]["action_type"]
            for name in self.params["polygons"]
        }

        self.assertIn("stop", actions.values())
        self.assertIn("slowdown", actions.values())

    def test_no_zone_can_ask_the_robot_to_keep_moving(self) -> None:
        # "approach" makes the monitor slow the robot towards a collision
        # rather than stop short of it, which is a different safety argument
        # from the one made at the top of the yaml. If it is ever wanted, it
        # needs that argument rewritten, not this assertion deleted.
        for name in self.params["polygons"]:
            with self.subTest(zone=name):
                self.assertIn(
                    self.params[name]["action_type"],
                    {"stop", "slowdown", "limit"},
                )

    def test_every_zone_is_switched_on(self) -> None:
        # A zone present but disabled looks like protection in a diff.
        for name in self.params["polygons"]:
            with self.subTest(zone=name):
                self.assertTrue(self.params[name]["enabled"])

    def test_the_monitor_watches_the_same_lidar_the_gate_does(self) -> None:
        self.assertEqual(self.params["observation_sources"], ["scan"])
        self.assertEqual(self.params["scan"]["topic"], "scan")
        self.assertTrue(self.params["scan"]["enabled"])


class CollisionMonitorTopologyTests(unittest.TestCase):
    """The monitor constrains requests; it must never command the wheels.

    Nav2's own default for `cmd_vel_out_topic` is "cmd_vel", because upstream
    puts this node last. Here the safety gate is last, so taking the upstream
    default - or copying an upstream example - would put a second publisher on
    the topic that reaches the motors and break the single property the whole
    project rests on.
    """

    def setUp(self) -> None:
        self.params = yaml.safe_load(CONFIG_FILE.read_text())[
            "collision_monitor"
        ]["ros__parameters"]
        self.launch = LAUNCH_FILE.read_text()

    def test_the_monitor_publishes_a_request_and_not_a_wheel_command(
        self,
    ) -> None:
        out = self.params["cmd_vel_out_topic"].lstrip("/")

        self.assertEqual(out, REQUEST_TOPIC)
        self.assertNotEqual(out, WHEEL_TOPIC)

    def test_the_monitor_does_not_read_from_the_topic_it_writes_to(
        self,
    ) -> None:
        # In and out the same topic is a feedback loop, and in particular
        # reading cmd_vel_requested would put this node in parallel with the
        # gate rather than in front of it.
        self.assertNotEqual(
            self.params["cmd_vel_in_topic"].lstrip("/"),
            self.params["cmd_vel_out_topic"].lstrip("/"),
        )

    def test_the_monitor_does_not_read_the_wheel_topic_either(self) -> None:
        # Reading /cmd_vel would mean re-filtering the gate's own output and
        # publishing the result back upstream of it.
        self.assertNotEqual(
            self.params["cmd_vel_in_topic"].lstrip("/"), WHEEL_TOPIC
        )

    def test_the_launch_file_starts_the_monitor_and_its_manager_only(
        self,
    ) -> None:
        # Anything else started here would be a motion source hiding inside a
        # file whose whole argument is that it cannot drive.
        packages = set()
        for node in ast.walk(ast.parse(self.launch)):
            if not isinstance(node, ast.Call):
                continue
            if getattr(node.func, "id", None) not in {"Node", "LifecycleNode"}:
                continue
            for keyword in node.keywords:
                if keyword.arg == "package" and isinstance(
                    keyword.value, ast.Constant
                ):
                    packages.add(keyword.value.value)

        self.assertEqual(
            packages, {"nav2_collision_monitor", "nav2_lifecycle_manager"}
        )

    def test_the_launch_file_never_mentions_the_wheel_topic(self) -> None:
        # No remapping, no parameter, no argument in this file may name the
        # topic that reaches the motors.
        for node in ast.walk(ast.parse(self.launch)):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                self.assertNotEqual(
                    node.value.lstrip("/"),
                    WHEEL_TOPIC,
                    "collision_monitor.launch.py must not name /cmd_vel",
                )

    def test_the_safety_gate_is_not_managed_from_here(self) -> None:
        # The lifecycle manager deactivates its nodes on failure. The gate has
        # to still be running then, so it is in nobody's list - including the
        # one this file declares.
        managed = next(
            node
            for node in ast.walk(ast.parse(self.launch))
            if isinstance(node, ast.Assign)
            and any(
                getattr(target, "id", None) == "MANAGED_NODES"
                for target in node.targets
            )
        )
        names = {element.value for element in managed.value.elts}

        self.assertEqual(names, {"collision_monitor"})
        self.assertNotIn("safety_controller", names)
        self.assertNotIn("robot_safety", names)

    def test_the_monitor_is_not_started_by_any_other_launch_file(self) -> None:
        # It is deliberately not wired into navigation.launch.py yet: the
        # velocity sources still point straight at the request topic, so a
        # monitor started alongside them would sit with an empty input and
        # protect nothing while appearing in the graph. When that changes,
        # this test is the one to update, which is the point of it.
        for path in sorted(LAUNCH.glob("*.launch.py")):
            if path == LAUNCH_FILE:
                continue
            with self.subTest(launch_file=path.name):
                self.assertNotIn("nav2_collision_monitor", path.read_text())


class CollisionMonitorPackagingTests(unittest.TestCase):
    def test_the_package_declares_the_collision_monitor_dependency(
        self,
    ) -> None:
        # Without this, `rosdep install` does not fetch the package and the
        # launch file fails with an error about a missing executable.
        manifest = (SHARE / "package.xml").read_text()

        self.assertIn("nav2_collision_monitor", manifest)

    def test_the_config_and_launch_file_are_installed(self) -> None:
        # setup.py globs both directories, so this checks the globs still
        # cover these files rather than that they are listed by name.
        setup = (SHARE / "setup.py").read_text()

        self.assertIn("config/*.yaml", setup)
        self.assertIn("launch/*.launch.py", setup)


if __name__ == "__main__":
    unittest.main()
