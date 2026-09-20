"""Checks on launch wiring and parameters that do not need ROS or Gazebo.

The launch files cannot be executed here, so these tests assert the things a
typo would break quietly: the topic that reaches the wheels, the parameters
the safety gate is given, and the world the robot is dropped into.
"""

import ast
import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml

SHARE = Path(__file__).resolve().parents[1]
CONFIG = SHARE / "config"
LAUNCH = SHARE / "launch"
MAPS = SHARE / "maps"
WORLDS = SHARE / "worlds"
DESCRIPTION_URDF = SHARE.parent / "robot_description" / "urdf"
XACRO_NS = "http://www.ros.org/wiki/xacro"


def xacro_properties() -> dict[str, float]:
    """Numeric xacro properties from the URDF, read without expanding it."""
    robot = ET.parse(DESCRIPTION_URDF / "robot.urdf.xacro").getroot()
    properties = {}
    for element in robot.iter(f"{{{XACRO_NS}}}property"):
        try:
            properties[element.get("name")] = float(element.get("value"))
        except ValueError:
            continue  # A non-numeric property, such as a material name.
    return properties


def evaluate(expression: str, properties: dict[str, float]) -> float:
    """Evaluate a ${...} xacro expression against the URDF properties.

    xacro is not importable without ROS, so the alternative is to hard-code
    the sensor position in a second place and let it drift.
    """
    body = expression.strip()
    if body.startswith("${") and body.endswith("}"):
        body = body[2:-1]
    return float(eval(body, {"__builtins__": {}, "pi": math.pi}, dict(properties)))


# Every launch_ros action that starts a process which could publish a
# velocity. LifecycleNode belongs here as much as Node does: a node that can
# drive can drive whether or not it is managed, and a helper blind to it would
# report an empty set for a launch file full of them - the safety assertions
# below would then pass by finding nothing.
LAUNCH_NODE_ACTIONS = {"Node", "LifecycleNode"}


def launched_packages(source: str) -> set[str]:
    """The `package=` of every launch_ros node action in a launch file.

    Read from the syntax tree rather than by searching the text, so a package
    named in a comment or a docstring is not mistaken for a running node.
    """
    packages = set()
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) not in LAUNCH_NODE_ACTIONS:
            continue
        for keyword in node.keywords:
            if keyword.arg == "package" and isinstance(keyword.value, ast.Constant):
                packages.add(keyword.value.value)
    return packages


def origin_xyz(joint: ET.Element, properties: dict[str, float]) -> list[float]:
    """The three components of a joint origin, each possibly an expression.

    Splitting on whitespace is wrong here: a ${...} expression contains
    spaces of its own, so the components are tokenised before evaluating.
    """
    text = joint.find("origin").get("xyz")
    components = re.findall(r"\$\{[^}]*\}|\S+", text)
    if len(components) != 3:
        raise ValueError(f"expected three origin components in {text!r}")
    return [evaluate(component, properties) for component in components]


class BridgeConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = yaml.safe_load((CONFIG / "ros_gz_bridge.yaml").read_text())

    def test_cmd_vel_is_the_only_topic_driving_the_simulator(self) -> None:
        to_gz = [
            entry["ros_topic_name"]
            for entry in self.bridge
            if entry["direction"] == "ROS_TO_GZ"
        ]

        # A second ROS_TO_GZ motion topic would be a path around the safety
        # gate, which is the one thing this bridge must never allow.
        self.assertEqual(to_gz, ["/cmd_vel"])

    def test_bridges_the_sensors_the_safety_gate_and_slam_need(self) -> None:
        from_gz = {
            entry["ros_topic_name"]
            for entry in self.bridge
            if entry["direction"] == "GZ_TO_ROS"
        }

        self.assertLessEqual(
            {"/scan", "/odom", "/tf", "/clock", "/joint_states"}, from_gz
        )

    def test_every_entry_declares_both_type_names(self) -> None:
        for entry in self.bridge:
            with self.subTest(topic=entry["ros_topic_name"]):
                self.assertTrue(entry["ros_type_name"].count("/msg/"))
                self.assertTrue(entry["gz_type_name"].startswith("gz.msgs."))


class SafetyParameterTests(unittest.TestCase):
    def setUp(self) -> None:
        loaded = yaml.safe_load((CONFIG / "safety.yaml").read_text())
        self.params = loaded["safety_controller"]["ros__parameters"]

    def test_parameters_are_accepted_by_the_controller(self) -> None:
        from robot_safety import SafetyController

        controller = SafetyController(
            stop_distance=self.params["stop_distance"],
            caution_distance=self.params["caution_distance"],
            sensor_timeout=self.params["sensor_timeout"],
            command_timeout=self.params["command_timeout"],
        )

        self.assertEqual(controller.stop_distance, self.params["stop_distance"])

    def test_node_name_matches_the_safety_node(self) -> None:
        # A mismatched key means the node silently runs on its defaults.
        source = (
            SHARE.parent / "robot_safety" / "robot_safety" / "safety_node.py"
        ).read_text()

        self.assertIn('super().__init__("safety_controller")', source)

    def test_simulation_uses_the_simulator_clock(self) -> None:
        self.assertTrue(self.params["use_sim_time"])


class SafetyDistanceDerivationTests(unittest.TestCase):
    """The shipped distances must still equal what base_dynamics.yaml implies.

    stop_distance and caution_distance are the distance the robot keeps
    travelling after it sees an obstacle. Editing one of them by hand, or
    changing the base without re-deriving them, is how a margin stops being
    true without anyone noticing. These tests make that a build failure.
    """

    def setUp(self) -> None:
        from robot_safety import BaseDynamics, derive_safety_distances

        self.dynamics = BaseDynamics.from_mapping(
            yaml.safe_load((CONFIG / "base_dynamics.yaml").read_text())
        )
        self.derived = derive_safety_distances(self.dynamics)
        self.parameter_files = {
            path.name: yaml.safe_load(path.read_text())["safety_controller"][
                "ros__parameters"
            ]
            for path in sorted(CONFIG.glob("safety*.yaml"))
        }
        self.params = self.parameter_files["safety.yaml"]
        self.properties = xacro_properties()

    def test_every_parameter_file_ships_the_derived_distances(self) -> None:
        # Both teleop and navigation run the same base at the same speed, so
        # a gate that is more permissive in one of them is just a mistake.
        self.assertIn("safety_navigation.yaml", self.parameter_files)

        for name, params in self.parameter_files.items():
            with self.subTest(config=name):
                self.assertAlmostEqual(
                    params["stop_distance"], self.derived.stop_distance, places=6
                )
                self.assertAlmostEqual(
                    params["caution_distance"],
                    self.derived.caution_distance,
                    places=6,
                )

    def test_every_parameter_file_is_accepted_by_the_controller(self) -> None:
        from robot_safety import SafetyController

        for name, params in self.parameter_files.items():
            with self.subTest(config=name):
                controller = SafetyController(
                    stop_distance=params["stop_distance"],
                    caution_distance=params["caution_distance"],
                    sensor_timeout=params["sensor_timeout"],
                    command_timeout=params["command_timeout"],
                    require_localization=params["require_localization"],
                    localization_timeout=params["localization_timeout"],
                    max_localization_covariance=(
                        None
                        if params["max_localization_covariance"] < 0.0
                        else params["max_localization_covariance"]
                    ),
                    low_battery_fraction=(
                        None
                        if params["low_battery_fraction"] < 0.0
                        else params["low_battery_fraction"]
                    ),
                )

                self.assertEqual(
                    controller.stop_distance, params["stop_distance"]
                )

    def test_autonomous_navigation_requires_localization(self) -> None:
        navigation = self.parameter_files["safety_navigation.yaml"]

        # Under Nav2 the pose is what chooses where to drive, so a robot that
        # has lost it drives confidently to the wrong place.
        self.assertTrue(navigation["require_localization"])
        self.assertGreater(navigation["max_localization_covariance"], 0.0)

    def test_control_rate_matches_the_control_period_it_was_derived_from(
        self,
    ) -> None:
        # The derivation charges the robot one control period of travel. A
        # gate running slower than that moves further than the margin allows.
        self.assertAlmostEqual(
            self.params["control_rate_hz"],
            1.0 / self.dynamics.control_period,
            places=6,
        )

    def test_caution_speed_scale_matches_what_the_gate_applies(self) -> None:
        from robot_safety import SafetyController

        # The caution zone is only deep enough if the robot really does slow
        # to this fraction inside it.
        self.assertAlmostEqual(
            self.dynamics.caution_speed_scale,
            SafetyController.CAUTION_SPEED_SCALE,
            places=6,
        )

    def test_sensor_overhang_matches_the_urdf(self) -> None:
        robot = ET.parse(DESCRIPTION_URDF / "robot.urdf.xacro").getroot()
        lidar_joint = next(
            joint
            for joint in robot.iter("joint")
            if joint.get("name") == "lidar_joint"
        )
        lidar_x = origin_xyz(lidar_joint, self.properties)[0]
        front_edge = self.properties["base_length"] / 2.0

        # The gate compares raw lidar ranges, so whatever sticks out ahead of
        # the lidar is distance the robot does not actually have.
        self.assertAlmostEqual(
            self.dynamics.sensor_to_front_edge, front_edge - lidar_x, places=6
        )

    def test_speed_and_braking_match_the_drive_plugin(self) -> None:
        gazebo = ET.parse(DESCRIPTION_URDF / "robot.gazebo.xacro").getroot()
        diff_drive = next(
            plugin
            for plugin in gazebo.iter("plugin")
            if plugin.get("name") == "gz::sim::systems::DiffDrive"
        )

        # Raising the simulated speed limit without re-deriving the distances
        # would let the robot outrun its own stop distance.
        self.assertAlmostEqual(
            self.dynamics.max_speed,
            float(diff_drive.find("max_linear_velocity").text),
            places=6,
        )
        self.assertLessEqual(
            self.dynamics.max_deceleration,
            abs(float(diff_drive.find("min_linear_acceleration").text)),
        )

    def test_sensor_period_matches_the_lidar_update_rate(self) -> None:
        gazebo = ET.parse(DESCRIPTION_URDF / "robot.gazebo.xacro").getroot()
        lidar = next(
            sensor
            for sensor in gazebo.iter("sensor")
            if sensor.get("type") == "gpu_lidar"
        )

        self.assertGreaterEqual(
            self.dynamics.sensor_period,
            1.0 / float(lidar.find("update_rate").text) - 1e-9,
        )

    def test_controller_defaults_are_no_less_conservative(self) -> None:
        from robot_safety import SafetyController

        # These are what a node started without a parameter file falls back
        # to, so they must never be the shorter of the two.
        fallback = SafetyController()

        self.assertGreaterEqual(fallback.stop_distance, self.derived.stop_distance)
        self.assertGreaterEqual(
            fallback.caution_distance, self.derived.caution_distance
        )

    def test_inputs_are_still_declared_unmeasured(self) -> None:
        # Fails the day someone sets measured: true, which is the moment the
        # placeholder URDF and these assumptions must be revisited together.
        self.assertFalse(
            self.dynamics.measured,
            "base_dynamics.yaml claims measured inputs: confirm the URDF "
            "geometry is measured too and update this test",
        )


class LaunchFileTests(unittest.TestCase):
    def setUp(self) -> None:
        self.simulation = (LAUNCH / "simulation.launch.py").read_text()
        self.teleop = (LAUNCH / "teleop.launch.py").read_text()

    def test_launch_files_are_valid_python(self) -> None:
        for path in sorted(LAUNCH.glob("*.launch.py")):
            with self.subTest(path=path.name):
                compile(path.read_text(), str(path), "exec")

    def test_simulation_starts_the_safety_gate(self) -> None:
        self.assertIn('package="robot_safety"', self.simulation)
        self.assertIn('executable="safety_node"', self.simulation)

    def test_teleop_publishes_requests_not_gated_motion(self) -> None:
        # Teleop must go through the gate like every other motion source.
        self.assertIn('("/cmd_vel", "/cmd_vel_requested")', self.teleop)


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    """The width, height and pixels of a binary (P5) portable greymap."""
    data = path.read_bytes()
    fields: list[bytes] = []
    offset = 0
    while len(fields) < 4:
        while data[offset : offset + 1].isspace():
            offset += 1
        if data[offset : offset + 1] == b"#":  # A comment runs to end of line.
            offset = data.index(b"\n", offset) + 1
            continue
        end = offset
        while not data[end : end + 1].isspace():
            end += 1
        fields.append(data[offset:end])
        offset = end
    width, height = int(fields[1]), int(fields[2])
    return width, height, data[offset + 1 :]


class MapTests(unittest.TestCase):
    """The map navigation.launch.py defaults to has to exist and be loadable.

    Without a committed map the default is a path to nothing: Nav2 comes up,
    map_server fails to activate, and the whole lifecycle group stays down.
    """

    def setUp(self) -> None:
        self.yaml_path = MAPS / "test_room.yaml"
        self.map = yaml.safe_load(self.yaml_path.read_text())
        self.image = MAPS / self.map["image"]
        self.width, self.height, self.pixels = read_pgm(self.image)

    def test_the_committed_map_and_its_image_are_both_present(self) -> None:
        self.assertTrue(self.yaml_path.is_file())
        self.assertTrue(self.image.is_file())

        # A bare filename, so the pair can be copied or installed together.
        self.assertEqual(self.map["image"], self.image.name)

    def test_the_map_declares_what_map_server_needs(self) -> None:
        self.assertLessEqual(
            {"image", "resolution", "origin", "negate", "occupied_thresh",
             "free_thresh"},
            set(self.map),
        )
        self.assertGreater(self.map["resolution"], 0.0)
        self.assertEqual(len(self.map["origin"]), 3)
        self.assertLess(self.map["free_thresh"], self.map["occupied_thresh"])

    def test_the_image_holds_the_pixels_the_header_promises(self) -> None:
        self.assertGreaterEqual(len(self.pixels), self.width * self.height)

    def test_the_map_covers_the_world_the_robot_is_spawned_into(self) -> None:
        world = ET.parse(WORLDS / "test_room.sdf").getroot()
        walls = next(
            model for model in world.iter("model") if model.get("name") == "walls"
        )
        extents = []
        for collision in walls.iter("collision"):
            pose = [float(part) for part in collision.find("pose").text.split()]
            size = [
                float(part)
                for part in collision.find("geometry/box/size").text.split()
            ]
            extents.append((pose[0], pose[1], size[0], size[1]))

        origin_x, origin_y = self.map["origin"][0], self.map["origin"][1]
        far_x = origin_x + self.width * self.map["resolution"]
        far_y = origin_y + self.height * self.map["resolution"]

        # Every wall has to be inside the image, or the room the robot drives
        # in is partly off the edge of the map it is localizing against.
        for x, y, length, width in extents:
            with self.subTest(wall=(x, y)):
                self.assertLessEqual(origin_x, x - length / 2.0)
                self.assertGreaterEqual(far_x, x + length / 2.0)
                self.assertLessEqual(origin_y, y - width / 2.0)
                self.assertGreaterEqual(far_y, y + width / 2.0)

    def test_the_pose_amcl_starts_at_is_free_space_on_this_map(self) -> None:
        amcl = yaml.safe_load((CONFIG / "nav2.yaml").read_text())["amcl"]
        pose = amcl["ros__parameters"]["initial_pose"]

        resolution = self.map["resolution"]
        column = int((pose["x"] - self.map["origin"][0]) / resolution)
        row = int((pose["y"] - self.map["origin"][1]) / resolution)
        # Row 0 of a .pgm is the top of the image, which is the highest y.
        pixel = self.pixels[(self.height - 1 - row) * self.width + column]

        # map_server reads a pixel as occupancy (255 - pixel) / 255 when
        # negate is 0, and calls anything below free_thresh free.
        self.assertLess(
            (255 - pixel) / 255.0,
            self.map["free_thresh"],
            "AMCL starts the robot inside an obstacle or in unknown space, so "
            "the first goal will be rejected before anything moves",
        )

    def test_amcl_starts_where_the_simulation_spawns_the_robot(self) -> None:
        simulation = (LAUNCH / "simulation.launch.py").read_text()
        spawn = {
            axis: float(
                re.search(rf'"-{axis}",\s*"(-?[\d.]+)"', simulation).group(1)
            )
            for axis in ("x", "y")
        }
        amcl = yaml.safe_load((CONFIG / "nav2.yaml").read_text())["amcl"]
        pose = amcl["ros__parameters"]["initial_pose"]

        # This map is rendered from the world file, so map coordinates are
        # world coordinates and AMCL can be told exactly where the robot is.
        # Move the spawn without moving this and navigation comes up believing
        # the robot is somewhere it is not.
        self.assertAlmostEqual(pose["x"], spawn["x"])
        self.assertAlmostEqual(pose["y"], spawn["y"])


class MapInstallTests(unittest.TestCase):
    """A committed map that is not installed is still a path to nothing.

    navigation.launch.py resolves its default through the package share
    directory, which only contains what setup.py lists.
    """

    def setUp(self) -> None:
        self.setup = (SHARE / "setup.py").read_text()
        self.navigation = (LAUNCH / "navigation.launch.py").read_text()

    def test_setup_installs_the_map_and_its_image(self) -> None:
        self.assertIn('glob("maps/*.yaml")', self.setup)
        self.assertIn('glob("maps/*.pgm")', self.setup)

    def test_the_default_map_is_the_one_that_is_installed(self) -> None:
        self.assertIn('"maps" / "test_room.yaml"', self.navigation)
        self.assertTrue((MAPS / "test_room.yaml").is_file())


class MapGeneratorTests(unittest.TestCase):
    """The world-to-map fallback, which is what produced the committed map.

    It is not SLAM and does not pretend to be; it exists so navigation has a
    map before a simulator run can be completed. These tests keep it honest
    about the world it claims to render.
    """

    def setUp(self) -> None:
        from robot_bringup.world_to_map import render

        self.grid = render(WORLDS / "test_room.sdf")

    def cell(self, x: float, y: float) -> int:
        column = int((x - self.grid.origin[0]) / self.grid.resolution)
        row = int((y - self.grid.origin[1]) / self.grid.resolution)
        return self.grid.cells[row][column]

    def test_obstacles_in_the_world_are_occupied_on_the_map(self) -> None:
        from robot_bringup.world_to_map import OCCUPIED

        self.assertEqual(self.cell(-1.2, 1.0), OCCUPIED, "the table")
        self.assertEqual(self.cell(1.6, -1.4), OCCUPIED, "the cabinet")
        self.assertEqual(self.cell(0.0, 2.5), OCCUPIED, "the north wall")

    def test_the_open_floor_is_free(self) -> None:
        from robot_bringup.world_to_map import FREE

        self.assertEqual(self.cell(-2.0, -1.5), FREE, "where the robot spawns")
        self.assertEqual(self.cell(0.0, 0.0), FREE, "the middle of the room")

    def test_outside_the_room_is_unknown_rather_than_free(self) -> None:
        from robot_bringup.world_to_map import UNKNOWN

        # The doorway in the east wall opens onto space no scan ever saw.
        # Mapping it as free would invite the planner to route through it.
        self.assertEqual(self.cell(3.2, 0.0), UNKNOWN)

    def test_rendering_the_same_world_twice_gives_the_same_map(self) -> None:
        from robot_bringup.world_to_map import render

        self.assertEqual(self.grid.pgm(), render(WORLDS / "test_room.sdf").pgm())

    def test_the_committed_map_is_what_this_world_renders_to(self) -> None:
        # If the world gains a wall, the committed map is stale, and the
        # robot will localize against a room that no longer exists.
        self.assertEqual(self.grid.pgm(), (MAPS / "test_room.pgm").read_bytes())


class Nav2GeometryTests(unittest.TestCase):
    """Nav2's idea of the robot's shape must match the robot's actual shape.

    The footprint and inflation radius are the URDF restated in another file.
    Left hand-written, a base that grows 10 cm keeps planning through gaps it
    no longer fits.
    """

    def setUp(self) -> None:
        from robot_navigation import BaseFootprint

        self.nav2 = yaml.safe_load((CONFIG / "nav2.yaml").read_text())
        self.properties = xacro_properties()
        self.footprint = BaseFootprint(
            length=self.properties["base_length"],
            width=self.properties["base_width"],
            clearance_margin=0.25,
        )
        self.costmaps = {
            "local_costmap": self.nav2["local_costmap"]["local_costmap"][
                "ros__parameters"
            ],
            "global_costmap": self.nav2["global_costmap"]["global_costmap"][
                "ros__parameters"
            ],
        }

    def test_both_costmaps_use_the_urdf_footprint(self) -> None:
        expected = self.footprint.polygon

        for name, costmap in self.costmaps.items():
            with self.subTest(costmap=name):
                corners = [
                    tuple(corner)
                    for corner in ast.literal_eval(costmap["footprint"])
                ]

                self.assertEqual(len(corners), len(expected))
                for actual, wanted in zip(corners, expected, strict=True):
                    self.assertAlmostEqual(actual[0], wanted[0], places=6)
                    self.assertAlmostEqual(actual[1], wanted[1], places=6)

    def test_inflation_covers_the_circumscribed_radius(self) -> None:
        # Inside the circumscribed radius the planner can produce a path the
        # robot cannot even rotate on the spot to follow.
        for name, costmap in self.costmaps.items():
            with self.subTest(costmap=name):
                inflation = costmap["inflation_layer"]["inflation_radius"]

                self.assertGreaterEqual(
                    inflation, self.footprint.circumscribed_radius
                )
                self.assertAlmostEqual(
                    inflation, self.footprint.inflation_radius, places=6
                )

    def test_both_costmaps_agree(self) -> None:
        # One costmap planning a route the other will not follow produces a
        # robot that oscillates in doorways.
        local = self.costmaps["local_costmap"]["inflation_layer"]
        global_ = self.costmaps["global_costmap"]["inflation_layer"]

        self.assertEqual(local["inflation_radius"], global_["inflation_radius"])

    def test_controller_speed_matches_the_derived_dynamics(self) -> None:
        from robot_safety import BaseDynamics

        dynamics = BaseDynamics.from_mapping(
            yaml.safe_load((CONFIG / "base_dynamics.yaml").read_text())
        )
        follow_path = self.nav2["controller_server"]["ros__parameters"]["FollowPath"]

        # The stop distance was derived for this speed. A controller allowed
        # to drive faster than base_dynamics assumes outruns its own margin.
        self.assertLessEqual(follow_path["max_vel_x"], dynamics.max_speed)
        self.assertLessEqual(follow_path["max_speed_xy"], dynamics.max_speed)

        # And it must be able to brake at least as hard as the derivation
        # assumed, or the braking distance is optimistic.
        self.assertGreaterEqual(
            abs(follow_path["decel_lim_x"]), dynamics.max_deceleration
        )

    def test_amcl_publishes_faster_than_the_gate_gives_up_on_it(self) -> None:
        navigation = yaml.safe_load(
            (CONFIG / "safety_navigation.yaml").read_text()
        )["safety_controller"]["ros__parameters"]

        # AMCL's transform tolerance is the slack it allows itself; the gate
        # stopping sooner than that would stop a healthy robot.
        self.assertGreaterEqual(
            navigation["localization_timeout"],
            self.nav2["amcl"]["ros__parameters"]["transform_tolerance"],
        )

    def test_amcl_laser_range_matches_the_lidar(self) -> None:
        gazebo = ET.parse(DESCRIPTION_URDF / "robot.gazebo.xacro").getroot()
        lidar = next(
            sensor
            for sensor in gazebo.iter("sensor")
            if sensor.get("type") == "gpu_lidar"
        )
        amcl = self.nav2["amcl"]["ros__parameters"]

        self.assertLessEqual(
            amcl["laser_max_range"], float(lidar.find(".//range/max").text)
        )
        self.assertGreaterEqual(
            amcl["laser_min_range"], float(lidar.find(".//range/min").text)
        )

    def test_frames_agree_across_every_nav2_component(self) -> None:
        robot = ET.parse(DESCRIPTION_URDF / "robot.urdf.xacro").getroot()
        links = {link.get("name") for link in robot.iter("link")}
        base_frames = {
            self.nav2["amcl"]["ros__parameters"]["base_frame_id"],
            self.nav2["bt_navigator"]["ros__parameters"]["robot_base_frame"],
            self.nav2["behavior_server"]["ros__parameters"]["robot_base_frame"],
            self.costmaps["local_costmap"]["robot_base_frame"],
            self.costmaps["global_costmap"]["robot_base_frame"],
        }

        # A component quietly using a different base frame plans for a robot
        # offset from the real one.
        self.assertEqual(len(base_frames), 1)
        self.assertLessEqual(base_frames, links)


class NavigationTopologyTests(unittest.TestCase):
    """Nav2 is a motion source, so it has to sit behind the gate like any other."""

    def setUp(self) -> None:
        self.navigation = (LAUNCH / "navigation.launch.py").read_text()
        self.slam = (LAUNCH / "slam.launch.py").read_text()

    def test_every_velocity_emitting_node_is_remapped_to_the_request_topic(
        self,
    ) -> None:
        import ast as _ast

        tree = _ast.parse(self.navigation)
        sources = next(
            node
            for node in _ast.walk(tree)
            if isinstance(node, _ast.Assign)
            and any(
                getattr(target, "id", None) == "VELOCITY_SOURCES"
                for target in node.targets
            )
        )
        declared = {element.value for element in sources.value.elts}

        # controller_server drives during a goal; behavior_server drives
        # during a recovery, which is when things have already gone wrong.
        self.assertLessEqual({"controller_server", "behavior_server"}, declared)

        for name in declared:
            with self.subTest(node=name):
                node_block = self.navigation.split(f'name="{name}"')[1]
                node_block = node_block.split("Node(")[0]

                self.assertIn("remappings=gated", node_block)

    def test_the_remap_points_at_the_request_topic(self) -> None:
        self.assertIn('gated = [("/cmd_vel", "/cmd_vel_requested")]', self.navigation)

    def test_navigation_runs_the_gate_with_the_localization_checks(self) -> None:
        # safety.yaml has require_localization false, because teleop has no
        # pose to lose. Loading it here would silently drop the check.
        self.assertIn("safety_navigation.yaml", self.navigation)
        self.assertNotIn('"safety.yaml"', self.navigation)
        self.assertIn('package="robot_safety"', self.navigation)

    def test_the_gate_is_not_under_lifecycle_control(self) -> None:
        import ast as _ast

        tree = _ast.parse(self.navigation)
        managed = next(
            node
            for node in _ast.walk(tree)
            if isinstance(node, _ast.Assign)
            and any(
                getattr(target, "id", None) == "MANAGED_NODES"
                for target in node.targets
            )
        )
        names = {element.value for element in managed.value.elts}

        # The lifecycle manager deactivates its nodes on shutdown and on
        # failure. The gate is the thing that has to still be running then.
        self.assertNotIn("safety_controller", names)
        self.assertNotIn("robot_safety", names)

    def test_the_gate_is_not_under_lifecycle_control_while_mapping_either(
        self,
    ) -> None:
        import ast as _ast

        tree = _ast.parse(self.navigation)
        managed = next(
            node
            for node in _ast.walk(tree)
            if isinstance(node, _ast.Assign)
            and any(
                getattr(target, "id", None) == "SLAM_MANAGED_NODES"
                for target in node.targets
            )
        )

        # The slam list is derived from MANAGED_NODES rather than written out
        # again, so the gate cannot be added to one list and not the other.
        self.assertIn("MANAGED_NODES", _ast.dump(managed.value))
        self.assertNotIn("safety_controller", _ast.dump(managed.value))

    def test_mapping_mode_drops_the_nodes_that_need_a_saved_map(self) -> None:
        # map_server and amcl have nothing to do while slam_toolbox owns
        # map->odom, and a lifecycle manager waiting on nodes that were never
        # started never activates the ones that were.
        self.assertIn('LOCALIZATION_NODES = ["map_server", "amcl"]', self.navigation)
        self.assertIn('"slam"', self.navigation)

    def test_navigation_has_a_map_to_fall_back_on(self) -> None:
        # Without a default, `ros2 launch robot_bringup navigation.launch.py`
        # is an error rather than a run.
        self.assertIn('default_value=default_map', self.navigation)

    def test_mapping_launches_nothing_that_can_drive(self) -> None:
        # Mapping is supervised and manual: the operator drives with teleop
        # and watches the map form. Starting a planner here would mean the
        # robot could move while the map it is moving through is half built.
        packages = launched_packages(self.slam)

        self.assertEqual(packages, {"slam_toolbox"})
        self.assertNotIn("cmd_vel", self.slam)


class WorldTests(unittest.TestCase):
    def setUp(self) -> None:
        self.world = ET.parse(WORLDS / "test_room.sdf").getroot()

    def test_world_is_well_formed_and_named(self) -> None:
        self.assertEqual(self.world.find("world").get("name"), "test_room")

    def test_world_loads_the_systems_the_sensors_need(self) -> None:
        plugins = {plugin.get("name") for plugin in self.world.iter("plugin")}

        # Without the Sensors system the lidar never publishes, and the
        # safety gate would fail closed on a world configuration mistake.
        self.assertIn("gz::sim::systems::Sensors", plugins)
        self.assertIn("gz::sim::systems::Physics", plugins)
        self.assertIn("gz::sim::systems::Imu", plugins)

    def test_world_contains_obstacles_to_stop_for(self) -> None:
        models = {model.get("name") for model in self.world.iter("model")}

        self.assertIn("walls", models)
        self.assertTrue({"table", "cabinet"} <= models)


def safety_gate_nodes(source: str) -> list[ast.Call]:
    """Every launch_ros node action in `source` that starts the safety gate."""
    return [
        node
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) in LAUNCH_NODE_ACTIONS
        and any(
            keyword.arg == "executable"
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == "safety_node"
            for keyword in node.keywords
        )
    ]


class SingleSafetyGateTests(unittest.TestCase):
    """Only one gate may publish /cmd_vel, including across launch files.

    simulation.launch.py and navigation.launch.py are documented as being run
    together, and each starts a gate. Run at once they both subscribe to
    /cmd_vel_requested and both publish /cmd_vel, so the simulation gate -
    which has no localization or battery checks - keeps commanding motion
    while the navigation gate is trying to stop. Observed on a live run:
    `ros2 topic info /cmd_vel` reported two publishers, both safety_controller.
    """

    def setUp(self) -> None:
        self.simulation = (LAUNCH / "simulation.launch.py").read_text()
        self.navigation = (LAUNCH / "navigation.launch.py").read_text()

    def test_each_launch_file_starts_at_most_one_gate(self) -> None:
        self.assertEqual(len(safety_gate_nodes(self.simulation)), 1)
        self.assertEqual(len(safety_gate_nodes(self.navigation)), 1)

    def test_the_simulation_gate_can_be_turned_off(self) -> None:
        gate = safety_gate_nodes(self.simulation)[0]
        conditions = [k for k in gate.keywords if k.arg == "condition"]

        # Unconditional here means the two launch files cannot be run together
        # without stacking two gates on /cmd_vel.
        self.assertEqual(
            len(conditions),
            1,
            "the simulation gate must be conditional so navigation.launch.py "
            "can supply the gate instead",
        )

    def test_the_simulation_declares_the_argument_that_turns_it_off(self) -> None:
        self.assertIn('"safety"', self.simulation)

    def test_the_navigation_gate_is_unconditional(self) -> None:
        gate = safety_gate_nodes(self.navigation)[0]

        # Nav2 is a motion source; it must never run without its gate.
        self.assertEqual([k for k in gate.keywords if k.arg == "condition"], [])


if __name__ == "__main__":
    unittest.main()
