import ast
import os
from pathlib import Path
import sys
import tempfile
import unittest

try:
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    import rclpy
    from robot_locations.location_node import LocationNode
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class PublishedMessages:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class LocationNodeFixture(unittest.TestCase):
    """A node whose store is a throwaway file, with its publishers captured."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        # The node resolves its default store under XDG_STATE_HOME, so
        # pointing that at a temporary directory keeps the test off both the
        # developer's real store and the committed web_ui fixture.
        self._directory = tempfile.TemporaryDirectory()
        self._previous_state_home = os.environ.get("XDG_STATE_HOME")
        os.environ["XDG_STATE_HOME"] = self._directory.name

        self.node = self.make_node()

    def tearDown(self) -> None:
        self.node.destroy_node()
        if self._previous_state_home is None:
            os.environ.pop("XDG_STATE_HOME", None)
        else:
            os.environ["XDG_STATE_HOME"] = self._previous_state_home
        self._directory.cleanup()

    def make_node(self) -> "LocationNode":
        node = LocationNode()
        self.goals = PublishedMessages()
        self.statuses = PublishedMessages()
        node._goal_publisher = self.goals
        node._status_publisher = self.statuses
        return node

    def at_pose(self, x: float, y: float) -> None:
        """Tell the node where the robot is, as AMCL would."""
        message = PoseWithCovarianceStamped()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        self.node._on_pose(message)

    def save(self, name: str) -> None:
        self.node._on_save(String(data=name))

    def recall(self, name: str) -> None:
        self.node._on_recall(String(data=name))

    def last_status(self) -> str:
        return self.statuses.messages[-1].data


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class LocationNodeTests(LocationNodeFixture):
    def test_saving_then_recalling_publishes_the_pose_as_a_goal(self) -> None:
        self.at_pose(4.0, 4.0)
        self.save(" Kitchen ")

        # The robot has since driven away: the recall must send the pose that
        # was saved, not wherever the robot happens to be standing now.
        self.at_pose(-1.0, 0.5)
        self.recall("kitchen")

        self.assertEqual(len(self.goals.messages), 1)
        goal = self.goals.messages[-1]
        self.assertAlmostEqual(goal.pose.position.x, 4.0)
        self.assertAlmostEqual(goal.pose.position.y, 4.0)
        self.assertEqual(goal.header.frame_id, "map")

    def test_recalling_an_unknown_place_publishes_no_goal(self) -> None:
        self.at_pose(1.0, 1.0)
        self.recall("sofa")

        # Nothing reaches the navigator, and the refusal is reported rather
        # than raised: the next command still has to be answered.
        self.assertEqual(self.goals.messages, [])
        self.assertIn("sofa", self.last_status())
        self.assertIn("unknown location", self.last_status())

    def test_a_name_the_store_rejects_is_refused_not_raised(self) -> None:
        self.at_pose(1.0, 1.0)
        self.save("   ")

        self.assertEqual(self.node._store.names(), ())
        self.assertIn("refused to save", self.last_status())

    def test_saving_without_a_known_pose_is_refused(self) -> None:
        # No AMCL pose has arrived, so the robot does not know where it is
        # and must not record a guess under the operator's chosen name.
        self.save("kitchen")

        self.assertEqual(self.node._store.names(), ())
        self.assertIn("does not have a current pose", self.last_status())

    def test_a_stale_pose_is_not_saved(self) -> None:
        self.at_pose(4.0, 4.0)
        self.node._last_pose_time = self.node._now() - 30.0
        self.save("kitchen")

        self.assertEqual(self.node._store.names(), ())
        self.assertIn("does not have a current pose", self.last_status())

    def test_saved_places_are_listed_by_the_service(self) -> None:
        self.at_pose(4.0, 4.0)
        self.save("kitchen")
        self.at_pose(1.0, 1.0)
        self.save("bedroom")

        response = self.node._on_list_locations(Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success)
        self.assertEqual(response.message, "bedroom, kitchen")

    def test_saved_places_persist_for_the_next_run(self) -> None:
        self.at_pose(2.0, 3.0)
        self.save("living room")
        first_store_path = self.node._store_path

        self.node.destroy_node()
        self.node = self.make_node()

        self.assertEqual(self.node._store_path, first_store_path)
        self.assertEqual(self.node._store.names(), ("living room",))

    def test_the_store_is_not_the_committed_web_ui_fixture(self) -> None:
        # The web console rewrites that file wholesale from its own cached
        # copy, so a second writer would silently drop its places.
        self.assertNotIn("web_ui", str(self.node._store_path))
        self.assertIn(Path(self._directory.name), self.node._store_path.parents)


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class LocationNodeMotionTests(LocationNodeFixture):
    """This node asks for motion; it must have no way to command it."""

    def test_the_node_has_no_velocity_publisher(self) -> None:
        node = LocationNode()
        try:
            published_types = {publisher.msg_type for publisher in node.publishers}
        finally:
            node.destroy_node()

        self.assertNotIn(Twist, published_types)
        self.assertIn(PoseStamped, published_types)

    def test_no_publisher_addresses_the_gated_velocity_topic(self) -> None:
        node = LocationNode()
        try:
            topics = {publisher.topic_name for publisher in node.publishers}
        finally:
            node.destroy_node()

        # Only robot_safety publishes /cmd_vel, and a request for motion goes
        # to Nav2 as a goal rather than to /cmd_vel_requested as a velocity.
        self.assertNotIn("/cmd_vel", topics)
        self.assertNotIn("/cmd_vel_requested", topics)
        self.assertIn("/goal_pose", topics)

    def test_no_twist_is_published_by_any_command(self) -> None:
        self.at_pose(4.0, 4.0)
        self.save("kitchen")
        self.recall("kitchen")
        self.recall("sofa")
        self.save("   ")

        published = self.goals.messages + self.statuses.messages

        self.assertTrue(published)
        for message in published:
            with self.subTest(message=type(message).__name__):
                self.assertNotIsInstance(message, Twist)
                self.assertIsInstance(message, PoseStamped | String)

    def test_the_module_never_imports_a_velocity_type(self) -> None:
        source = Path(sys.modules[LocationNode.__module__].__file__).read_text()
        imported = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.ImportFrom):
                imported.update(alias.asname or alias.name for alias in node.names)
            elif isinstance(node, ast.Import):
                imported.update(alias.asname or alias.name for alias in node.names)

        # Importing a velocity type here would be the first step of a node
        # that drives the robot itself, bypassing both Nav2 and the gate.
        # Read from the syntax tree, not the text, so the safety note in the
        # module docstring is not mistaken for the import it warns against.
        self.assertNotIn("Twist", imported)
        self.assertIn("PoseStamped", imported)


if __name__ == "__main__":
    unittest.main()
