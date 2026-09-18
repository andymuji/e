import math
from pathlib import Path
import tempfile
import unittest

try:
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import rclpy
    from rclpy.parameter import Parameter
    from robot_core import Pose2D
    from robot_locations import LocationStore
    from robot_voice.voice_node import VoiceNode
    from std_msgs.msg import String

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class PublishedMessages:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeGoalHandle:
    def __init__(self, accepted: bool) -> None:
        self.accepted = accepted
        self.cancelled = False

    def cancel_goal_async(self) -> None:
        self.cancelled = True


class FakeGoalFuture:
    """send_goal_async's future, with no executor to spin it.

    Nav2 accepts a goal some time after it is sent, so the tests decide when
    that answer arrives and what the node is told in the meantime.
    """

    def __init__(self, goal_handle: FakeGoalHandle) -> None:
        self.goal_handle = goal_handle
        self._callback = None

    def add_done_callback(self, callback) -> None:
        self._callback = callback

    def result(self) -> FakeGoalHandle:
        return self.goal_handle

    def answer(self) -> None:
        self._callback(self)


class FakeNavigator:
    """The Nav2 action client, minus Nav2."""

    def __init__(self) -> None:
        self.ready = True
        self.accepted = True
        self.sent = []
        self.futures = []

    def server_is_ready(self) -> bool:
        return self.ready

    def send_goal_async(self, goal) -> FakeGoalFuture:
        self.sent.append(goal)
        future = FakeGoalFuture(FakeGoalHandle(self.accepted))
        self.futures.append(future)
        return future

    def goal_handle(self, index: int = 0) -> FakeGoalHandle:
        return self.futures[index].goal_handle


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 Python dependencies are unavailable")
class VoiceNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.locations_file = Path(self.directory.name) / "locations.json"
        store = LocationStore(self.locations_file)
        store.save_location("kitchen", Pose2D(1.0, 2.0))
        store.save_location("living room", Pose2D(3.0, 4.0, math.pi / 2.0))

        self.node = self.build_node()
        self.navigator = self.node._navigator
        self.responses = self.node._response_publisher
        self.emergency_stop = self.node._emergency_stop_publisher

    def build_node(self, **parameters):
        """A node with the Nav2 client and both publishers swapped for fakes."""
        overrides = {"locations_file": str(self.locations_file), **parameters}
        node = VoiceNode(
            parameter_overrides=[
                Parameter(name, value=value) for name, value in overrides.items()
            ]
        )
        self.addCleanup(node.destroy_node)
        # Node.destroy_node leaves waitables alone, and a live action client
        # keeps the node handle in use, so retire the real one before the
        # fake takes its place.
        node._navigator.destroy()
        node._navigator = FakeNavigator()
        node._response_publisher = PublishedMessages()
        node._emergency_stop_publisher = PublishedMessages()
        return node

    def speak(self, transcript: str, node=None) -> None:
        (node or self.node)._on_transcript(String(data=transcript))

    def last_response(self) -> str:
        return self.responses.messages[-1].data

    def emergency_stops(self) -> list:
        return [message.data for message in self.emergency_stop.messages]

    def position(self, x: float, y: float):
        message = PoseWithCovarianceStamped()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        return message

    def test_approved_destination_becomes_a_nav2_goal(self) -> None:
        self.speak("Go to the kitchen")

        self.assertEqual(len(self.navigator.sent), 1)
        self.assertEqual(self.last_response(), "Going to the kitchen.")

    def test_the_goal_carries_the_approved_pose_in_the_map_frame(self) -> None:
        self.speak("navigate to the living room")

        pose = self.navigator.sent[0].pose

        self.assertEqual(pose.header.frame_id, "map")
        self.assertAlmostEqual(pose.pose.position.x, 3.0)
        self.assertAlmostEqual(pose.pose.position.y, 4.0)
        # A quarter turn, as a quaternion about z alone.
        self.assertAlmostEqual(pose.pose.orientation.z, math.sin(math.pi / 4.0))
        self.assertAlmostEqual(pose.pose.orientation.w, math.cos(math.pi / 4.0))

    def test_unapproved_destination_produces_no_goal(self) -> None:
        self.speak("go to the moon")

        self.assertEqual(self.navigator.sent, [])
        self.assertIn("I do not know a place called moon", self.last_response())

    def test_unparseable_speech_produces_no_goal(self) -> None:
        for transcript in ("find my keys", "", "kitchen", "take me"):
            with self.subTest(transcript=transcript):
                self.speak(transcript)

                self.assertEqual(self.navigator.sent, [])
                self.assertIn("I did not understand that", self.last_response())

    def test_loose_destination_phrases_produce_no_goal(self) -> None:
        # The parser reads these as destinations, and the gateway then finds
        # no such approved place. Either way nothing drives.
        for transcript in ("go to sleep", "go away", "navigate outside"):
            with self.subTest(transcript=transcript):
                self.speak(transcript)

                self.assertEqual(self.navigator.sent, [])
                self.assertIn("I do not know a place called", self.last_response())

    def test_the_node_publishes_nothing_that_can_drive(self) -> None:
        topics = {publisher.topic_name for publisher in self.node._publishers}

        # The node asks Nav2 for a trip. It has no velocity topic of its own,
        # gated or otherwise, and no way to release the gate's latch.
        self.assertEqual([topic for topic in topics if "cmd_vel" in topic], [])
        self.assertEqual([topic for topic in topics if "reset" in topic], [])
        self.assertIn("/emergency_stop", topics)

    def test_stop_latches_the_emergency_stop_and_cancels_the_trip(self) -> None:
        self.speak("go to the kitchen")
        self.navigator.futures[0].answer()

        self.speak("robot please stop now")

        self.assertEqual(self.emergency_stops(), [True])
        self.assertTrue(self.navigator.goal_handle().cancelled)
        self.assertIsNone(self.node._goal_handle)
        self.assertEqual(self.last_response(), "Stopping now.")

    def test_stop_in_the_same_breath_as_a_destination_sends_no_goal(self) -> None:
        self.speak("go to the kitchen no stop")

        self.assertEqual(self.navigator.sent, [])
        self.assertEqual(self.emergency_stops(), [True])

    def test_a_stop_while_nav2_is_still_accepting_still_cancels(self) -> None:
        self.speak("go to the kitchen")
        self.speak("stop")

        # Nav2 accepts the goal only now, after the stop was heard.
        self.navigator.futures[0].answer()

        self.assertTrue(self.navigator.goal_handle().cancelled)
        self.assertIsNone(self.node._goal_handle)

    def test_the_latch_can_be_left_to_the_operator_console(self) -> None:
        node = self.build_node(stop_engages_emergency_stop=False)

        self.speak("stop", node)

        self.assertEqual(node._emergency_stop_publisher.messages, [])

    def test_no_goal_is_sent_while_navigation_is_not_running(self) -> None:
        self.navigator.ready = False

        self.speak("go to the kitchen")

        self.assertEqual(self.navigator.sent, [])
        self.assertIn("Navigation is not running", self.last_response())

    def test_a_goal_nav2_refuses_is_said_out_loud(self) -> None:
        self.navigator.accepted = False

        self.speak("go to the kitchen")
        self.navigator.futures[0].answer()

        self.assertIsNone(self.node._goal_handle)
        self.assertEqual(self.last_response(), "Navigation would not take that trip.")

    def test_a_destination_approved_after_start_up_is_accepted(self) -> None:
        # The operator adds a destination in the console while the node runs.
        LocationStore(self.locations_file).save_location("bathroom", Pose2D(5.0, 6.0))

        self.speak("go to the bathroom")

        self.assertEqual(len(self.navigator.sent), 1)

    def test_a_destination_withdrawn_after_start_up_is_refused(self) -> None:
        LocationStore(self.locations_file).remove_location("kitchen")

        self.speak("go to the kitchen")

        self.assertEqual(self.navigator.sent, [])
        self.assertIn("I do not know a place called kitchen", self.last_response())

    def test_a_corrupt_location_file_keeps_the_approved_destinations(self) -> None:
        self.locations_file.write_text("{ not json")

        self.speak("go to the kitchen")

        # Keeping the list the operator already approved is safe; so would
        # refusing everything be. Taking the node down would not be.
        self.assertEqual(len(self.navigator.sent), 1)

    def test_configured_home_goes_to_its_approved_location(self) -> None:
        node = self.build_node(home_location="Living Room")

        self.speak("go home", node)

        self.assertEqual(len(node._navigator.sent), 1)

    def test_a_home_nobody_approved_stops_the_node_from_starting(self) -> None:
        with self.assertRaises(ValueError):
            self.build_node(home_location="garage")

    def test_the_approved_destination_file_is_required(self) -> None:
        with self.assertRaises(ValueError):
            self.build_node(locations_file="")

    def test_report_location_names_the_nearest_approved_place(self) -> None:
        self.node._on_position(self.position(1.1, 2.0))

        self.speak("where are you")

        self.assertEqual(self.navigator.sent, [])
        self.assertEqual(self.last_response(), "I am near the kitchen.")

    def test_report_location_admits_it_when_the_pose_is_unknown(self) -> None:
        self.speak("where are you")

        self.assertEqual(self.last_response(), "I do not know where I am yet.")

    def test_report_location_does_not_claim_a_place_it_is_far_from(self) -> None:
        self.node._on_position(self.position(20.0, 20.0))

        self.speak("report location")

        self.assertEqual(
            self.last_response(), "I am not near any of the places I know."
        )


if __name__ == "__main__":
    unittest.main()
