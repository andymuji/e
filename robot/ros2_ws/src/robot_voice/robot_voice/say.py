"""Type what you would have said, and publish it on speech_transcript.

SAFETY: this tool publishes one topic, speech_transcript, and nothing else.
It never publishes a velocity, never publishes emergency_stop, and never
publishes emergency_stop_reset: releasing a latched stop stays an operator
action at the gate, not something a transcript can ask for. What a transcript
is allowed to cause is still CommandGateway's decision.

It is the sanctioned stand-in for `ros2 topic pub`, which repository policy
denies, and the demonstrable half of the voice path until ADR 0002 picks a
speech engine:

    ros2 run robot_voice say "go to the kitchen"   # one-shot
    ros2 run robot_voice say                       # interactive, Ctrl-D ends

A publisher that exits the moment it has published loses the message, which
from the outside looks exactly like a robot ignoring you. So this waits for a
subscriber to appear before publishing, and settles afterwards before shutting
down; with nobody listening it says so and exits non-zero rather than
pretending the robot heard.
"""

from collections.abc import Callable
import sys

from robot_voice.recognizer import TypedTextRecognizer

TRANSCRIPT_TOPIC = "speech_transcript"
_LISTENER_POLL_SECONDS = 0.1


def wait_for_listeners(
    subscription_count: Callable[[], int],
    settle: Callable[[], None],
    attempts: int,
) -> bool:
    """Wait until someone is subscribed, so the transcript is not dropped.

    Publishing into a topic nobody has matched yet is silent data loss, and
    silence is the one answer a voice path must never give.
    """
    for _ in range(max(attempts, 1)):
        if subscription_count() > 0:
            return True
        settle()
    return subscription_count() > 0


def phrases_from_args(argv: list[str]) -> tuple[str, ...]:
    """Join the command-line words into a single spoken phrase.

    `say go to the kitchen` and `say "go to the kitchen"` are the same
    sentence; quoting is a shell habit, not something the speaker should have
    to remember.
    """
    words = [word for word in argv if word.strip()]
    if not words:
        return ()
    return (" ".join(words),)


def main(args=None) -> int:
    # Imported here rather than at module scope so the delivery policy above
    # stays importable, and testable, on a machine with no ROS installed.
    import rclpy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import String

    rclpy.init(args=args)
    argv = remove_ros_args(args if args is not None else sys.argv)[1:]
    phrases = phrases_from_args(argv)
    interactive = not phrases

    recognizer = TypedTextRecognizer(
        phrases=phrases,
        prompt=(lambda: input("say> ")) if interactive else None,
    )

    node = rclpy.create_node("transcript_typist")
    publisher = node.create_publisher(String, TRANSCRIPT_TOPIC, 10)
    attempts = int(5.0 / _LISTENER_POLL_SECONDS)

    def settle() -> None:
        rclpy.spin_once(node, timeout_sec=_LISTENER_POLL_SECONDS)

    result = 0
    try:
        if interactive:
            print("Type a command and press enter. Ctrl-D to finish.")
        for transcript in recognizer.transcripts():
            if not wait_for_listeners(publisher.get_subscription_count, settle, attempts):
                node.get_logger().error(
                    f"nobody is subscribed to {TRANSCRIPT_TOPIC}: "
                    "is the voice node running?"
                )
                result = 1
                break
            publisher.publish(String(data=transcript))
            print(f"heard: {transcript}")
            # Hand the message to the middleware before anything shuts down.
            settle()
    except KeyboardInterrupt:
        pass
    finally:
        recognizer.close()
        node.destroy_node()
        rclpy.shutdown()

    return result


if __name__ == "__main__":
    sys.exit(main())
