"""Write down which nodes were publishing the safety-critical topics.

A rosbag stores messages and not their senders, so a recording on its own
cannot answer the question the motion path depends on: was the safety gate the
only thing publishing `/cmd_vel`? This node watches the live topic graph while
the run is recorded and leaves the answer in a small JSON file inside the bag
directory, so the evidence travels with the recording.

SAFETY: this node creates no publisher of its own and no service client. It
reads the graph and writes a file. Nothing in `robot_telemetry` may ever send
a message to a running robot - least of all a velocity or an
`emergency_stop_reset` - because a recorder that can also act is no longer a
witness.
"""

from datetime import UTC, datetime
import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .records import (
    CMD_VEL,
    CMD_VEL_REQUESTED,
    EMERGENCY_STOP,
    EMERGENCY_STOP_RESET,
    SAFETY_STATE,
    SCAN,
)

# The topics whose publishers decide whether the motion path is what the
# documentation claims. `/cmd_vel` is the one that matters; the rest are
# recorded because knowing who was asking is how you explain what the gate did.
WATCHED_TOPICS = (
    CMD_VEL,
    CMD_VEL_REQUESTED,
    SAFETY_STATE,
    SCAN,
    EMERGENCY_STOP,
    EMERGENCY_STOP_RESET,
)


class GraphProbe(Node):
    """Poll the topic graph and keep a running union of what it saw.

    A union rather than a snapshot: a node that published to `/cmd_vel` for
    one second during a ten-minute run is exactly the finding this exists for,
    and a snapshot taken a second later would miss it.
    """

    def __init__(self) -> None:
        super().__init__("telemetry_graph_probe")

        self.declare_parameter("output_file", "")
        self.declare_parameter("poll_period", 1.0)
        self.declare_parameter("topics", list(WATCHED_TOPICS))

        output = str(self.get_parameter("output_file").value)
        if not output:
            raise ValueError("output_file is required: nowhere to record the graph")
        self._output = Path(output)
        self._topics = [str(topic) for topic in self.get_parameter("topics").value]

        poll_period = float(self.get_parameter("poll_period").value)
        if poll_period <= 0.0:
            raise ValueError("poll_period must be greater than zero")

        self._seen: dict[str, set[str]] = {topic: set() for topic in self._topics}
        self._polls = 0
        self._started = datetime.now(UTC).isoformat(timespec="seconds")
        self._warned = False

        self.create_timer(poll_period, self._poll)

    def _poll(self) -> None:
        self._polls += 1
        for topic in self._topics:
            for endpoint in self.get_publishers_info_by_topic(topic):
                self._seen[topic].add(
                    _full_name(endpoint.node_namespace, endpoint.node_name)
                )
        self.write()

    def write(self) -> bool:
        """Save what has been seen so far. Safe to call as often as you like.

        The bag directory is created by `ros2 bag record`, which refuses to
        start if it already exists, so this waits for it rather than creating
        it. Until then the observations are simply held in memory.
        """
        if not self._output.parent.is_dir():
            if not self._warned and self._polls > 5:
                self._warned = True
                self.get_logger().warning(
                    f"{self._output.parent} does not exist; the topic graph "
                    "cannot be saved and the recording will not be able to show "
                    "who published /cmd_vel"
                )
            return False
        document = {
            "started": self._started,
            "polls": self._polls,
            "topics": {
                topic: sorted(names) for topic, names in self._seen.items()
            },
        }
        self._output.write_text(json.dumps(document, indent=2) + "\n")
        return True


def _full_name(namespace: str, name: str) -> str:
    return f"{namespace.rstrip('/')}/{name}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraphProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C is how a recording normally ends, and launch shuts the
        # context down under the node rather than raising KeyboardInterrupt in
        # it. Neither is a failure, and a recorder that exits non-zero on a
        # normal stop teaches an operator to ignore its exit status.
        pass
    finally:
        # The last write is the one that covers the end of the run.
        node.write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
