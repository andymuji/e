"""ROS 2 node that gates requested velocity commands through safety checks."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from robot_safety.safety_controller import SafetyController


class SafetyNode(Node):
    """Publish only velocity commands that pass the safety controller."""

    def __init__(self) -> None:
        super().__init__("safety_controller")

        self.declare_parameter("stop_distance", 0.35)
        self.declare_parameter("caution_distance", 0.8)
        self.declare_parameter("sensor_timeout", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)

        stop_distance = float(self.get_parameter("stop_distance").value)
        caution_distance = float(self.get_parameter("caution_distance").value)
        sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if not math.isfinite(control_rate_hz) or control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be finite and greater than zero")

        self._controller = SafetyController(
            stop_distance=stop_distance,
            caution_distance=caution_distance,
            sensor_timeout=sensor_timeout,
        )
        self._requested_velocity = Twist()
        self._nearest_obstacle = None
        self._last_scan_time = None
        self._last_state = None

        self._velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_publisher = self.create_publisher(String, "safety_state", 10)
        self.create_subscription(Twist, "cmd_vel_requested", self._on_velocity, 10)
        self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "emergency_stop", self._on_emergency_stop, 10)
        self.create_timer(1.0 / control_rate_hz, self._publish_safe_velocity)

    def _on_velocity(self, message: Twist) -> None:
        self._requested_velocity = message

    def _on_scan(self, message: LaserScan) -> None:
        valid_ranges = [
            distance
            for distance in message.ranges
            if math.isfinite(distance)
            and message.range_min <= distance <= message.range_max
        ]
        self._nearest_obstacle = min(valid_ranges) if valid_ranges else None
        self._last_scan_time = time.monotonic()

    def _on_emergency_stop(self, message: Bool) -> None:
        self._controller.set_emergency_stop(message.data)

    def _publish_safe_velocity(self) -> None:
        reading_age = (
            time.monotonic() - self._last_scan_time
            if self._last_scan_time is not None
            else 0.0
        )
        decision = self._controller.evaluate(self._nearest_obstacle, reading_age)

        safe = Twist()
        safe.linear.x = self._requested_velocity.linear.x * decision.speed_scale
        safe.linear.y = self._requested_velocity.linear.y * decision.speed_scale
        safe.linear.z = self._requested_velocity.linear.z * decision.speed_scale
        safe.angular.x = self._requested_velocity.angular.x * decision.speed_scale
        safe.angular.y = self._requested_velocity.angular.y * decision.speed_scale
        safe.angular.z = self._requested_velocity.angular.z * decision.speed_scale
        self._velocity_publisher.publish(safe)

        if decision.state.value != self._last_state:
            state = String()
            state.data = f"{decision.state.value}: {decision.reason}"
            self._state_publisher.publish(state)
            self.get_logger().info(state.data)
            self._last_state = decision.state.value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
