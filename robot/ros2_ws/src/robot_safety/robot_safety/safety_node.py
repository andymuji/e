"""ROS 2 node that gates requested velocity commands through safety checks."""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from robot_safety.safety_controller import SafetyController

# The gate says what it is doing only when that changes, so on a settled robot
# this topic is silent for minutes and a subscriber that arrives in the quiet
# hears nothing at all. Keeping the last line lets a console or a recorder
# learn the current state on arrival instead of waiting for the next thing to
# go wrong. The subscriber has to ask for it: a volatile subscription is still
# told nothing, which is why `ros2 topic echo` needs
# `--qos-durability transient_local` to see a quiet gate.
#
# The state is kept, not the liveness: a dead publisher delivers nothing, so
# this cannot hand anyone a state from a gate that has stopped running. What
# proves the loop is still running is cmd_vel, which goes out every cycle.
STATE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _optional(value: float) -> float | None:
    """Turn the negative sentinel used by the parameters into a disabled check."""
    return None if value < 0.0 else value


class SafetyNode(Node):
    """Publish only velocity commands that pass the safety controller."""

    def __init__(self) -> None:
        super().__init__("safety_controller")

        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("caution_distance", 0.85)
        self.declare_parameter("sensor_timeout", 0.5)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("require_localization", False)
        self.declare_parameter("localization_timeout", 1.0)
        # A negative limit means the check is off: ROS parameters have no
        # null, and 0.0 is a meaningful (impossible to satisfy) value.
        self.declare_parameter("max_localization_covariance", -1.0)
        # The frames of the transform AMCL broadcasts. That transform, not the
        # amcl_pose topic, is what says localization is still alive: see
        # _on_tf.
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("low_battery_fraction", -1.0)

        stop_distance = float(self.get_parameter("stop_distance").value)
        caution_distance = float(self.get_parameter("caution_distance").value)
        sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        command_timeout = float(self.get_parameter("command_timeout").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if not math.isfinite(control_rate_hz) or control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be finite and greater than zero")

        require_localization = bool(self.get_parameter("require_localization").value)
        localization_timeout = float(self.get_parameter("localization_timeout").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        max_covariance = _optional(
            float(self.get_parameter("max_localization_covariance").value)
        )
        low_battery_fraction = _optional(
            float(self.get_parameter("low_battery_fraction").value)
        )

        self._controller = SafetyController(
            stop_distance=stop_distance,
            caution_distance=caution_distance,
            sensor_timeout=sensor_timeout,
            command_timeout=command_timeout,
            require_localization=require_localization,
            localization_timeout=localization_timeout,
            max_localization_covariance=max_covariance,
            low_battery_fraction=low_battery_fraction,
        )
        self._requested_velocity = Twist()
        self._nearest_obstacle = None
        self._last_scan_time = None
        self._last_command_time = None
        self._last_status = None
        self._last_localization_time = None
        self._localization_covariance = None
        self._battery_fraction = None

        self._velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_publisher = self.create_publisher(
            String, "safety_state", STATE_QOS
        )
        self.create_subscription(Twist, "cmd_vel_requested", self._on_velocity, 10)
        self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(
            Bool, "emergency_stop_reset", self._on_emergency_stop_reset, 10
        )
        # Subscribed to only when the check is on, so an unused topic
        # cannot be mistaken for a check that is running.
        if require_localization:
            self.create_subscription(
                PoseWithCovarianceStamped, "amcl_pose", self._on_localization, 10
            )
            self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        if low_battery_fraction is not None:
            self.create_subscription(
                BatteryState, "battery_state", self._on_battery, 10
            )

        self.create_timer(1.0 / control_rate_hz, self._publish_safe_velocity)

    def _on_velocity(self, message: Twist) -> None:
        self._requested_velocity = message
        self._last_command_time = self._now()

    def _on_scan(self, message: LaserScan) -> None:
        valid_ranges = [
            distance
            for distance in message.ranges
            if math.isfinite(distance)
            and message.range_min <= distance <= message.range_max
        ]
        self._nearest_obstacle = min(valid_ranges) if valid_ranges else None
        self._last_scan_time = self._now()

    def _on_localization(self, message: PoseWithCovarianceStamped) -> None:
        """Record how sure the localizer is of the pose.

        The covariance is 6x6 row-major; entries 0 and 7 are the x and y
        position variances. The larger of the two is the one that decides
        whether the robot still knows where it is.

        Freshness deliberately does not come from here. AMCL only publishes
        this topic after the robot has moved past update_min_d/update_min_a,
        so a standing robot publishes nothing at all - see _on_tf.
        """
        covariance = message.pose.covariance
        self._localization_covariance = max(covariance[0], covariance[7])

    def _on_tf(self, message: TFMessage) -> None:
        """Treat AMCL's map->odom transform as the localization heartbeat.

        The amcl_pose topic is an event, not a heartbeat: AMCL publishes it
        only when the filter updates, which needs the robot to have moved.
        Taking freshness from it deadlocked the robot - the gate declared
        localization lost one second after it stopped, held the velocity at
        zero, and a robot that cannot move can never produce the update that
        would clear the fault. Measured on a standing robot: one amcl_pose in
        twenty seconds against seventeen map->odom broadcasts.

        Only AMCL publishes map->odom, so this stays a statement about the
        localizer rather than about tf traffic in general. It does assume
        AMCL's tf_broadcast is on, which is what navigation.launch.py runs.
        """
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") == self._map_frame
                and transform.child_frame_id.lstrip("/") == self._odom_frame
            ):
                self._last_localization_time = self._now()
                return

    def _on_battery(self, message: BatteryState) -> None:
        # A driver that reports NaN is reporting that it does not know, which
        # the controller treats as a stop rather than as a full battery.
        self._battery_fraction = message.percentage

    def _on_emergency_stop(self, message: Bool) -> None:
        """Engage the latch. A False here never releases it: see the reset topic."""
        if message.data:
            self._controller.engage_emergency_stop()

    def _on_emergency_stop_reset(self, message: Bool) -> None:
        """Release the latch only on a deliberate reset from the operator."""
        if message.data:
            self._controller.clear_emergency_stop()
            self.get_logger().warning("software emergency stop reset by operator")

    def _now(self) -> float:
        """Seconds from the node clock, so use_sim_time governs every timeout.

        Wall-clock ages would drift against sim-time data whenever the
        simulator runs slower than real time, tripping the timeouts on a
        healthy system. A clock that jumps backwards on a sim reset yields a
        negative age, which the controller already treats as a stop.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _age(self, timestamp: float | None) -> float | None:
        return None if timestamp is None else self._now() - timestamp

    def _publish_safe_velocity(self) -> None:
        decision = self._controller.evaluate(
            self._nearest_obstacle,
            self._age(self._last_scan_time),
            self._age(self._last_command_time),
            localization_age=self._age(self._last_localization_time),
            localization_covariance=self._localization_covariance,
            battery_fraction=self._battery_fraction,
        )

        safe = Twist()
        safe.linear.x = self._requested_velocity.linear.x * decision.speed_scale
        safe.linear.y = self._requested_velocity.linear.y * decision.speed_scale
        safe.linear.z = self._requested_velocity.linear.z * decision.speed_scale
        safe.angular.x = self._requested_velocity.angular.x * decision.speed_scale
        safe.angular.y = self._requested_velocity.angular.y * decision.speed_scale
        safe.angular.z = self._requested_velocity.angular.z * decision.speed_scale
        self._velocity_publisher.publish(safe)

        # Drop the stale request so a resumed command stream cannot replay it.
        if decision.reason == "motion command timed out":
            self._requested_velocity = Twist()

        status = f"{decision.state.value}: {decision.reason}"
        if status != self._last_status:
            state = String()
            state.data = status
            self._state_publisher.publish(state)
            self.get_logger().info(status)
            self._last_status = status


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
