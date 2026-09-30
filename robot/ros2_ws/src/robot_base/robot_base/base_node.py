"""ROS 2 node that drives the mecanum base's wheels from the gate's output.

Subscribes to /cmd_vel - the safety gate's output, and nothing else - sends
wheel commands to the Pico over USB serial every control cycle, and
publishes /odom from the Pico's encoder reports. It publishes no velocity
topic and never emergency_stop_reset: it is at the end of the motion path,
not in it. See base_controller.py for when it commands zero.

The Pico stops the motors by itself if these lines stop arriving (see
robot/firmware/pico_base), so a crash here, a pulled cable, or a hung Pi all
end with the wheels stopped. Zero is still sent on purpose on every path
this node controls, so that stopping never depends on that timeout alone.
"""

import math
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from robot_base.base_controller import (
    STOPPED,
    BaseController,
    DriveLimits,
    wheel_topic_problem,
)
from robot_base.kinematics import MecanumGeometry, OdometryIntegrator
from robot_base.protocol import (
    FULL_SCALE,
    BootReport,
    EncoderReport,
    Refusal,
    count_delta,
    format_motor_line,
    parse_pico_line,
    ticks_diff,
)

# Longest the receive buffer may grow without a newline before it is thrown
# away: a Pico printing garbage must not grow the driver's memory forever.
RX_LIMIT = 4096

# How often to retry opening a serial port that is missing or failed, s.
REOPEN_PERIOD = 1.0

# Encoder reports further apart than this are not turned into a velocity:
# the Pico rebooted, or the link stalled, and the step is not one motion.
MAX_REPORT_GAP = 1.0

# Uncertainty published with the odometry. GUESSES, not measurements: nothing
# consumes /odom on the car yet. Mecanum rollers slip, sideways most of all.
# The unused axes of a flat base get a huge variance rather than zero, which
# would claim they are known exactly.
POSE_VARIANCE = (0.05, 0.05, 1e6, 1e6, 1e6, 0.1)
TWIST_VARIANCE = (0.02, 0.05, 1e6, 1e6, 1e6, 0.05)


def _open_serial(port: str):
    """The real serial port. Imported here so the tests need no pyserial."""
    import serial

    # timeout=0: reads return what has arrived and never block the timer.
    # write_timeout: a Pico that stops reading is a link failure, not a wait.
    return serial.Serial(port, 115200, timeout=0, write_timeout=0.1)


def ramp_step(ramp_time: float, control_rate_hz: float) -> int | None:
    """Thousandths of full power a wheel may gain per control cycle."""
    if not math.isfinite(ramp_time) or ramp_time < 0.0:
        raise ValueError("ramp_time must be finite and not negative")
    if ramp_time == 0.0:
        return None
    return max(1, math.ceil(FULL_SCALE / (ramp_time * control_rate_hz)))


def _diagonal(variances: tuple[float, ...]) -> list[float]:
    covariance = [0.0] * 36
    for index, variance in enumerate(variances):
        covariance[index * 7] = variance
    return covariance


class BaseNode(Node):
    """Turn /cmd_vel into wheel commands for the Pico, and encoders into /odom."""

    def __init__(self, serial_factory=None) -> None:
        super().__init__("base_driver")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("gate_node", "safety_controller")
        # PLACEHOLDERS until measured on the real chassis: see base.yaml.
        self.declare_parameter("wheel_radius", 0.04)
        self.declare_parameter("half_wheelbase", 0.10)
        self.declare_parameter("half_track", 0.10)
        self.declare_parameter("encoder_counts_per_rev", 660.0)
        self.declare_parameter("max_wheel_speed", 60.0)
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 1.0)
        # Tank driving for the proof of concept: sideways requests dropped.
        self.declare_parameter("allow_sideways", False)
        # Seconds for a wheel to go from rest to full power. 0 = no ramp.
        # Stops are never ramped.
        self.declare_parameter("ramp_time", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # Off by default: on the car Cartographer publishes odom -> base_link
        # itself (provide_odom_frame), and a second publisher would give
        # base_link two parents. robot_bringup's tests prove this default.
        self.declare_parameter("publish_tf", False)

        def number(name: str) -> float:
            return float(self.get_parameter(name).value)

        control_rate_hz = number("control_rate_hz")
        if not math.isfinite(control_rate_hz) or control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be finite and greater than zero")
        self._counts_per_rev = number("encoder_counts_per_rev")
        if not math.isfinite(self._counts_per_rev) or self._counts_per_rev <= 0.0:
            raise ValueError("encoder_counts_per_rev must be finite and positive")

        geometry = MecanumGeometry(
            wheel_radius=number("wheel_radius"),
            half_wheelbase=number("half_wheelbase"),
            half_track=number("half_track"),
        )
        self._controller = BaseController(
            geometry,
            DriveLimits(
                max_linear_speed=number("max_linear_speed"),
                max_angular_speed=number("max_angular_speed"),
                max_wheel_speed=number("max_wheel_speed"),
            ),
            command_timeout=number("command_timeout"),
            allow_sideways=bool(self.get_parameter("allow_sideways").value),
            max_step=ramp_step(number("ramp_time"), control_rate_hz),
        )
        self._odometry = OdometryIntegrator(geometry)
        self._port_name = str(self.get_parameter("serial_port").value)
        self._gate_node = str(self.get_parameter("gate_node").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        if serial_factory is None:
            # Fail at start-up, with a clear message, if pyserial is missing,
            # rather than on the first timer tick.
            import serial  # noqa: F401

            serial_factory = _open_serial
        self._serial_factory = serial_factory
        self._port = None
        self._next_open = 0.0
        self._rx = bytearray()
        self._last_report: EncoderReport | None = None
        self._last_status: str | None = None
        self._last_link_error: str | None = None

        self._odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self._tf = (
            TransformBroadcaster(self)
            if bool(self.get_parameter("publish_tf").value)
            else None
        )
        self._cmd_vel = self.create_subscription(
            Twist, "cmd_vel", self._on_cmd_vel, 10
        )
        self.create_timer(1.0 / control_rate_hz, self._on_timer)

    # Commands -----------------------------------------------------------

    def _on_cmd_vel(self, message: Twist) -> None:
        # Monotonic, not the ROS clock: staleness is about wall time passing
        # on this computer, and must not freeze if use_sim_time is set.
        self._controller.on_command(
            message.linear.x, message.linear.y, message.angular.z, time.monotonic()
        )

    def _publishers_on_cmd_vel(self) -> list[str]:
        return [
            info.node_name
            for info in self.get_publishers_info_by_topic(self._cmd_vel.topic_name)
        ]

    def _on_timer(self) -> None:
        now = time.monotonic()
        decision = self._controller.decide(
            now, wheel_topic_problem(self._publishers_on_cmd_vel(), self._gate_node)
        )
        self._report_status(decision.reason)

        if not self._ensure_port(now):
            return
        try:
            self._read_reports()
            self._send(decision.commands)
        except OSError as error:  # pyserial's SerialException is an OSError
            self._drop_port(f"serial link to the Pico failed: {error}")

    def _send(self, commands: tuple[int, ...]) -> None:
        self._port.write((format_motor_line(commands) + "\n").encode("ascii"))

    def _report_status(self, status: str) -> None:
        if status == self._last_status:
            return
        self._last_status = status
        if status in ("driving", "stopped"):
            self.get_logger().info(f"wheels: {status}")
        else:
            self.get_logger().warning(f"wheels held at zero: {status}")

    # Serial link --------------------------------------------------------

    def _ensure_port(self, now: float) -> bool:
        if self._port is not None:
            return True
        if now < self._next_open:
            return False
        self._next_open = now + REOPEN_PERIOD
        try:
            self._port = self._serial_factory(self._port_name)
        except OSError as error:
            self._link_error(
                f"cannot open the Pico on {self._port_name}: {error}. "
                f"Retrying every {REOPEN_PERIOD:.0f} s; the wheels stay stopped."
            )
            return False
        self._rx.clear()
        self._last_report = None
        self._last_link_error = None
        self.get_logger().info(f"serial link to the Pico open on {self._port_name}")
        return True

    def _link_error(self, message: str) -> None:
        # Once per distinct failure, so a missing Pico is one line, not twenty
        # a second.
        if message != self._last_link_error:
            self._last_link_error = message
            self.get_logger().error(message)

    def _drop_port(self, message: str) -> None:
        self._link_error(message)
        port, self._port = self._port, None
        try:
            port.close()
        except OSError:
            pass

    def close(self) -> None:
        """Send one last zero and close the port. Safe to call twice."""
        if self._port is None:
            return
        port = self._port
        try:
            self._send(STOPPED)
        except OSError as error:
            self.get_logger().warning(f"could not send the final stop: {error}")
        self._port = None
        try:
            port.close()
        except OSError:
            pass

    def destroy_node(self) -> None:
        self.close()
        super().destroy_node()

    # Encoder reports ----------------------------------------------------

    def _read_reports(self) -> None:
        waiting = self._port.in_waiting
        if not waiting:
            return
        self._rx += self._port.read(waiting)
        *lines, rest = self._rx.split(b"\n")
        self._rx = bytearray(rest)
        if len(self._rx) > RX_LIMIT:
            self.get_logger().warning("discarding a line from the Pico that never ended")
            self._rx.clear()
        for raw in lines:
            self._on_line(raw.decode("ascii", errors="replace").strip())

    def _on_line(self, line: str) -> None:
        if not line:
            return
        try:
            message = parse_pico_line(line)
        except ValueError:
            # Most likely a MicroPython traceback: worth reading in full.
            self.get_logger().warning(f"from the Pico: {line}")
            return
        if isinstance(message, EncoderReport):
            self._on_report(message)
        elif isinstance(message, BootReport):
            self._last_report = None
            log = self.get_logger().info if message.cause == "power" else (
                self.get_logger().error
            )
            log(f"the Pico started (last reset: {message.cause}); motors are off")
        elif isinstance(message, Refusal):
            self.get_logger().error(
                f"the Pico refused a command ({message.reason}) and stopped the motors"
            )

    def _on_report(self, report: EncoderReport) -> None:
        previous, self._last_report = self._last_report, report
        if previous is None:
            return
        dt = ticks_diff(report.ms, previous.ms) / 1000.0
        if not 0.0 < dt <= MAX_REPORT_GAP:
            return
        angles = [
            count_delta(new, old) * 2.0 * math.pi / self._counts_per_rev
            for new, old in zip(report.totals, previous.totals, strict=True)
        ]
        vx, vy, wz = self._odometry.update(angles, dt)
        self._publish_odometry(vx, vy, wz)

    def _publish_odometry(self, vx: float, vy: float, wz: float) -> None:
        stamp = self.get_clock().now().to_msg()
        half = self._odometry.theta / 2.0

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odometry.x
        odom.pose.pose.position.y = self._odometry.y
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.pose.covariance = _diagonal(POSE_VARIANCE)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = _diagonal(TWIST_VARIANCE)
        self._odom_publisher.publish(odom)

        if self._tf is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = self._odometry.x
            transform.transform.translation.y = self._odometry.y
            transform.transform.rotation = odom.pose.pose.orientation
            self._tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # How the driver is meant to be stopped; see robot_safety's node.
        pass
    finally:
        node.destroy_node()  # Sends the final zero and closes the port.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
