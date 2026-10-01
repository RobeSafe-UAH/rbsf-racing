"""
drive_gate: the only path from any command source to a car's actuators.

Runs inside a car's namespace (/r0, ...) on the car side (docs/DESIGN_PHILOSOPHY.md §6), next to
joy_node. It is the only publisher on `drive`; controllers publish on drive_gate/auto_cmd.

Subscribes  joy                   sensor_msgs/Joy (manual axes, deadman, arm/disarm buttons)
            drive_gate/auto_cmd   ackermann_msgs/AckermannDriveStamped (controller slot)
Publishes   drive                 ackermann_msgs/AckermannDriveStamped (at `rate`, always)
            drive_gate/state      std_msgs/String, JSON {state, reason, armed} (latched, on change
                                  and at 2 Hz)

Algorithm (state machine in gate.py), on a steady-clock timer at `rate` Hz:
    1. Step the gate: MANUAL if the manual deadman is held; AUTONOMOUS if the joystick is alive,
       autonomy is armed, its deadman is held and auto_cmd is fresh; else STOPPED (speed 0,
       steering 0) with the reason.
    2. Publish the resulting command on `drive`.
Timeouts use the receive time on a monotonic clock, never message stamps or sim time, so a
stalled simulator or a skewed remote clock cannot keep a stale command alive.

Invariants checked at runtime:
    - Every second, `drive` must have exactly one publisher (this node). Otherwise the gate logs
      a fatal error and exits, and its launch file shuts the whole stack down.
    - require_deadman:=false is refused (the node exits) when platform is 'real'.
"""

import json
import time

from ackermann_msgs.msg import AckermannDriveStamped
from rbsf_safety.gate import DriveGate, GateConfig, State
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class GateViolation(RuntimeError):
    """A safety invariant does not hold: the stack must shut down."""


class DriveGateNode(Node):

    def __init__(self):
        super().__init__('drive_gate')
        self.declare_parameter('platform', 'real')
        self.declare_parameter('rate', 50.0)
        defaults = GateConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        config = GateConfig(**{
            name: type(value)(self.get_parameter(name).value)
            for name, value in vars(defaults).items()})

        platform = self.get_parameter('platform').value
        if not config.require_deadman and platform == 'real':
            raise GateViolation(
                'require_deadman:=false is not allowed on the real platform')
        if not config.require_deadman:
            self.get_logger().warn(
                f'Deadman NOT required (platform {platform}): auto_cmd alone drives the car')

        self.gate = DriveGate(config)
        self.last = None
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, 'drive_gate/state', latched)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(
            AckermannDriveStamped, 'drive_gate/auto_cmd', self.auto_callback, 10)

        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step, clock=steady)
        self.create_timer(0.5, self.publish_state, clock=steady)
        self.create_timer(1.0, self.check_publishers, clock=steady)
        self.drive_topic = self.resolve_topic_name('drive')
        self.get_logger().info(f'Gating {self.drive_topic}; starting STOPPED')

    def joy_callback(self, msg):
        self.gate.on_joy(time.monotonic(), msg.axes, msg.buttons)

    def auto_callback(self, msg):
        self.gate.on_auto_cmd(time.monotonic(), msg.drive.speed, msg.drive.steering_angle)

    def step(self):
        out = self.gate.step(time.monotonic())
        self.publish_drive(out.speed, out.steering)
        key = (out.state, out.reason, self.gate.armed)
        if key != self.last:
            text = (f"{out.state.value}{f' ({out.reason.value})' if out.reason.value else ''}"
                    f"{', armed' if self.gate.armed else ''}")
            # rclpy forbids changing the severity of one logging call site: one call each.
            if out.state == State.STOPPED:
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
            self.last = key
            self.publish_state()

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def publish_state(self):
        if self.last is None:
            return
        state, reason, armed = self.last
        self.state_pub.publish(String(data=json.dumps(
            {'state': state.value, 'reason': reason.value, 'armed': armed})))

    def check_publishers(self):
        count = self.count_publishers(self.drive_topic)
        if count > 1:
            raise GateViolation(
                f'{count} publishers on {self.drive_topic}: drive_gate must be the only one')

    def stop(self):
        """Command a stop on the way out (best effort)."""
        if rclpy.ok():
            self.publish_drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = DriveGateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except GateViolation as e:
        rclpy.logging.get_logger('drive_gate').fatal(f'{e}. Shutting down the stack.')
        code = 1
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
