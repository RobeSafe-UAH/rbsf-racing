"""
drive_gate state machine (no ROS dependencies). See docs/DESIGN_PHILOSOPHY.md §6.

The gate decides, at every step, which command reaches the actuators:

    MANUAL      joystick alive and manual deadman held: the joystick axes drive (always wins).
    AUTONOMOUS  joystick alive, armed and autonomous deadman held, and a fresh auto_cmd: the
                controller's command passes, clipped to the limits.
    STOPPED     anything else: speed 0, steering 0. The reason says which condition failed.

It is fail-closed: AUTONOMOUS needs positive, live evidence for every condition, and a joystick
that was never seen keeps the gate STOPPED. Arming happens on a press of the arm button while the
autonomous deadman is released, and is lost on a disarm press or when the joystick times out.

`require_deadman=False` (simulation only; the node refuses it on the real platform) drops the
joystick conditions: a fresh auto_cmd is enough for AUTONOMOUS, and manual still pre-empts it.

All times are seconds from a monotonic clock supplied by the caller.
"""

from dataclasses import dataclass
import enum
import math


class State(str, enum.Enum):
    STOPPED = 'STOPPED'
    MANUAL = 'MANUAL'
    AUTONOMOUS = 'AUTONOMOUS'


class Reason(str, enum.Enum):
    NONE = ''
    NO_JOY = 'NO_JOY'
    JOY_TIMEOUT = 'JOY_TIMEOUT'
    DISARMED = 'DISARMED'
    DEADMAN_RELEASED = 'DEADMAN_RELEASED'
    AUTO_CMD_TIMEOUT = 'AUTO_CMD_TIMEOUT'


@dataclass
class GateConfig:
    require_deadman: bool = True
    joy_timeout: float = 0.5
    auto_cmd_timeout: float = 0.2
    manual_deadman_button: int = 4
    auto_deadman_button: int = 5
    arm_button: int = 7
    disarm_button: int = 6
    speed_axis: int = 1
    steering_axis: int = 3
    max_manual_speed: float = 2.0
    max_auto_speed: float = 2.0
    max_steering: float = 0.4189


@dataclass
class Output:
    state: State
    reason: Reason
    speed: float
    steering: float


def _clip(value, limit):
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def _index(values, i, default):
    return values[i] if 0 <= i < len(values) else default


class DriveGate:

    def __init__(self, config):
        self.config = config
        self.armed = False
        self.joy_time = None
        self.axes = []
        self.buttons = []
        self.auto_time = None
        self.auto_cmd = (0.0, 0.0)

    def button(self, index):
        return bool(_index(self.buttons, index, 0))

    def joy_alive(self, now):
        return self.joy_time is not None and now - self.joy_time <= self.config.joy_timeout

    def on_joy(self, now, axes, buttons):
        """Store a joystick message; handle arm/disarm presses (rising edges)."""
        c = self.config
        # Only a release -> press transition seen on a live joystick is a press: on the first
        # message, or after a timeout, a button already held does not count.
        previous = self.buttons if self.joy_alive(now) else list(buttons)
        self.joy_time = now
        self.axes = list(axes)
        self.buttons = list(buttons)
        pressed = lambda i: self.button(i) and not _index(previous, i, 0)  # noqa: E731
        if pressed(c.disarm_button):
            self.armed = False
        elif pressed(c.arm_button) and not self.button(c.auto_deadman_button):
            self.armed = True

    def on_auto_cmd(self, now, speed, steering):
        self.auto_time = now
        self.auto_cmd = (speed, steering)

    def step(self, now):
        """Return the Output to apply now."""
        c = self.config
        alive = self.joy_alive(now)
        if self.joy_time is not None and not alive:
            self.armed = False

        if alive and self.button(c.manual_deadman_button):
            return Output(
                State.MANUAL, Reason.NONE,
                _clip(_index(self.axes, c.speed_axis, 0.0) * c.max_manual_speed,
                      c.max_manual_speed),
                _clip(_index(self.axes, c.steering_axis, 0.0) * c.max_steering, c.max_steering))

        if c.require_deadman:
            if self.joy_time is None:
                return self.stopped(Reason.NO_JOY)
            if not alive:
                return self.stopped(Reason.JOY_TIMEOUT)
            if not self.armed:
                return self.stopped(Reason.DISARMED)
            if not self.button(c.auto_deadman_button):
                return self.stopped(Reason.DEADMAN_RELEASED)
        if self.auto_time is None or now - self.auto_time > c.auto_cmd_timeout:
            return self.stopped(Reason.AUTO_CMD_TIMEOUT)
        speed, steering = self.auto_cmd
        return Output(State.AUTONOMOUS, Reason.NONE,
                      _clip(speed, c.max_auto_speed), _clip(steering, c.max_steering))

    @staticmethod
    def stopped(reason):
        return Output(State.STOPPED, reason, 0.0, 0.0)
