import math

import pytest
from rbsf_safety.gate import DriveGate, GateConfig, Reason, State

MANUAL, AUTO, ARM, DISARM = 4, 5, 7, 6
AXES = [0.0, 0.5, 0.0, -0.5]  # speed axis 1, steering axis 3


def buttons(*held):
    b = [0] * 12
    for i in held:
        b[i] = 1
    return b


def armed_gate(config=None, t=0.0):
    """Gate armed at time t, deadman released, joystick alive."""
    gate = DriveGate(config or GateConfig())
    gate.on_joy(t, AXES, buttons())
    gate.on_joy(t, AXES, buttons(ARM))
    gate.on_joy(t, AXES, buttons())
    assert gate.armed
    return gate


def autonomous_gate(t=0.0):
    gate = armed_gate(t=t)
    gate.on_joy(t, AXES, buttons(AUTO))
    gate.on_auto_cmd(t, 1.5, 0.1)
    out = gate.step(t)
    assert out.state == State.AUTONOMOUS
    return gate


def test_never_received_joy_stays_stopped():
    gate = DriveGate(GateConfig())
    gate.on_auto_cmd(0.0, 1.0, 0.0)
    for t in (0.0, 10.0, 1000.0):
        out = gate.step(t)
        assert (out.state, out.reason, out.speed, out.steering) == \
            (State.STOPPED, Reason.NO_JOY, 0.0, 0.0)


def test_disarmed_until_armed():
    gate = DriveGate(GateConfig())
    gate.on_joy(0.0, AXES, buttons(AUTO))
    gate.on_auto_cmd(0.0, 1.0, 0.0)
    assert gate.step(0.0).reason == Reason.DISARMED


def test_arming_with_deadman_held_is_refused():
    gate = DriveGate(GateConfig())
    gate.on_joy(0.0, AXES, buttons(AUTO))
    gate.on_joy(0.0, AXES, buttons(AUTO, ARM))
    assert not gate.armed


def test_autonomous_needs_deadman_held():
    gate = autonomous_gate()
    out = gate.step(0.0)
    assert (out.speed, out.steering) == (1.5, 0.1)
    gate.on_joy(0.05, AXES, buttons())
    gate.on_auto_cmd(0.05, 1.5, 0.1)
    out = gate.step(0.05)
    assert (out.state, out.reason, out.speed) == (State.STOPPED, Reason.DEADMAN_RELEASED, 0.0)


def test_auto_cmd_timeout_stops():
    # Controller crash or network loss in distributed mode.
    gate = autonomous_gate()
    gate.on_joy(0.3, AXES, buttons(AUTO))
    out = gate.step(0.3)
    assert (out.state, out.reason, out.speed) == (State.STOPPED, Reason.AUTO_CMD_TIMEOUT, 0.0)
    gate.on_auto_cmd(0.35, 1.0, 0.0)
    assert gate.step(0.35).state == State.AUTONOMOUS


def test_joy_timeout_stops_and_disarms():
    gate = autonomous_gate()
    gate.on_auto_cmd(0.6, 1.0, 0.0)
    out = gate.step(0.6)
    assert (out.state, out.reason) == (State.STOPPED, Reason.JOY_TIMEOUT)
    assert not gate.armed
    # The joystick comes back with the deadman still held: it must be re-armed first.
    gate.on_joy(0.7, AXES, buttons(AUTO))
    gate.on_auto_cmd(0.7, 1.0, 0.0)
    assert gate.step(0.7).reason == Reason.DISARMED


def test_held_arm_button_is_not_a_press():
    gate = DriveGate(GateConfig())
    gate.on_joy(0.0, AXES, buttons(ARM))
    assert not gate.armed  # held on the first message
    gate = armed_gate()
    gate.step(1.0)  # timeout disarms
    gate.on_joy(1.0, AXES, buttons(ARM))
    assert not gate.armed  # held across the timeout
    gate.on_joy(1.05, AXES, buttons())
    gate.on_joy(1.1, AXES, buttons(ARM))
    assert gate.armed


def test_disarm_button():
    gate = autonomous_gate()
    gate.on_joy(0.05, AXES, buttons(AUTO, DISARM))
    gate.on_auto_cmd(0.05, 1.0, 0.0)
    assert gate.step(0.05).reason == Reason.DISARMED


def test_manual_preempts_autonomous():
    gate = autonomous_gate()
    gate.on_joy(0.05, AXES, buttons(AUTO, MANUAL))
    gate.on_auto_cmd(0.05, 1.5, 0.1)
    out = gate.step(0.05)
    c = GateConfig()
    assert out.state == State.MANUAL
    assert out.speed == pytest.approx(0.5 * c.max_manual_speed)
    assert out.steering == pytest.approx(-0.5 * c.max_steering)


def test_manual_needs_live_joystick():
    gate = DriveGate(GateConfig())
    gate.on_joy(0.0, AXES, buttons(MANUAL))
    assert gate.step(0.0).state == State.MANUAL
    out = gate.step(1.0)
    assert (out.state, out.reason, out.speed) == (State.STOPPED, Reason.JOY_TIMEOUT, 0.0)


def test_commands_are_clipped():
    config = GateConfig(max_auto_speed=2.0, max_steering=0.4)
    gate = armed_gate(config)
    gate.on_joy(0.0, AXES, buttons(AUTO))
    for cmd, expected in [((5.0, 1.0), (2.0, 0.4)), ((-5.0, -1.0), (-2.0, -0.4)),
                          ((math.nan, math.inf), (0.0, 0.0))]:
        gate.on_auto_cmd(0.0, *cmd)
        out = gate.step(0.0)
        assert (out.speed, out.steering) == expected


def test_missing_axes_and_buttons_are_neutral():
    gate = DriveGate(GateConfig())
    gate.on_joy(0.0, [], [])
    out = gate.step(0.0)
    assert (out.state, out.reason) == (State.STOPPED, Reason.DISARMED)


def test_waiver_drives_on_auto_cmd_alone():
    gate = DriveGate(GateConfig(require_deadman=False))
    assert gate.step(0.0).reason == Reason.AUTO_CMD_TIMEOUT
    gate.on_auto_cmd(0.0, 1.0, 0.0)
    assert gate.step(0.0).state == State.AUTONOMOUS
    assert gate.step(0.5).reason == Reason.AUTO_CMD_TIMEOUT
    # A joystick still takes over with the manual deadman.
    gate.on_auto_cmd(1.0, 1.0, 0.0)
    gate.on_joy(1.0, AXES, buttons(MANUAL))
    assert gate.step(1.0).state == State.MANUAL
