#!/usr/bin/env python3

import atexit
import os
import time
from collections import deque

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from shared_link_bridge.msg import KairosValues, VehicleControl
from shared_link_bridge.srv import GetVehicleControl, ExecCmd, FieldUpdate

from enum import Enum

from tui import TerminalUI, Page, MenuItem


class RateTracker:
    """Sliding-window Hz estimator: avg interval over the last N timestamps."""

    def __init__(self, window: int = 5):
        self._times: deque[float] = deque(maxlen=window)

    def tick(self) -> None:
        self._times.append(time.monotonic())

    def hz(self) -> float:
        if len(self._times) < 2:
            return 0.0
        dt = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / dt if dt > 0 else 0.0


# Map Joy.buttons indices to command names (edge-triggered).
# Default layout assumes an Xbox-style controller as published by joy_node.
class BUTTON(Enum):
    A = 0       # A
    B = 1       # B
    X = 2       # X
    Y = 3       # Y
    LB = 4      # LEFT BUMPER (upper)
    RB = 5      # RIGHT BUMPER (upper)
    BACK = 6    # BACK
    START = 7   # START
    LOGI = 8    # CENTER LOGITECH BUTTON
    LJ = 9      # LEFT JOY PRESS
    RJ = 10     # RIGHT JOY PRESS

class AXIS(Enum):
    LX = 0      # LEFT X Axis           1 (left) to -1 (right)
    LY = 1      # LEFT Y Axis           1 (up) to -1 (down)
    LT = 2      # LEFT Trigger          1 (released) to -1 (deperessed)
    RX = 3      # RIGHT X Axis          1 (left) to -1 (right)
    RY = 4      # RIGHT Y Axis          1 (up) to -1 (down)
    RT = 5      # RIGHT Trigger         1 (released) to -1 (deperessed)
    DPX = 6     # D-PAD X Axis          DISCRETE: 1 (left), 0 (released), -1 (right)
    DPY = 7     # D-PAD Y Axis          DISCRETE: 1 (up), 0 (released), -1 (down)


BUTTONS = {
    # BUTTON.B:     'stop',
    # BUTTON.LB:    'connect',
    # BUTTON.BACK:  'teleop_stop',
    # BUTTON.START: 'teleop_start',
    BUTTON.LOGI:  'ping'
}

# A is reserved for the TUI as the "select" button.
SELECT_BUTTON = BUTTON.A

# Buttons handled separately (hold/release semantics rather than edge actions).
DEADMAN_BUTTON = BUTTON.RB         # held -> motion enabled
THROTTLE_BOOST_BUTTON = BUTTON.LJ  # held -> boosted throttle scale
STEER_BOOST_BUTTON = BUTTON.RJ     # held -> boosted steering scale

# Continuous control axes.
THROTTLE_AXIS = AXIS.LY   # positive forward, negative brake
STEER_AXIS = AXIS.RX

# Default and boosted scales. Loaded from config/controller_teleop.yaml.
_config_path = os.path.join(
    get_package_share_directory('shared_link_bridge'),
    'config', 'controller_teleop.yaml')
with open(_config_path) as _f:
    _scales = yaml.safe_load(_f)['scales']

# ICD maxima (SharedLink Teleop Only ICD). Scales above these are clamped here,
# and the bridge clamps every outbound command to them as well.
JOY_STEER_MAX = 450.0
VEH_THROTTLE_MAX = 1000
VEH_BRAKE_MAX = 100

THROTTLE_SCALE = min(_scales['throttle_scale'], VEH_THROTTLE_MAX)
THROTTLE_SCALE_BOOSTED = min(_scales['throttle_scale_boosted'], VEH_THROTTLE_MAX)
STEER_SCALE = min(_scales['steer_scale'], JOY_STEER_MAX)
STEER_SCALE_BOOSTED = min(_scales['steer_scale_boosted'], JOY_STEER_MAX)
BRAKE_MAX = min(_scales['brake_max'], VEH_BRAKE_MAX)


class ShiftPosition(Enum):
    PARK = 0
    REVERSE = 1
    NEUTRAL = 2
    DRIVE = 3


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_teleop')

        self._cmd = VehicleControl()
        self._prev_buttons: list[int] = []
        self._prev_axes: list[float] = []
        self._joy_rate = RateTracker()
        self._kairos_rate = RateTracker()
        self._connected = False
        self._teleop_on = False

        self._get_ctrl_client = self.create_client(GetVehicleControl, 'get_vehicle_control')
        self._exec_cmd_client = self.create_client(ExecCmd, 'exec_cmd')
        self._field_update_client = self.create_client(FieldUpdate, 'field_update')

        # Waiting for get_vehicle_control service (waits for shared_link_bridge_node)
        while not self._get_ctrl_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for get_vehicle_control service...')
        future = self._get_ctrl_client.call_async(GetVehicleControl.Request())
        rclpy.spin_until_future_complete(self, future)
        self._cmd = future.result().control

        # Publishers
        self._ctrl_pub = self.create_publisher(VehicleControl, 'vehicle_control', 10)

        # Subscriptions
        self._kairos_sub = self.create_subscription(
            KairosValues, 'kairos_values',
            self._kairos_callback, 10)
        self._joy_sub = self.create_subscription(
            Joy, 'joy',
            self._joy_callback, 10)

        # Terminal UI
        self._tui = TerminalUI(
            pages=[
                Page("Main", [
                    MenuItem(lambda: "connected" if self._connected else "disconnected",
                             self.toggle_connect),
                    MenuItem(lambda: f"teleop: {'on' if self._teleop_on else 'off'}",
                             self.toggle_teleop),
                ]),
                Page("Shift", [
                    MenuItem("drive",   lambda: self.set_shift(ShiftPosition.DRIVE)),
                    MenuItem("neutral", lambda: self.set_shift(ShiftPosition.NEUTRAL)),
                    MenuItem("reverse", lambda: self.set_shift(ShiftPosition.REVERSE)),
                    MenuItem("parking", lambda: self.set_shift(ShiftPosition.PARK)),
                ], initial_cursor=3),
            ],
            info_provider=self._info_lines,
        )
        atexit.register(self._tui.stop)
        self._tui.render()
        self._render_timer = self.create_timer(0.2, self._tui.render)

    # -------------- #
    #    Callbacks
    # -------------- #
    def _kairos_callback(self, msg: KairosValues):
        self._kairos_rate.tick()

    def _joy_callback(self, msg: Joy):
        self._joy_rate.tick()
        # Wrong layout (e.g. F710 switched to D mode): warn and ignore the message instead
        # of crashing. No /vehicle_control is published, so the bridge's watchdog disables
        # motion if it was enabled.
        if len(msg.buttons) != len(BUTTON) or len(msg.axes) != len(AXIS):
            self.get_logger().warn(
                f'Unexpected joystick layout: {len(msg.buttons)} buttons, {len(msg.axes)} axes '
                f'(expected {len(BUTTON)} and {len(AXIS)}). Is the F710 switch on X? '
                'Ignoring input.',
                throttle_duration_sec=2.0)
            return

        # Edge-triggered button actions
        for btn, action in BUTTONS.items():
            if self._held(msg, btn) and not self._was_held(btn):
                getattr(self, action)()

        # TUI navigation via D-pad + A
        if self._axis_edge(msg, AXIS.DPY, 1.0):
            self._tui.move_up()
            self._tui.render()
        if self._axis_edge(msg, AXIS.DPY, -1.0):
            self._tui.move_down()
            self._tui.render()
        if self._axis_edge(msg, AXIS.DPX, 1.0):
            self._tui.prev_page()
            self._tui.render()
        if self._axis_edge(msg, AXIS.DPX, -1.0):
            self._tui.next_page()
            self._tui.render()
        if self._held(msg, SELECT_BUTTON) and not self._was_held(SELECT_BUTTON):
            self._tui.select()
            self._tui.render()

        # Deadman: motion enabled only while held
        deadman = self._held(msg, DEADMAN_BUTTON)
        was_deadman = self._was_held(DEADMAN_BUTTON)
        if deadman and not was_deadman:
            self.start()
        elif not deadman and was_deadman:
            self.stop()

        # Continuous steering. ROS convention (REP 103): positive = left, matching the
        # joystick (RX is +1 left). The bridge converts to Kairos's positive-right convention.
        steer_scale = STEER_SCALE_BOOSTED if self._held(msg, STEER_BOOST_BUTTON) else STEER_SCALE
        self._cmd.joy_steer = msg.axes[STEER_AXIS.value] * steer_scale

        # Throttle / brake
        v = msg.axes[THROTTLE_AXIS.value]
        if v >= 0:
            throttle_scale = THROTTLE_SCALE_BOOSTED if self._held(msg, THROTTLE_BOOST_BUTTON) else THROTTLE_SCALE
            self._cmd.veh_throttle = int(v * throttle_scale)
            self._cmd.veh_brake = 0
        else:
            self._cmd.veh_throttle = 0
            self._cmd.veh_brake = int(-v * BRAKE_MAX)

        self._publish()
        self._prev_buttons = list(msg.buttons)
        self._prev_axes = list(msg.axes)

    def _held(self, msg: Joy, btn: BUTTON) -> bool:
        return btn.value < len(msg.buttons) and msg.buttons[btn.value] == 1

    def _was_held(self, btn: BUTTON) -> bool:
        return btn.value < len(self._prev_buttons) and self._prev_buttons[btn.value] == 1

    def _axis_edge(self, msg: Joy, axis: AXIS, value: float) -> bool:
        """Edge-trigger: true when `axis` just transitioned to `value`."""
        prev = self._prev_axes[axis.value] if axis.value < len(self._prev_axes) else 0.0
        return msg.axes[axis.value] == value and prev != value

    # -------------- #
    #    Commands
    # -------------- #
    def _publish(self):
        self._ctrl_pub.publish(self._cmd)

    def start(self):
        # Technically veh_enable can be set to 1 perminently
        # The veh_motion is supposed to be the deadman's switch
        self._cmd.veh_motion = 1
        self._cmd.veh_enable = 1
        self._publish()

    def stop(self):
        self._cmd.veh_motion = 0
        self._cmd.veh_enable = 0
        self._publish()

    def connect(self):
        req = ExecCmd.Request()
        req.cmd = ExecCmd.Request.CONNECT
        self._exec_cmd_client.call_async(req)

    def disconnect(self):
        req = ExecCmd.Request()
        req.cmd = ExecCmd.Request.DISCONNECT
        self._exec_cmd_client.call_async(req)

    def teleop_start(self):
        req = ExecCmd.Request()
        req.cmd = ExecCmd.Request.TELEOP_START
        self._exec_cmd_client.call_async(req)

    def teleop_stop(self):
        req = ExecCmd.Request()
        req.cmd = ExecCmd.Request.TELEOP_STOP
        self._exec_cmd_client.call_async(req)

    def ping(self):
        req = ExecCmd.Request()
        req.cmd = ExecCmd.Request.PING
        self._exec_cmd_client.call_async(req)

    def set_shift(self, position: ShiftPosition):
        self._cmd.veh_shift = position.value
        self._publish()

    def toggle_connect(self):
        if self._connected:
            self.disconnect()
        else:
            self.connect()
        self._connected = not self._connected

    def toggle_teleop(self):
        if self._teleop_on:
            self.teleop_stop()
        else:
            self.teleop_start()
        self._teleop_on = not self._teleop_on

    # -------------- #
    #    TUI info
    # -------------- #
    def _info_lines(self) -> list[str]:
        deadman = "ON" if self._cmd.veh_motion else "OFF"
        return [
            f"deadman:  {deadman}",
            f"throttle: {self._cmd.veh_throttle}",
            f"brake:    {self._cmd.veh_brake}",
            f"steer:    {self._cmd.joy_steer:+.1f}",
            "",
            f"/joy:           {self._joy_rate.hz():5.1f} Hz",
            f"/kairos_values: {self._kairos_rate.hz():5.1f} Hz",
        ]

    def field_update(self, name: str, value):
        req = FieldUpdate.Request()
        req.name = name
        req.value = str(value)
        self._field_update_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
