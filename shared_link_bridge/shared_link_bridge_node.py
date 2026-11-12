#!/usr/bin/env python3
import time
from typing import Union

from enum import Enum
from dataclasses import dataclass, field, fields

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from shared_link_bridge.msg import GpsValues, KairosValues, VehicleControl
from shared_link_bridge.srv import GetVehicleControl, ExecCmd, FieldUpdate

from UDPConnection import UdpConnection
from premade_msgs import *

# Command ranges from the SharedLink Teleop Only ICD (BUL-110, v1.00.15).
# Every outbound value is clamped to these before it goes on the wire.
JOY_STEER_MAX = 450.0    # steering-wheel degrees, +-450
VEH_BRAKE_MAX = 100      # 0-100 %
VEH_THROTTLE_MAX = 1000  # 0-1000 = 0-100.0 %
VEH_SHIFT_MAX = 3        # 0 Park, 1 Reverse, 2 Neutral, 3 Drive


class TYPE(Enum):
    INT = 0
    FLOAT = 2
    STRING = 4

@dataclass
class SVField:
    type: int  # 0=long, 2=double, 4=string
    value: Union[int, float, str, None] = None
    updated: bool = False

    def __post_init__(self):
        if self.value is None:
            if self.type == TYPE.INT:
                self.value = 0
            elif self.type == TYPE.FLOAT:
                self.value = 0.0
            elif self.type == TYPE.STRING:
                self.value = ""

    def apply(self, raw: str):
        """Parse a raw D field string and apply it to this variable."""
        if raw is None or raw == '':
            return
        if self.type == TYPE.INT:
            self.value = int(float(raw))
        elif self.type == TYPE.FLOAT:
            self.value = float(raw)
        elif self.type == TYPE.STRING:
            self.value = str(raw)
        self.updated = True


@dataclass
class SharedLinkOutbound:
    """OCU -> Vehicle (N list)"""
    video_mux1:   SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_start:    SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_enable:   SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_motion:   SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    joy_steer:    SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    veh_steer:    SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_brake:    SVField = field(default_factory=lambda: SVField(type=TYPE.INT, value=100))
    veh_throttle: SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_shift:    SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_clutch:   SVField = field(default_factory=lambda: SVField(type=TYPE.INT))


@dataclass
class SharedLinkInbound:
    """Vehicle -> OCU (L list)"""
    AutoGpsHead:       SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    AutoGpsLat:        SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    AutoGpsLon:        SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    AutoGpsVel:        SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    AutoGpsReferences: SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    AutoGpsQuality:    SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    AutoGpsStamp:      SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    pause:             SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    key_on:            SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    eng_enb:           SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    pe7:               SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    bbrake:            SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    fbrake:            SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    steerang:          SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    throttle:          SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    svp_enb_eng:       SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    serialpck:         SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_gear:          SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_state:         SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_estop:         SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    veh_obstacle:      SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    vehicle_bat:       SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))
    run_composite:     SVField = field(default_factory=lambda: SVField(type=TYPE.STRING))
    VehicleName:       SVField = field(default_factory=lambda: SVField(type=TYPE.STRING))
    Health_total:      SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    rpm:               SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    fuel:              SVField = field(default_factory=lambda: SVField(type=TYPE.INT))
    sys_thd:           SVField = field(default_factory=lambda: SVField(type=TYPE.FLOAT))

INBOUND_TO_ROS = {
    'AutoGpsHead':       'auto_gps_head',
    'AutoGpsLat':        'auto_gps_lat',
    'AutoGpsLon':        'auto_gps_lon',
    'AutoGpsVel':        'auto_gps_vel',
    'AutoGpsReferences': 'auto_gps_references',
    'AutoGpsQuality':    'auto_gps_quality',
    'AutoGpsStamp':      'auto_gps_stamp',
    'VehicleName':       'vehicle_name',
    'Health_total':      'health_total',
}

GPS_FIELDS = {
    'AutoGpsHead':       'auto_gps_head',
    'AutoGpsLat':        'auto_gps_lat',
    'AutoGpsLon':        'auto_gps_lon',
    'AutoGpsVel':        'auto_gps_vel',
    'AutoGpsReferences': 'auto_gps_references',
    'AutoGpsQuality':    'auto_gps_quality',
    'AutoGpsStamp':      'auto_gps_stamp',
}

class SharedLinkNode(Node):
    def __init__(self):
        super().__init__('shared_link_bridge_node')
        self.declare_parameter('update_rate', 0.05)
        # ROS side (/vehicle_control, /kairos_values) uses REP 103: positive steering = left.
        # Kairos uses positive = right (ICD example stream: "Steer Left" sends joy_steer -359).
        self.declare_parameter('invert_steering', True)
        # Hard limits applied on the wire. Teleop's own scales live in controller_teleop.yaml.
        self.declare_parameter('steer_limit', JOY_STEER_MAX)        # float, <= 450.0
        self.declare_parameter('throttle_limit', VEH_THROTTLE_MAX)  # int, <= 1000
        # Watchdog. If /vehicle_control stops for command_timeout seconds while motion is
        # enabled, the bridge stops the vehicle in two stages, because Kairos freezes the
        # actuators where they are when motion is disabled:
        #   stage 1: throttle 0 and brake timeout_brake, motion still enabled, for
        #            stop_hold_time seconds so the actuators actually move;
        #   stage 2: motion disabled. The stop then latches until a command arrives with
        #            veh_motion 0 (deadman released). 0.0 turns the watchdog off.
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('timeout_brake', VEH_BRAKE_MAX)      # int, 0-100
        self.declare_parameter('stop_hold_time', 5.0)               # float, seconds
        # Motion may only be enabled while veh_throttle is at or below this (0-1000), so a
        # stored throttle command is never applied the moment motion comes on.
        self.declare_parameter('enable_throttle_max', 50)

        self._update_rate = self.get_parameter('update_rate').value
        self._invert_steering = bool(self.get_parameter('invert_steering').value)
        self._steer_limit = min(abs(float(self.get_parameter('steer_limit').value)), JOY_STEER_MAX)
        self._throttle_limit = max(0, min(int(self.get_parameter('throttle_limit').value),
                                          VEH_THROTTLE_MAX))
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._timeout_brake = max(0, min(int(self.get_parameter('timeout_brake').value),
                                         VEH_BRAKE_MAX))
        self._stop_hold_time = max(0.0, float(self.get_parameter('stop_hold_time').value))
        self._enable_throttle_max = max(0, min(int(self.get_parameter('enable_throttle_max').value),
                                               VEH_THROTTLE_MAX))
        self._last_cmd_time = None   # monotonic time of the last /vehicle_control message
        self._stop_state = 'run'     # 'run' | 'stopping' (stage 1) | 'stopped' (latched)
        self._stop_started = None
        self._disconnected = False
        self._closed = False
        self._outbound = SharedLinkOutbound()
        self._inbound = SharedLinkInbound()
        self._udp = UdpConnection(KAIROS_IP, KAIROS_PORT, LOCAL_PORT, self._msg_parser)

        self._gps_pub = self.create_publisher(GpsValues, 'gps_values', 10)

        self._kairos_pub = self.create_publisher(KairosValues, 'kairos_values', 10)

        self._outbound_pub = self.create_publisher(String, 'outbound_msgs', 10)

        self._vehicle_ctrl_sub = self.create_subscription(
            VehicleControl, 'vehicle_control',
            self._vehicle_control_callback, 10)

        self._get_ctrl_srv = self.create_service(
            GetVehicleControl, 'get_vehicle_control',
            self._get_vehicle_control_callback)
        
        self._exec_cmd_srv = self.create_service(
            ExecCmd, 'exec_cmd',
            self._exec_cmd_callback)

        self._field_update_srv = self.create_service(
            FieldUpdate, 'field_update',
            self._field_update_callback)

        self._update_timer = self.create_timer(self._update_rate, self._timer_callback)
        self._inbound_count = 0

        self._connect()

        self.get_logger().info(
            f"Safety: command_timeout {self._command_timeout:.2f} s, stop_hold_time "
            f"{self._stop_hold_time:.2f} s, timeout_brake {self._timeout_brake}, "
            f"enable_throttle_max {self._enable_throttle_max}, "
            f"invert_steering {self._invert_steering}")
        self.get_logger().info("shared_link_bridge_node successfully initialized")

    def shutdown(self):
        """End the SharedLink session and stop the UDP link. Safe to call more than once."""
        if getattr(self, '_closed', True):
            return
        self._closed = True
        try:
            if not self._disconnected:
                self._disconnect()
        finally:
            self._udp.stop()

    def __del__(self):
        self.shutdown()

    # -------------- #
    #      ROS
    # -------------- #
    ### PUBLISHERS
    def _publish_kairos_values(self):
        ros_msg = KairosValues()
        for f in fields(self._inbound):
            sv: SVField = getattr(self._inbound, f.name)
            if sv.value is None:
                continue
            ros_field = INBOUND_TO_ROS.get(f.name, f.name) # Swaps for ROS name, or leaves as is if not on list
            setattr(ros_msg, ros_field, self._ros_inbound_value(f.name, sv.value))
        self._kairos_pub.publish(ros_msg)

    def _publish_gps_values(self):
        ros_msg = GpsValues()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        for inbound_name, ros_name in GPS_FIELDS.items():
            sv: SVField = getattr(self._inbound, inbound_name)
            if sv.value is not None:
                setattr(ros_msg, ros_name, sv.value)
        self._gps_pub.publish(ros_msg)

    ### SUBSCRIBERS
    def _vehicle_control_callback(self, msg: VehicleControl):
        self._last_cmd_time = time.monotonic()
        if self._stop_state == 'stopping':
            return  # let the stop finish; commands are ignored until it latches
        if self._stop_state == 'stopped':
            if msg.veh_motion != 0:
                self.get_logger().warn(
                    'Watchdog stop is latched: release the deadman (veh_motion 0) to re-arm',
                    throttle_duration_sec=2.0)
                return
            self._stop_state = 'run'
            self.get_logger().info('Watchdog stop cleared by veh_motion 0')

        enabling = self._outbound.veh_motion.value == 0 and msg.veh_motion != 0
        for f in fields(self._outbound):
            sv: SVField = getattr(self._outbound, f.name)
            sv.apply(str(getattr(msg, f.name)))

        # Enable interlock: Kairos applies whatever is stored the moment motion comes on.
        if enabling and int(msg.veh_throttle) > self._enable_throttle_max:
            for name in ('veh_motion', 'veh_enable', 'veh_throttle'):
                getattr(self._outbound, name).apply('0')
            self.get_logger().warn(
                f'Motion enable refused: veh_throttle {int(msg.veh_throttle)} > '
                f'{self._enable_throttle_max}. Release the throttle first.',
                throttle_duration_sec=2.0)
        # self.send_data()
        
    ### SERVICES
    def _get_vehicle_control_callback(self, request: GetVehicleControl.Request, response: GetVehicleControl.Response):
        for f in fields(self._outbound):
            sv: SVField = getattr(self._outbound, f.name)
            if sv.value is not None:
                setattr(response.control, f.name, sv.value)
        return response

    def _exec_cmd_callback(self, request: ExecCmd.Request, response: ExecCmd.Response):
        match request.cmd:
            case ExecCmd.Request.CONNECT:
                self._connect()
            case ExecCmd.Request.DISCONNECT:
                self._disconnect()
            case ExecCmd.Request.TELEOP_START:
                self._teleop_start()
            case ExecCmd.Request.TELEOP_STOP:
                self._teleop_stop()
            case ExecCmd.Request.PING:
                self.sendMsg(ping_msg)
                self.get_logger().info("Ping Sent")
            case _:
                self.get_logger().warn(f"Unknown cmd value: {request.cmd}")
        return response

    def _field_update_callback(self, request: FieldUpdate.Request, response: FieldUpdate.Response):
        sv = self.get_outbound_field(request.name)
        if sv is None:
            response.success = False
            return response
        if self._stop_state != 'run' and request.name in (
                'veh_motion', 'veh_enable', 'veh_throttle', 'veh_brake'):
            self.get_logger().warn(f'field_update of {request.name} refused during a watchdog stop')
            response.success = False
            return response
        sv.apply(request.value)

        self.send_data()

        response.success = True        
        return response

    ### TIMER
    def _timer_callback(self):
        self._check_command_timeout()
        self._inbound_count += 1
        if self._inbound_count < int(1.0 / self._update_rate):
            self.send_data()
        else:
            self.send_data(delta_only=False)
            self._inbound_count = 0

    # -------------- #
    #      UDP
    # -------------- #
    ### ON RECEIVE
    def _msg_parser(self, data_raw, addr):
        # was getting error that it couldn't decode xff, but it also wasn't
        if data_raw[1:4] == b'\xff\xd8\xff':
            print("Seems to be a JPEG")
        else:
            data = data_raw.decode("utf-8")

            if data[:5] != "[:AB|":
                print("Data isn't in SharedLink form")
                print(data)
                return

            split_data = data[1:-1].split("|")

            # assume that checksum is always right because sent over UDP
            # assert calcCheckSum(data[1:-3]) == data[-3:-1]

            match split_data[1]:
                case "N0":
                    match split_data[2][0]:
                        case "D":
                            self._apply_inbound_data(split_data[2:-1])
                        case "R":
                            print("Received R type:")
                            print(data)
                        case _:
                            print(f"Received N0 then {split_data[2]}")
                            print(data)
                case "P0":
                    self.sendMsg(pong_msg)
                case "G0":
                    print("Received Pong Message (ping ACK)")
                case _:
                    print(f"Received {split_data[1]} msg: {data}")

    ### SEND

    # Confirm functionality with 
    # tcpdump -i wlp3s0 -n 'udp port 4000'
    def send_data(self, delta_only: bool = True):
        # ICD layout: [:BA|N0|D<v>|D<v>|...|D|C<xx>]. No empty field after N0.
        msg = 'N0'
        for f in fields(self._outbound):
            sv: SVField = getattr(self._outbound, f.name)
            if not delta_only or sv.updated:
                msg += f'|D{self._wire_value(f.name, sv.value)}'
                sv.updated = False
            else:
                msg += '|D'
        self.sendMsg(msg)

    # -------------- #
    #    INTERNALS
    # -------------- #
    ### COMMANDS
    def _connect(self):
        self._disconnected = False
        self.sendMsg(declare_myIP_msg)
        self.sendMsgs(enab_msgs)
        self.sendMsgs(list_SVs_msgs)
        self.get_logger().info("Connection Messages Sent")
        
    def _disconnect(self):
        self._teleop_stop()
        self.sendMsgs(term_msgs)
        self._disconnected = True
        self.get_logger().info("Disconnect Messages Sent")

    def _teleop_start(self):
        self.sendMsg(teleop_start)
        self.get_logger().info("Teleop START Messages Sent")

    def _teleop_stop(self):
        self.sendMsg(teleop_stop)
        self.get_logger().info("Teleop STOP Messages Sent")


    def _wire_value(self, name: str, value):
        """Convert one outbound value from the ROS convention to Kairos's, clamped to the ICD range."""
        if name == 'joy_steer':
            v = -float(value) if self._invert_steering else float(value)
            v = max(-self._steer_limit, min(self._steer_limit, v))
            return v if v != 0 else 0.0   # never send '-0.0'
        if name == 'veh_throttle':
            return max(0, min(int(value), self._throttle_limit))
        if name == 'veh_brake':
            return max(0, min(int(value), VEH_BRAKE_MAX))
        if name == 'veh_shift':
            return max(0, min(int(value), VEH_SHIFT_MAX))
        return value

    def _ros_inbound_value(self, name: str, value):
        """Convert one inbound value from Kairos's convention to the ROS one."""
        if name == 'steerang' and self._invert_steering and isinstance(value, (int, float)):
            return -value if value != 0 else value
        return value

    def _check_command_timeout(self):
        """Two-stage stop if /vehicle_control stops arriving while motion is enabled."""
        now = time.monotonic()
        if self._stop_state == 'stopping':
            if now - self._stop_started >= self._stop_hold_time:
                for name in ('veh_motion', 'veh_enable'):
                    getattr(self._outbound, name).apply('0')
                self._stop_state = 'stopped'
                self.get_logger().warn(
                    'Watchdog stop stage 2: motion disabled. Latched until veh_motion 0.')
            return
        if (self._stop_state == 'stopped' or self._command_timeout <= 0
                or self._last_cmd_time is None or self._outbound.veh_motion.value == 0):
            return
        age = now - self._last_cmd_time
        if age < self._command_timeout:
            return
        # Stage 1: release the throttle and brake while Kairos is still acting on commands.
        for name, value in (('veh_throttle', 0), ('veh_brake', self._timeout_brake)):
            getattr(self._outbound, name).apply(str(value))
        self._stop_state = 'stopping'
        self._stop_started = now
        self.get_logger().warn(
            f'No /vehicle_control for {age:.2f} s with motion enabled. Watchdog stop stage 1: '
            f'throttle 0, brake {self._timeout_brake}, holding {self._stop_hold_time:.2f} s')

    ### BRIDGE TO INTERNAL REP
    def _apply_inbound_data(self, d_fields: list[str]):
        for i, f in enumerate(fields(self._inbound)):
            sv: SVField = getattr(self._inbound, f.name)
            sv.apply(d_fields[i][1:])  # strip leading 'D'
        self._publish_kairos_values()
        self._publish_gps_values()

    ### SEND MSG + HELPERS
    def sendMsg(self, msg: str) -> None:
        msg = self._prep_for_send(msg)
        # UDP first: during shutdown the ROS publish can fail, and it must not stop the
        # frame (e.g. the disconnect messages) from reaching Kairos.
        self._udp.send(msg)
        if rclpy.ok():
            ros_msg = String()
            ros_msg.data = msg
            try:
                self._outbound_pub.publish(ros_msg)
            except Exception:  # context already shutting down
                pass
    
    def sendMsgs(self, msgs: list):
        for msg in msgs:
            self.sendMsg(msg)

    def get_checksum(self, s: str) -> str:
        # Two hex digits, zero-padded, as in the ICD's own frames (e.g. C07).
        return f'{sum(ord(c) for c in s) % 0x100:02X}'
    
    def _prep_for_send(self, msg: str) -> str:
        msg = f":BA|{msg}|C"
        return f"[{msg}{self.get_checksum(msg)}]"
    
    # ACCESS
    def get_outbound_field(self, name: str) -> SVField | None:
        if hasattr(self._outbound, name):
            return getattr(self._outbound, name)
        return None
    


def main(args=None):
    rclpy.init(args=args)
    node = SharedLinkNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()   # end the Kairos session before the node goes away
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
