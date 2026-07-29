#!/usr/bin/env python3
from typing import Union

from enum import Enum
from dataclasses import dataclass, field, fields

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_link_bridge.msg import GpsValues, KairosValues, VehicleControl
from shared_link_bridge.srv import GetVehicleControl, ExecCmd, FieldUpdate

from UDPConnection import UdpConnection
from premade_msgs import *


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
        self._update_rate = self.get_parameter('update_rate').value
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

        self.get_logger().info("shared_link_bridge_node successfully initialized")

    def __del__(self):
        self._disconnect()
        self._udp.stop()

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
            setattr(ros_msg, ros_field, sv.value)
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
        for f in fields(self._outbound):
            sv: SVField = getattr(self._outbound, f.name)
            sv.apply(str(getattr(msg, f.name)))
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
        sv.apply(request.value)

        self.send_data()

        response.success = True        
        return response

    ### TIMER
    def _timer_callback(self):
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
                    print(f"Received {split_data[1]} msg")

    ### SEND

    # Confirm functionality with 
    # tcpdump -i wlp3s0 -n 'udp port 4000'
    def send_data(self, delta_only: bool = True):
        msg = 'N0|'
        for f in fields(self._outbound):
            sv: SVField = getattr(self._outbound, f.name)
            if not delta_only or sv.updated:
                msg += f'|D{sv.value}'
                sv.updated = False
            else:
                msg += '|D'
        self.sendMsg(msg)

    # -------------- #
    #    INTERNALS
    # -------------- #
    ### COMMANDS
    def _connect(self):
        self.sendMsg(declare_myIP_msg)
        self.sendMsgs(enab_msgs)
        self.sendMsgs(list_SVs_msgs)
        self.get_logger().info("Connection Messages Sent")
        
    def _disconnect(self):
        self._teleop_stop()
        self.sendMsgs(term_msgs)
        self.get_logger().info("Disconnect Messages Sent")

    def _teleop_start(self):
        self.sendMsg(teleop_start)
        self.get_logger().info("Teleop START Messages Sent")

    def _teleop_stop(self):
        self.sendMsg(teleop_stop)
        self.get_logger().info("Teleop STOP Messages Sent")


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
        ros_msg = String()
        ros_msg.data = msg
        self._outbound_pub.publish(ros_msg)
        self._udp.send(msg)
    
    def sendMsgs(self, msgs: list):
        for msg in msgs:
            self.sendMsg(msg)

    def get_checksum(self, s: str) -> str:
        return f'{sum(ord(c) for c in s) % 0x100:X}'
    
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
    rclpy.spin(SharedLinkNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
