#!/usr/bin/env python3

from enum import Enum

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from shared_link_bridge.srv import SetEStopState

from UDPConnection import UdpSender


# ESTOP_BROADCAST_ADDR = '192.168.200.255'
ESTOP_BROADCAST_ADDR = '255.255.255.255'
ESTOP_PORT = 7001
ESTOP_LOCAL_PORT = 7000

class EStopState(Enum):
    ESTOP = 0
    PAUSE = 1
    RUN = 2

_ESTOP_PKT = bytes([
    0x24, 0x28, 0xB2, 0xAA, 0x11, 0x27, 0xA2, 0x0B, 0xFD, 0xDF,
    0x00, 0x00, 0x55, 0xAA, 0x00, 0x00, 0x2A, 0x2A, 0x44, 0x4F,
    0x5A, 0x45, 0x52, 0x5F, 0x42, 0x41, 0x53, 0x45, 0x2A, 0x2A,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x0D, 0x0A,
])

_PAUSE_PKT = bytes([
    0x24, 0x28, 0xB2, 0xAA, 0x11, 0x27, 0xA2, 0x0B, 0xFD, 0xDF,
    0x00, 0x00, 0xAA, 0x55, 0x00, 0x00, 0x2A, 0x2A, 0x44, 0x4F,
    0x5A, 0x45, 0x52, 0x5F, 0x42, 0x41, 0x53, 0x45, 0x2A, 0x2A,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x01, 0x5D, 0x0D, 0x0A,
])

_RUN_PKT = bytes([
    0x24, 0x28, 0xB2, 0xAA, 0x11, 0x27, 0xA2, 0x0B, 0xFD, 0xDF,
    0x00, 0x00, 0xAA, 0x55, 0x00, 0x00, 0x2A, 0x2A, 0x44, 0x4F,
    0x5A, 0x45, 0x52, 0x5F, 0x42, 0x41, 0x53, 0x45, 0x2A, 0x2A,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x0D, 0x0A,
])

_STATE_TO_PKT = {
    EStopState.ESTOP: _ESTOP_PKT,
    EStopState.PAUSE: _PAUSE_PKT,
    EStopState.RUN:   _RUN_PKT,
}


class EStopBeaconNode(Node):
    def __init__(self):
        super().__init__('estop_beacon')

        self._state = EStopState.RUN  # CHANGE TO ESTOP AFTER TESTING

        self._udp = UdpSender(ESTOP_BROADCAST_ADDR, ESTOP_PORT, local_port=ESTOP_LOCAL_PORT, broadcast=True)

        self._set_state_srv = self.create_service(
            SetEStopState, 'set_estop_state',
            self._set_state_callback)

        # Rule 1: transmit 1 per second
        self._beacon_timer = self.create_timer(1.0, self._beacon_timer_cb)

        self.get_logger().info(f"EStop beacon initialized in {self._state} mode")

    def _set_state_callback(self, request: SetEStopState.Request, response: SetEStopState.Response):
        self._state = EStopState(request.state)
        self._send_beacon()  # send immediately on state change
        self.get_logger().info(f'EStop state set to {self._state.name}')
        return response

    def _beacon_timer_cb(self):
        self._send_beacon()

    def _send_beacon(self):
        try:
            for _ in range(3):
                self._udp.send_bytes(_STATE_TO_PKT[self._state])
        except OSError as e:
            self.get_logger().warn(f"Beacon send failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EStopBeaconNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
