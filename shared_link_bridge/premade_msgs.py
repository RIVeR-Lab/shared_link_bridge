# Had to give myself 192.168.200.x subnet alias as kairos is there and can't route to 192.168.0.x I guess
# Command I used was
# sudo ifconfig en0 alias 192.168.200.50 255.255.255.0

import socket

KAIROS_IP = "192.168.200.220"

KAIROS_PORT = 4000
LOCAL_PORT = 4220


def get_local_ip(target_ip=KAIROS_IP, target_port=KAIROS_PORT):
    """Determine the local IP of the interface that routes to KAIROS.

    Opens a UDP socket "connected" to KAIROS (no packets are actually sent)
    so the OS picks the correct source interface, then reads back its address.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((target_ip, target_port))
        IP = s.getsockname()[0]
        print(f"Detected IP: {IP}")
        return IP
    finally:
        s.close()


LOCAL_IP = get_local_ip()

declare_myIP_msg = f"ELink|D0, SetIP, {LOCAL_IP}:{LOCAL_PORT}, 4000"

enab_msgs = [
    "ELink|D0, outbound, on",
    "ELink|D0, inbound, on"
]

list_SVs_msgs = [
    "MUI",
    "MUO",
    "N0|T0|Qvideo_mux1|T0|Qveh_start|T0|Qveh_enable|T0|Qveh_motion|T2|Qjoy_steer|T0|Qveh_steer|T0|Qveh_brake|T0|Qveh_throttle|T0|Qveh_shift|T0|Qveh_clutch",
    "L0|T2|QAutoGpsHead|T2|QAutoGpsLat|T2|QAutoGpsLon|T2|QAutoGpsVel|T0|QAutoGpsReferences|T0|QAutoGpsQuality|T2|QAutoGpsStamp|T0|Qpause|T0|Qkey_on|T0|Qeng_enb|T0|Qpe7|T0|Qbbrake|T0|Qfbrake|T2|Qsteerang|T0|Qthrottle|Qsvp_enb_eng|T0|Qserialpck|T0|Qveh_gear|T0|Qveh_state|T0|Qveh_estop|T0|Qveh_obstacle|T2|Qvehicle_bat|T4|Qrun_composite|T4|QVehicleName|T0|QHealth_total|T0|Qrpm|T0|Qfuel|T2|Qsys_thd",
    # "MW" (write the lists to the P4S4's INI file) is deliberately not sent. The bridge sets
    # both lists on every connect, so there is nothing to persist, and saved lists could
    # change what Shepherd sees. A missing comma used to glue "MW" onto the L0 string above,
    # so MW was never sent and the last variable was requested as "sys_thdMW".
    "ML"
]
# ^^^^^
# Should respond with something like
# [:AB|N0|Rvideo_mux1|D0|Rveh_start|D0|Rveh_enable|D0|Rveh_motion|D0|Rjoy_steer|D0|Rveh_steer|D0|Rveh_brake|D100|Rveh_throttle|D0|Rveh_shift|D0|Rveh_clutch|D0|C92]
# Not sure if it'll also have that for the quires for KAIROS's outbound variables too 
# this might even be unnecessary and just used to confirm that it is getting the changes

term_msgs = [
    "ELink|D0, outbound, off",
    "ELink|D0, inbound, off"
]

ping_msg = "P0"
pong_msg = "G0"

teleop_start = "EServoPod|D0,TELEOP START"
teleop_stop = "EServoPod|D0,TELEOP STOP"