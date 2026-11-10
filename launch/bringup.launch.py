from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='shared_link_bridge',
            executable='estop_beacon.py',
            name='estop_beacon',
            output='screen',
        ),
        Node(
            package='shared_link_bridge',
            executable='shared_link_bridge_node.py',
            name='shared_link_bridge_node',
            output='screen',
        ),
        Node(
            package='shared_link_bridge',
            executable='controller_teleop.py',
            name='controller_teleop',
            output='screen',
            prefix='xterm -hold -e',   # see note below
        ),
    ])