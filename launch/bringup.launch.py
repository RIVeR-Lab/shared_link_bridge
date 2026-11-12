from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_xterm = LaunchConfiguration('teleop_xterm')

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop_xterm',
            default_value='true',
            description='Start controller_teleop in its own xterm. false = leave it out of '
                        'the launch and run it yourself in a terminal: '
                        'ros2 run shared_link_bridge controller_teleop.py',
        ),
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
            condition=IfCondition(teleop_xterm),
        ),
    ])
