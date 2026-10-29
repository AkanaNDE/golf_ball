"""AprilTag tour mission WITHOUT joystick: starts by itself after a countdown.

  ros2 launch golf_bot_bringup tag_mission_auto.launch.py
  ros2 launch golf_bot_bringup tag_mission_auto.launch.py start_delay:=10.0
  ros2 launch golf_bot_bringup tag_mission_auto.launch.py simulate:=true use_agent:=false

The robot starts moving `start_delay` seconds after launch (default 5 s).
Stop / pause:   ros2 topic pub --once /tag_mission/stop std_msgs/msg/Bool "{data: true}"
Resume:         ros2 topic pub --once /tag_mission/stop std_msgs/msg/Bool "{data: false}"
Ctrl+C or the emergency switch also stop it (the ESP32 stops when /cmd_vel goes silent).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('golf_bot_bringup'), 'config')
    robot_params = os.path.join(config, 'golf_bot.yaml')
    mission_params = os.path.join(config, 'tag_mission.yaml')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_agent = LaunchConfiguration('use_agent')
    simulate = LaunchConfiguration('simulate')
    start_delay = LaunchConfiguration('start_delay')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('use_agent', default_value='true',
                              description='Start the micro-ROS agent for the ESP32'),
        DeclareLaunchArgument('simulate', default_value='false',
                              description='Simulated camera and tags (sim_tags in tag_mission.yaml)'),
        DeclareLaunchArgument('start_delay', default_value='5.0',
                              description='Seconds before the robot starts moving'),

        Node(package='golf_bot_control', executable='wheel_odometry', name='wheel_odometry',
             parameters=[robot_params], output='screen'),

        # respawn: if the ESP32 port cannot be opened (unplugged, USB chip stuck) or drops
        # out, the agent restarts every 2 s instead of staying dead for the whole run
        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
             arguments=['serial', '--dev', serial_port, '-b', baud_rate],
             respawn=True, respawn_delay=2.0,
             condition=IfCondition(use_agent), output='screen'),

        # No teleop in between: the mission drives /cmd_vel directly
        Node(package='golf_bot_tags', executable='tag_mission', name='tag_mission',
             parameters=[mission_params, {
                 'require_auto': False,
                 'simulate': ParameterValue(simulate, value_type=bool),
                 'start_delay': ParameterValue(start_delay, value_type=float),
             }],
             remappings=[('cmd_vel_auto', 'cmd_vel')],
             output='screen'),
    ])
