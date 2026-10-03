"""Bring up the golf ball collector robot.

  ros2 launch golf_bot_bringup bringup.launch.py                       # everything
  ros2 launch golf_bot_bringup bringup.launch.py serial_port:=/dev/ttyACM0
  ros2 launch golf_bot_bringup bringup.launch.py use_agent:=false      # test joystick without ESP32
  ros2 launch golf_bot_bringup bringup.launch.py use_vision:=true      # + golf ball detection / AUTO (Y)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('golf_bot_bringup'), 'config', 'golf_bot.yaml')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_agent = LaunchConfiguration('use_agent')
    use_vision = LaunchConfiguration('use_vision')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='ESP32 serial port'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('use_agent', default_value='true',
                              description='Start the micro-ROS agent for the ESP32'),
        DeclareLaunchArgument('use_vision', default_value='false',
                              description='Start golf ball detector + ball chaser (AUTO mode)'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[params], output='screen'),

        Node(package='golf_bot_control', executable='xbox_teleop', name='xbox_teleop',
             parameters=[params], output='screen'),

        Node(package='golf_bot_control', executable='wheel_odometry', name='wheel_odometry',
             parameters=[params], output='screen'),

        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
             arguments=['serial', '--dev', serial_port, '-b', baud_rate],
             condition=IfCondition(use_agent), output='screen'),

        Node(package='golf_bot_vision', executable='golf_ball_detector', name='golf_ball_detector',
             parameters=[params], condition=IfCondition(use_vision), output='screen'),

        Node(package='golf_bot_vision', executable='ball_chaser', name='ball_chaser',
             parameters=[params], condition=IfCondition(use_vision), output='screen'),
    ])
