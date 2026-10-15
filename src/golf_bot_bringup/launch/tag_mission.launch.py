"""AprilTag tour mission on the golf ball robot.

  ros2 launch golf_bot_bringup tag_mission.launch.py                    # real robot
  ros2 launch golf_bot_bringup tag_mission.launch.py serial_port:=/dev/ttyACM0
  ros2 launch golf_bot_bringup tag_mission.launch.py simulate:=true use_agent:=false
                                                                        # dry run, no hardware

Press Y on the joystick (AUTO) to start: scan 360 deg -> plan -> visit every tag.
LB = manual override, B = emergency stop, Y again = pause / resume.
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

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('use_agent', default_value='true',
                              description='Start the micro-ROS agent for the ESP32'),
        DeclareLaunchArgument('simulate', default_value='false',
                              description='Simulated camera and tags (sim_tags in tag_mission.yaml)'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[robot_params], output='screen'),

        Node(package='golf_bot_control', executable='xbox_teleop', name='xbox_teleop',
             parameters=[robot_params], output='screen'),

        Node(package='golf_bot_control', executable='wheel_odometry', name='wheel_odometry',
             parameters=[robot_params], output='screen'),

        Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
             arguments=['serial', '--dev', serial_port, '-b', baud_rate],
             condition=IfCondition(use_agent), output='screen'),

        Node(package='golf_bot_tags', executable='tag_mission', name='tag_mission',
             parameters=[mission_params,
                         {'simulate': ParameterValue(simulate, value_type=bool)}],
             output='screen'),
    ])
