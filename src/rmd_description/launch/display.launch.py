#!/usr/bin/env python3
"""display.launch.py — Ver el CAD del UGV (rmd_description) y mover sus juntas.

Uso:
    ros2 launch rmd_description display.launch.py               # RViz + sliders
    ros2 launch rmd_description display.launch.py rviz:=false   # sliders, para Foxglove
    ros2 launch rmd_description display.launch.py gui:=false    # juntas fijas en 0

Los sliders (joint_state_publisher_gui) publican /joint_states para las 4
ruedas (track_*) y los 4 flippers (flipper_*). No lanzar junto a flipper_node:
ambos publicarían en /joint_states.

Para Foxglove, en otra terminal:
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
y en el panel 3D: Display frame = base_footprint.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('rmd_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rmd.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
