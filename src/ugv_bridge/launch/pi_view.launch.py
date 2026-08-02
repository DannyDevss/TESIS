#!/usr/bin/env python3
"""pi_view.launch.py — Version headless para correr en la Raspberry Pi.
Solo nodos sin interfaz gráfica: robot_state_publisher, foxglove_bridge,
 pi3hat_imu,
kinematic_guardian. Los nodos con GUI (joint_state_publisher_gui, rviz2)
corren aparte en la laptop, ver laptop_view.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bridge')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ugv.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'send_buffer_limit': 100000000,
            }]
        ),
        Node(
            package='ugv_bridge',
            executable='kinematic_guardian',
            name='kinematic_guardian',
            output='screen',
        ),
        Node(
            package='ugv_bridge',
            executable='pi3hat_imu',
            name='pi3hat_imu_node',
            output='screen',
        ),
    ])
