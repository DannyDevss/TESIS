#!/usr/bin/env python3
"""can_view.launch.py — Sliders + RViz (view.rviz) sobre la simulación CAN real.

Une view.launch.py con can_sim.launch.py sin el conflicto de /joint_states:
aquí los sliders NO publican /joint_states; se remapean a /gui/joint_states y
`gui_a_comandos` los convierte en /cmd_flippers. Lo que RViz dibuja es lo que
flipper_node lee de vuelta del bus vcan0 (motor_emulator respondiendo), así que
mover un slider ejercita el protocolo CAN completo en tiempo real.

    slider -> /cmd_flippers -> flipper_node -> vcan0 -> motor_emulator
           -> vcan0 -> flipper_node -> /joint_states -> RViz

Requisito previo (una vez por arranque del PC):
    sudo bash src/ugv_bridge/scripts/setup_vcan.sh

Uso:
    ros2 launch ugv_bridge can_view.launch.py
    ros2 launch ugv_bridge can_view.launch.py can_canal:=vcan0 frecuencia_hz:=100.0

Las orugas se siguen comandando por /cmd_tracks (los sliders track_* no hacen
nada: son juntas de velocidad). Espiar el bus:  candump vcan0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bridge')
    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),

        # Emulador + flipper_node modo can + odometría + robot_state_publisher.
        # Sin RViz propio: aquí se abre con view.rviz (robot fijo en base_link).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'can_sim.launch.py')),
            launch_arguments={
                'can_canal': LaunchConfiguration('can_canal'),
                'frecuencia_hz': LaunchConfiguration('frecuencia_hz'),
                'use_ekf': 'false',
                'use_rviz': 'false',
            }.items(),
        ),

        # Sliders remapeados: publican /gui/joint_states, NO /joint_states
        # (ese tópico es de flipper_node; ver view.launch.py sobre el conflicto).
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            remappings=[('joint_states', 'gui/joint_states')],
        ),

        # Puente: posiciones de los sliders -> /cmd_flippers.
        Node(
            package='ugv_bridge',
            executable='gui_a_comandos',
            name='gui_a_comandos',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
