#!/usr/bin/env python3
"""can_sim.launch.py — Simulación de bus CAN completa: emulador de motores + sistema.

Levanta `motor_emulator` (los 8 motores al otro lado de vcan0) y encima el sistema
completo de flipper.launch.py con modo:=can, de forma que flipper_node hable con
los motores mediante TRAMAS CAN REALES sobre la interfaz virtual.

Requisito previo (una vez por arranque del PC, en el HOST o dentro de rosdev):
    sudo bash src/ugv_bridge/scripts/setup_vcan.sh

Uso:
    ros2 launch ugv_bridge can_sim.launch.py
    ros2 launch ugv_bridge can_sim.launch.py use_rviz:=true use_ekf:=true
    ros2 launch ugv_bridge can_sim.launch.py can_canal:=vcan0 frecuencia_hz:=100.0

Depurar el tráfico del bus en otra terminal:  candump vcan0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_canal = LaunchConfiguration('can_canal')
    pkg_share = get_package_share_directory('ugv_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        # Los 8 motores emulados escuchando el bus (proceso sin ROS).
        Node(
            package='ugv_bridge',
            executable='motor_emulator',
            name='motor_emulator',
            output='screen',
            arguments=['--canal', can_canal],
        ),

        # Sistema completo (flipper_node + odometría + robot_state_publisher...)
        # con el driver en modo CAN. flipper_node tolera arrancar antes que el
        # emulador: solo acumula timeouts hasta que este responda.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'flipper.launch.py')),
            launch_arguments={
                'modo': 'can',
                'can_canal': can_canal,
                'frecuencia_hz': LaunchConfiguration('frecuencia_hz'),
                'use_ekf': LaunchConfiguration('use_ekf'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
    ])
