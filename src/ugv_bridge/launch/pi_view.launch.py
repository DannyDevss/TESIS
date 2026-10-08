#!/usr/bin/env python3
"""pi_view.launch.py — Versión headless mínima para la Raspberry Pi.

Solo nodos SIN interfaz gráfica: robot_state_publisher, foxglove_bridge y la IMU
del pi3hat. La Pi no tiene escritorio y Foxglove corre en el PC, conectándose a
ws://<ip-de-la-pi>:8765. Igual que en el resto del proyecto: no hay RViz ni
ninguna otra ventana, aquí ni en ningún lado.

ATENCIÓN: esto NO levanta el driver de motores. Es el esqueleto para mirar el
modelo y la IMU. Para el sistema completo en la Pi usar los comandos
`compilar_simu` / `compilar_real` (scripts/tesis_lanzar.sh), que lanzan
flipper.launch.py con EKF y Foxglove.

El nodo `kinematic_guardian` se eliminó del proyecto (ya no es necesario), así
que este launch tampoco lo arranca.

Uso:
    ros2 launch ugv_bridge pi_view.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bridge')

    # Geometría: única fuente de verdad (URDF + IMU).
    geometria_yaml = os.path.join(pkg_share, 'config', 'geometria_robot.yaml')
    xacro_path = os.path.join(pkg_share, 'urdf', 'ugv.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' geometria:=', geometria_yaml]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Única visualización del proyecto. send_buffer_limit ampliado: las
        # mallas del modelo son pesadas y con el límite por defecto el puente
        # corta la conexión justo al mandar el modelo.
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
            executable='pi3hat_imu',
            name='pi3hat_imu_node',
            output='screen',
            parameters=[geometria_yaml],
        ),
    ])
