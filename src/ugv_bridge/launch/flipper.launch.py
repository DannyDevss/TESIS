#!/usr/bin/env python3
"""flipper.launch.py — Levanta el puente de motores (flipper_node), la odometría de
orugas, el modelo del robot (robot_state_publisher) y, opcionalmente, el EKF y RViz.

Uso:
    ros2 launch ugv_bridge flipper.launch.py
    ros2 launch ugv_bridge flipper.launch.py frecuencia_hz:=200.0 modo_simulacion:=true
    ros2 launch ugv_bridge flipper.launch.py modo:=can           # bus vcan0 + emulador
    ros2 launch ugv_bridge flipper.launch.py use_ekf:=true      # + ekf_filter_node
    ros2 launch ugv_bridge flipper.launch.py use_rviz:=true     # + RViz con el modelo

Argumentos:
    frecuencia_hz    (100.0) frecuencia del bucle de control de flipper_node.
    modo             ('')    'gemelo' | 'can' | 'pi3hat'. Vacío = usar modo_simulacion.
    modo_simulacion  (true)  true = gemelo digital; false = pi3hat/CAN real.
    can_canal        (vcan0) interfaz SocketCAN para modo:=can.
    use_ekf          (false) si true, arranca robot_localization con config/ekf.yaml.
    use_rviz         (false) si true, abre RViz con config/flippers.rviz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    frecuencia_hz = LaunchConfiguration('frecuencia_hz')
    modo = LaunchConfiguration('modo')
    modo_simulacion = LaunchConfiguration('modo_simulacion')
    can_canal = LaunchConfiguration('can_canal')
    use_ekf = LaunchConfiguration('use_ekf')
    use_rviz = LaunchConfiguration('use_rviz')

    pkg_share = get_package_share_directory('ugv_bridge')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'flippers.rviz')

    # Geometría: única fuente de verdad, compartida por el URDF y los nodos.
    geometria_yaml = os.path.join(pkg_share, 'config', 'geometria_robot.yaml')
    xacro_path = os.path.join(pkg_share, 'urdf', 'ugv.urdf.xacro')
    # Se procesa el xacro en caliente: el modelo nunca queda desfasado del YAML.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' geometria:=', geometria_yaml]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),
        DeclareLaunchArgument('modo', default_value=''),
        DeclareLaunchArgument('modo_simulacion', default_value='true'),
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        # Puente ROS <-> motores (orugas + flippers) + IMU.
        Node(
            package='ugv_bridge',
            executable='flipper_node',
            name='flipper_node',
            output='screen',
            parameters=[geometria_yaml, {
                'frecuencia_hz': frecuencia_hz,
                'modo': modo,
                'modo_simulacion': modo_simulacion,
                'can_canal': can_canal,
            }],
        ),

        # Odometría de orugas: /joint_states -> /odom (odom0 del EKF).
        # publish_tf=false: con EKF activo, el TF odom->base_link lo emite el EKF.
        Node(
            package='ugv_bridge',
            executable='track_odometry_node',
            name='track_odometry_node',
            output='screen',
            parameters=[geometria_yaml, {'publish_tf': False}],
        ),

        # Modelo del robot: lee /joint_states y publica el TF de cada flipper.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Fusión sensorial EKF (opcional). Requiere: apt install ros-jazzy-robot-localization
        Node(
            condition=IfCondition(use_ekf),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),

        # Visualización (opcional). Requiere: apt install ros-jazzy-rviz2
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
