#!/usr/bin/env python3
"""can_studio.launch.py — Robot (RViz) + monitor CAN por motor + editor de flippers.

Abre en una sola sesión:
    - RViz con el modelo del robot,
    - la ventana "Monitor CAN": panel por motor (tramas separadas y legibles) y
      editor de flippers/orugas para mover el robot y ver sus tramas.
y por debajo el emulador de motores + flipper_node en modo CAN + odometría + URDF.

El editor del monitor publica /cmd_flippers y /cmd_tracks él mismo, así que NO se
lanza joint_state_publisher_gui (evita que dos fuentes peleen por /cmd_flippers):
la ventana del monitor ES el mando. Al mover un flipper en el editor, ves la
articulación girar en RViz y su trama CAN en el panel del motor correspondiente.

    editor de flippers ─▶ /cmd_flippers ─▶ flipper_node ─▶ vcan0 ─▶ motor_emulator
                       ─▶ vcan0 ─▶ flipper_node ─▶ /joint_states ─▶ RViz
                                     │
                         can_monitor ┘  (panel por motor + log crudo)

Requisito previo (una vez por arranque del PC):
    sudo bash src/ugv_bridge/scripts/setup_vcan.sh

Uso:
    ros2 launch ugv_bridge can_studio.launch.py
    ros2 launch ugv_bridge can_studio.launch.py can_canal:=vcan0 frecuencia_hz:=100.0
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
    # view.rviz: Fixed Frame = base_link (robot fijo en el origen). Se usa este y
    # NO flippers.rviz porque este launch corre sin EKF: sin EKF nadie publica el
    # TF odom->base_link, y flippers.rviz (Fixed Frame = odom) dejaría el robot
    # invisible. Aquí el robot no se desplaza; solo articula flippers -> base_link
    # basta. Igual que can_view.launch.py.
    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),

        # Emulador + flipper_node modo can + odometría + robot_state_publisher.
        # Sin RViz propio (use_rviz:=false): lo abrimos aparte con view.rviz.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'can_sim.launch.py')),
            launch_arguments={
                'can_canal': can_canal,
                'frecuencia_hz': LaunchConfiguration('frecuencia_hz'),
                'use_ekf': 'false',
                'use_rviz': 'false',
            }.items(),
        ),

        # RViz con el robot fijo en base_link (siempre visible sin EKF).
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),

        # Ventana monitor por motor + editor de flippers (nodo ROS: publica comandos).
        Node(
            package='ugv_bridge',
            executable='can_monitor',
            name='can_monitor',
            output='screen',
            arguments=['--canal', can_canal],
        ),
    ])
