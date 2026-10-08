#!/usr/bin/env python3
"""can_view.launch.py — Conducir el robot simulado SOLO desde Foxglove.

Simulación CAN completa (motor_emulator + flipper_node en modo can + odometría +
EKF + URDF) con el puente de Foxglove encima. Mover algo desde Foxglove ejercita
el protocolo CAN entero en tiempo real:

    Foxglove [Teleop]  -> /teleop/flipper_* -> teleop_flippers -> /cmd_flippers
    Foxglove [Publish] -> /cmd_flippers | /cmd_tracks
                              |
                              v
        flipper_node -> vcan0 (tramas ODrive) -> motor_emulator
                     <- vcan0 (encoder) -> /joint_states -> panel 3D

POR QUÉ YA NO HAY SLIDERS NI RVIZ
---------------------------------
Este launch abría `joint_state_publisher_gui` (ventana Qt de sliders) y, a
través de `gui_a_comandos`, publicaba /cmd_flippers. Opcionalmente abría RViz.
Eso daba DOS mandos vivos a la vez (los sliders y Foxglove) sobre el mismo
tópico: como flipper_node retiene el último comando recibido, el flipper se iba
al valor de quien hablara último y parecía que se movía solo. Además las
ventanas Qt sobrevivían al Ctrl+C del launch y seguían publicando en segundo
plano, lo que hacía el síntoma intermitente e imposible de atribuir.

Ahora el ÚNICO mando es Foxglove. `gui_a_comandos` sigue en el paquete por si
alguna vez se quiere volver a los sliders, pero NINGÚN launch lo arranca.

Lo que en Foxglove reemplaza a cada slider:
    - panel *Teleop* sobre /teleop/flipper_fl|fr|rl|rr  (mantener pulsado = girar)
    - panel *Publish* sobre /cmd_flippers con una pose fija (0°, 45°, 90°...)
    - panel *Publish* sobre /cmd_tracks para las orugas (velocidad PERSISTENTE:
      hay que frenar mandando [0,0,0,0])
Todo eso ya viene configurado en config/ugv_control_v4.json.

Requisito previo (una vez por arranque del PC):
    sudo bash src/ugv_bridge/scripts/setup_vcan.sh

Uso:
    ros2 launch ugv_bridge can_view.launch.py
    ros2 launch ugv_bridge can_view.launch.py can_canal:=vcan0 frecuencia_hz:=100.0
    ros2 launch ugv_bridge can_view.launch.py use_ekf:=false

Luego, en Foxglove: conectar a ws://localhost:8765 y cargar el layout
`ugv_control_v4`. Panel 3D con **Display frame = odom**.

Argumentos:
    imu_externa  (false) true = la IMU real llega de la Raspberry; la sintética
                         se aparta a /imu/sintetica (ver flipper.launch.py).
    use_ekf      (true)  publica odom -> base_link; sin él el chasis NO se inclina.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),
        DeclareLaunchArgument('imu_externa', default_value='false'),
        DeclareLaunchArgument('motores_presentes', default_value=''),
        DeclareLaunchArgument('use_ekf', default_value='true'),

        # Emulador + flipper_node modo can + odometría + robot_state_publisher
        # + teleop_flippers + foxglove_bridge (8765). Un solo puente, aquí.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'can_sim.launch.py')),
            launch_arguments={
                'can_canal': LaunchConfiguration('can_canal'),
                'frecuencia_hz': LaunchConfiguration('frecuencia_hz'),
                'imu_externa': LaunchConfiguration('imu_externa'),
                'motores_presentes': LaunchConfiguration('motores_presentes'),
                'use_ekf': LaunchConfiguration('use_ekf'),
                'use_foxglove': 'true',
                'use_teleop': 'true',
            }.items(),
        ),
    ])
