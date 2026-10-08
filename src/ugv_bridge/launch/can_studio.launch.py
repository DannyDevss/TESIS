#!/usr/bin/env python3
"""can_studio.launch.py — can_view + el espía del bus CAN, todo dentro de Foxglove.

Lo mismo que can_view.launch.py (simulación CAN completa + puente Foxglove) más
`can_monitor`, que escucha vcan0 y vuelca lo que ve en tópicos ROS. Es el
"candump con panel" del proyecto, pero sin ventana propia: se mira en Foxglove.

    Foxglove (Teleop/Publish) -> /cmd_* -> flipper_node -> vcan0 -> motor_emulator
                                                             |
                                                 can_monitor | (solo escucha)
                                                             v
                            /diagnostics (filas can/...) + /can/* -> Foxglove

QUÉ CAMBIÓ RESPECTO A LA VERSIÓN ANTERIOR
-----------------------------------------
Antes este launch abría DOS ventanas: RViz y la ventana Qt de can_monitor, que
además de mirar el bus publicaba /cmd_flippers y /cmd_tracks desde sus sliders.
Con Foxglove abierto en paralelo había dos mandos compitiendo por los mismos
tópicos y los comandos se pisaban (flipper_node retiene el último que llega).
Y como la ventana Qt no siempre moría con el Ctrl+C del launch, podía quedar
publicando en segundo plano sin nada visible que lo delatara.

Hoy no hay ninguna ventana: RViz fuera, Qt fuera, y `can_monitor` es un espía de
SOLO LECTURA sin un solo publicador de comandos. El mando es Foxglove y nada más.

QUÉ MIRAR EN FOXGLOVE
---------------------
  Diagnostics – Summary  sobre /diagnostics
        filas `can/<junta>`  -> lo que REALMENTE circula por el cable
        filas `motores/...`  -> lo que CREE el driver (flipper_node)
        fila  `can/BUS`      -> tramas/s y cuántos motores emiten
        Si esas dos vistas discrepan, el problema está entre driver y bus
        (interfaz caída, bitrate mal puesto, node_id equivocado).
  Plot          /can/velocidad_rad_s.data[0]  (y posicion_rad, iq_a, torque_nm)
                índices 0..3 = orugas fl/fr/rl/rr, 4..7 = flippers fl/fr/rl/rr
  Gauge o Plot  /can/tasa_tramas_hz   -> si cae a 0, el bus murió
  Raw Messages  /can/trafico.data     -> las últimas tramas en texto

Requisito previo (una vez por arranque del PC):
    sudo bash src/ugv_bridge/scripts/setup_vcan.sh

Uso:
    ros2 launch ugv_bridge can_studio.launch.py
    ros2 launch ugv_bridge can_studio.launch.py can_canal:=vcan0 frecuencia_hz:=100.0
    ros2 launch ugv_bridge can_studio.launch.py publicar_trafico:=false  # bus cargado

Luego conectar Foxglove a ws://localhost:8765 con el layout `ugv_control_v4`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    can_canal = LaunchConfiguration('can_canal')
    pkg_share = get_package_share_directory('ugv_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),
        DeclareLaunchArgument('motores_presentes', default_value=''),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        # /can/trafico es texto y a 10 Hz pesa; en un bus real muy cargado o con
        # el puente por wifi conviene apagarlo y quedarse con /diagnostics.
        DeclareLaunchArgument('publicar_trafico', default_value='true'),

        # Simulación CAN completa + puente Foxglove (una sola vez, allí dentro).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'can_view.launch.py')),
            launch_arguments={
                'can_canal': can_canal,
                'frecuencia_hz': LaunchConfiguration('frecuencia_hz'),
                'motores_presentes': LaunchConfiguration('motores_presentes'),
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),

        # Espía del bus. SOLO LEE: no publica /cmd_flippers ni /cmd_tracks, así
        # que no puede pelearse con Foxglove por el mando.
        Node(
            package='ugv_bridge',
            executable='can_monitor',
            name='can_monitor',
            output='screen',
            parameters=[{
                'canal': can_canal,
                # El argumento del launch llega como TEXTO ("true"). El nodo
                # declara el parámetro como bool, y pasarle el texto crudo lo
                # tumba con un error de tipo: hay que convertirlo aquí.
                'publicar_trafico': ParameterValue(
                    LaunchConfiguration('publicar_trafico'), value_type=bool),
            }],
        ),
    ])
