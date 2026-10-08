#!/usr/bin/env python3
"""view.launch.py — Posar los flippers (sin conducir) desde Foxglove.

QUÉ ERA Y POR QUÉ CAMBIÓ
------------------------
Este launch abría RViz + la ventana Qt de sliders (`joint_state_publisher_gui`)
y era la fuente de /joint_states. Dos problemas, los dos resueltos aquí:

 1. DOS MANDOS A LA VEZ. La GUI de sliders publicaba por su cuenta, y Foxglove
    publicaba /cmd_flippers. Como las dos cadenas acaban en las mismas juntas,
    el robot saltaba entre el valor del slider y el de Foxglove. Peor: las
    ventanas Qt sobreviven con frecuencia al Ctrl+C del launch y siguen
    publicando en segundo plano, de modo que el síntoma aparecía incluso
    "sin nada abierto".
 2. ARRANCABA UN NODO QUE YA NO EXISTE. Lanzaba `kinematic_guardian`, que se
    eliminó del proyecto. El launch moría al no encontrar el ejecutable.

Ahora no hay sliders, no hay RViz y no hay guardián: este launch es un atajo a
flipper.launch.py en modo GEMELO DIGITAL con el puente de Foxglove. El
gemelo mueve las juntas sin hardware ni bus CAN, que es exactamente lo que se
quería para "posar y mirar".

CÓMO SE POSA AHORA EL ROBOT (todo en Foxglove)
----------------------------------------------
    panel *Publish* -> /cmd_flippers  con una pose fija ([0,0,0,0], [0.785 x4]...)
    panel *Teleop*  -> /teleop/flipper_fl|fr|rl|rr  (mantener pulsado = girar)
El layout config/ugv_control_v4.json ya trae esos paneles.

Uso:
    ros2 launch ugv_bridge view.launch.py
    ros2 launch ugv_bridge view.launch.py use_ekf:=true   # + inclinación del chasis

Conectar Foxglove a ws://localhost:8765. Panel 3D con Display frame = odom
(con base_link el chasis queda clavado y solo se ven girar los flippers).

Para la simulación con tramas CAN de verdad:  can_view.launch.py
Para la simulación CAN + espía del bus:       can_studio.launch.py
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
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),
        DeclareLaunchArgument('use_ekf', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'flipper.launch.py')),
            launch_arguments={
                # Gemelo digital: sin pi3hat, sin bus, sin emulador. Las juntas
                # siguen la consigna directamente.
                'modo': 'gemelo',
                'frecuencia_hz': LaunchConfiguration('frecuencia_hz'),
                'use_ekf': LaunchConfiguration('use_ekf'),
                'use_foxglove': 'true',
                'use_teleop': 'true',
            }.items(),
        ),
    ])
