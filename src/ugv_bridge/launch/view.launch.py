#!/usr/bin/env python3
"""view.launch.py — Visualizar y POSAR el robot: mueve las 4 articulaciones de los
flippers con sliders, sin conducir (robot fijo y centrado en base_link).

Lanza:
  - robot_state_publisher : carga el URDF y publica el TF de cada flipper.
  - joint_state_publisher_gui : ventana con un slider por cada junta revolute
      (los 4 flippers, fl/fr/rl/rr), respetando los límites del URDF (+-60 grados).
  - rviz2 : con config/view.rviz (fixed frame base_link -> el robot no se desplaza).
  - foxglove_bridge : abre el servidor WebSocket en el puerto 8765 para conectar con la GCS.

NO arranca flipper_node, ni EKF, ni odometría: aquí no hay locomoción. La fuente de
/joint_states es el propio joint_state_publisher_gui (por eso flipper_node no debe correr
a la vez: entrarían en conflicto publicando el mismo tópico).

Uso:  ros2 launch ugv_bridge view.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bridge')
    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')

    # Geometría: única fuente de verdad (URDF + guardián de colisiones).
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
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            #Redirige la salida de los sliders a /joint_states_raw, que es lo que lee kinematic_guardian.py
        
            remappings=[('/joint_states', '/joint_states_raw')] 
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        # --- NODO DE FOXGLOVE INTEGRADO ---
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'send_buffer_limit': 100000000, # Buffer ampliado para modelos 3D pesados
            }]
        ),
        Node(
            package='ugv_bridge',
            executable='kinematic_guardian',
            name='kinematic_guardian',
            output='screen',
            # Misma geometría que el URDF: el guardián protege exactamente
            # contra la forma que se está dibujando.
            parameters=[geometria_yaml],
        ),

    ])