#!/usr/bin/env python3
"""flipper.launch.py — Levanta el puente de motores (flipper_node), la odometría de
orugas, el modelo del robot (robot_state_publisher) y, opcionalmente, el EKF y RViz.

Uso:
    ros2 launch ugv_bridge flipper.launch.py
    ros2 launch ugv_bridge flipper.launch.py frecuencia_hz:=200.0 modo_simulacion:=true
    ros2 launch ugv_bridge flipper.launch.py modo:=can           # bus vcan0 + emulador
    ros2 launch ugv_bridge flipper.launch.py use_ekf:=true      # + ekf_filter_node
    ros2 launch ugv_bridge flipper.launch.py use_foxglove:=false  # sin puente (ya hay uno)

VISUALIZACIÓN: SOLO FOXGLOVE
----------------------------
Este launch no abre ninguna ventana. RViz y la GUI de sliders se quitaron a
propósito: con dos visores vivos había dos fuentes publicando /joint_states y
/cmd_flippers, y los comandos se pisaban entre sí (ver can_monitor.py). La única
interfaz es Foxglove, conectado al puente WebSocket del 8765.

Argumentos:
    frecuencia_hz    (100.0) frecuencia del bucle de control de flipper_node.
    modo             ('')    'gemelo' | 'can' | 'pi3hat'. Vacío = usar modo_simulacion.
    modo_simulacion  (true)  true = gemelo digital; false = pi3hat/CAN real.
    can_canal        (vcan0) interfaz SocketCAN para modo:=can.
    imu_fuente       (sintetica) 'sintetica' | 'pi3hat_real'. Independiente de
                             `modo`: permite motores emulados + IMU física real.
    imu_externa      (false) la IMU real la publica OTRA máquina (la Raspberry con
                             el pi3hat) en /imu/data_raw. Ver abajo.
    use_ekf          (false) si true, arranca robot_localization con config/ekf.yaml.
    use_politica     (false) arranca politica_flippers, que ejecuta el
                             modelo .onnx entrenado en el PC y comanda
                             flippers Y orugas. Sin `modelo_politica` solo
                             observa, no comanda.
    modelo_politica  ('')    ruta al .onnx en la Raspberry. Su .json de
                             contrato debe estar al lado, con el mismo nombre.
    politica_activa  (false) true = comandar desde el arranque. Dejarlo en
                             false y activar desde Foxglove con /politica/activa.
    use_rviz         (false) OBSOLETO. Se acepta para no romper llamadas viejas,
                             pero ya no hace nada: no se abre RViz nunca.
    use_foxglove     (true)  puente WebSocket en el 8765. Es la ÚNICA
                             visualización del proyecto; Foxglove se conecta a
                             ws://<ip-de-la-maquina>:8765.
    use_teleop       (true)  nodo teleop_flippers: mueve los flippers a mano desde
                             los paneles Teleop de Foxglove (ver su docstring).

IMU FÍSICA EN OTRA MÁQUINA (imu_externa:=true)
----------------------------------------------
La IMU del pi3hat se lee por SPI, así que solo puede abrirla un proceso que corra
EN la Raspberry. Para tener motores emulados en el PC + IMU física real, la Pi
publica /imu/data_raw y aquí hay que callar la IMU sintética: si las dos publican
en el mismo tópico, el EKF fusiona las dos y el modelo 3D se pelea consigo mismo.

`imu_externa:=true` no apaga la IMU sintética (flipper_node siempre publica una);
la desvía a /imu/sintetica, donde no molesta y además queda disponible para
compararla con la real. El EKF sigue leyendo /imu/data_raw, que ahora viene de la
Pi. En la Raspberry:

    ros2 run ugv_bridge pi3hat_imu --ros-args -r /imu/data:=/imu/data_raw
"""
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, LaunchConfiguration, NotSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    frecuencia_hz = LaunchConfiguration('frecuencia_hz')
    modo = LaunchConfiguration('modo')
    modo_simulacion = LaunchConfiguration('modo_simulacion')
    can_canal = LaunchConfiguration('can_canal')
    use_ekf = LaunchConfiguration('use_ekf')

    pkg_share = get_package_share_directory('ugv_bridge')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Geometría: única fuente de verdad, compartida por el URDF y los nodos.
    geometria_yaml = os.path.join(pkg_share, 'config', 'geometria_robot.yaml')
    xacro_path = os.path.join(pkg_share, 'urdf', 'ugv.urdf.xacro')
    # Se procesa el xacro en caliente: el modelo nunca queda desfasado del YAML.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' geometria:=', geometria_yaml]),
        value_type=str,
    )

    # Destino de la IMU que publica flipper_node. Con imu_externa:=true se aparta
    # a /imu/sintetica para dejarle /imu/data_raw a la IMU real de la Raspberry;
    # si no, el remapeo es tópico -> mismo tópico, o sea nada.
    destino_imu = PythonExpression([
        "'/imu/sintetica' if '", LaunchConfiguration('imu_externa'),
        "'.lower() in ('true', '1') else '/imu/data_raw'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('frecuencia_hz', default_value='100.0'),
        DeclareLaunchArgument('modo', default_value=''),
        DeclareLaunchArgument('modo_simulacion', default_value='true'),
        DeclareLaunchArgument('can_canal', default_value='vcan0'),
        DeclareLaunchArgument('imu_fuente', default_value='sintetica'),
        # Banco de pruebas: IDs de los motores realmente cableados, separados por
        # comas (ej. motores_presentes:=1). Vacío = los 8.
        DeclareLaunchArgument('motores_presentes', default_value=''),
        DeclareLaunchArgument('imu_externa', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='false'),
        # Obsoleto: se mantiene declarado para que `use_rviz:=false` de llamadas
        # antiguas no reviente, pero no enciende nada.
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        # Política de flippers (percepción táctil). Apagada por defecto: mueve
        # ocho motores de 48 V y no debe arrancar por accidente.
        DeclareLaunchArgument('use_politica', default_value='false'),
        DeclareLaunchArgument('modelo_politica', default_value=''),
        DeclareLaunchArgument('politica_activa', default_value='false'),

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
                'imu_fuente': LaunchConfiguration('imu_fuente'),
                # El argumento llega como texto ("1,5"); aquí se convierte en la
                # lista de enteros que espera el parámetro del nodo.
                'motores_presentes': ParameterValue(
                    PythonExpression([
                        '[int(x) for x in "',
                        LaunchConfiguration('motores_presentes'),
                        '".replace(",", " ").split()]',
                    ]),
                    value_type=List[int],
                ),
            }],
            remappings=[('/imu/data_raw', destino_imu)],
        ),

        # Odometría de orugas: /joint_states -> /odom (odom0 del EKF).
        #
        # publish_tf sigue a "not use_ekf", y esto NO es un detalle:
        #   - con EKF: el TF odom->base_link lo emite el EKF (con la inclinación
        #     que mide la IMU). Si este nodo también lo emitiera, base_link
        #     tendría dos padres y el árbol TF quedaría inválido.
        #   - sin EKF: lo emite este nodo, para que el robot no quede huérfano en
        #     el frame odom (en Foxglove: sin esto el modelo no aparece). Ojo, esa
        #     versión es sólo planar: mueve el robot pero no lo inclina.
        Node(
            package='ugv_bridge',
            executable='track_odometry_node',
            name='track_odometry_node',
            output='screen',
            parameters=[geometria_yaml, {
                'publish_tf': ParameterValue(NotSubstitution(use_ekf), value_type=bool),
            }],
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

        # RViz: ELIMINADO a propósito. Toda la visualización va por Foxglove.
        # Si alguien pasa use_rviz:=true (llamadas o scripts viejos), que al
        # menos se entere de por qué no se abre nada.
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            msg='[flipper.launch] use_rviz está OBSOLETO y se ignora: la única '
                'visualización del proyecto es Foxglove (ws://<host>:8765).'),

        # Mover los flippers a mano desde los paneles Teleop de Foxglove.
        # Solo publica cuando llega un comando, así que no estorba a nadie.
        Node(
            condition=IfCondition(LaunchConfiguration('use_teleop')),
            package='ugv_bridge',
            executable='teleop_flippers',
            name='teleop_flippers',
            output='screen',
        ),

        # Inferencia del modelo entrenado en el PC. Solo ejecuta
        # un .onnx con onnxruntime: en la Raspberry no hay torch ni SB3.
        #
        # Sin `modelo_politica` el nodo no comanda nada y se limita a publicar
        # /politica/observacion, la entrada de la red según el contrato.
        # Eso es lo útil AHORA, antes de entrenar: ver en Foxglove si el error
        # de seguimiento y el par de cada flipper reaccionan de verdad al pasar
        # por encima de un obstáculo.
        #
        # Y aunque haya modelo, arranca DESACTIVADA: hay que mandar `true` a
        # /politica/activa para que empiece a comandar. Al desactivarse o si
        # algo falla, manda orugas a 0 y deja los flippers donde están.
        Node(
            condition=IfCondition(LaunchConfiguration('use_politica')),
            package='ugv_bridge',
            executable='politica_flippers',
            name='politica_flippers',
            output='screen',
            parameters=[{
                'modelo': LaunchConfiguration('modelo_politica'),
                'activa_al_inicio': ParameterValue(
                    LaunchConfiguration('politica_activa'), value_type=bool),
            }],
        ),

        # Puente WebSocket para Foxglove Studio, que corre en otra máquina.
        # send_buffer_limit ampliado: las mallas del modelo son pesadas y con el
        # límite por defecto el puente corta la conexión al mandar el modelo.
        Node(
            condition=IfCondition(LaunchConfiguration('use_foxglove')),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765, 'send_buffer_limit': 100000000}],
        ),
    ])
