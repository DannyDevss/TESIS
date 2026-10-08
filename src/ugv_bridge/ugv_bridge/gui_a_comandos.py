#!/usr/bin/env python3
"""gui_a_comandos.py — LEGADO. Puente sliders (joint_state_publisher_gui) -> /cmd_flippers.

NINGÚN LAUNCH ARRANCA YA ESTE NODO.
-----------------------------------
Toda la interacción del proyecto pasa por Foxglove: paneles `Publish` para poses
fijas de /cmd_flippers y /cmd_tracks, y paneles `Teleop` sobre /teleop/flipper_*
(ver teleop_flippers.py). La ventana Qt de sliders se retiró porque era un
SEGUNDO mando: publicaba en /cmd_flippers a la vez que Foxglove y, como
flipper_node retiene el último comando que recibe, el flipper se iba al valor de
quien hablara último. Encima la ventana sobrevivía al Ctrl+C del launch y seguía
publicando en segundo plano.

El archivo se conserva por si alguna vez hace falta volver a los sliders en un
PC con escritorio. Para usarlo hay que levantar a mano las dos piezas:

    ros2 run joint_state_publisher_gui joint_state_publisher_gui \
        --ros-args -r joint_states:=gui/joint_states
    ros2 run ugv_bridge gui_a_comandos

Y entonces NO usar los paneles de mando de Foxglove al mismo tiempo.

En view.launch.py la GUI de sliders publica /joint_states directamente: "dibuja" el
robot sin pasar por el driver. Este nodo invierte el flujo para can_view.launch.py:
la GUI se remapea a /gui/joint_states y aquí sus posiciones se convierten en
COMANDOS de flipper, de modo que el movimiento visible en RViz recorre el camino
completo: slider -> /cmd_flippers -> flipper_node -> tramas CAN (vcan0) ->
motor_emulator -> respuesta CAN -> /joint_states -> robot_state_publisher -> RViz.

Tópicos
-------
Suscribe:
  /gui/joint_states (sensor_msgs/JointState)  posiciones de los sliders
Publica:
  /cmd_flippers (std_msgs/Float64MultiArray, 4 posiciones rad, orden fl/fr/rl/rr)

Los sliders de las orugas (track_*) se ignoran: son juntas de velocidad y un slider
de posición no tiene sentido para comandarlas (usar /cmd_tracks como siempre).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ugv_bridge.driver_movimiento import NOMBRES_FLIPPERS


class GuiAComandos(Node):
    def __init__(self):
        super().__init__('gui_a_comandos')
        self.ultimo_enviado = None
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/cmd_flippers', 10)
        self.create_subscription(JointState, '/gui/joint_states', self.on_gui, 10)
        self.get_logger().info(
            'gui_a_comandos iniciado: /gui/joint_states -> /cmd_flippers '
            f'({", ".join(NOMBRES_FLIPPERS)})')

    def on_gui(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        try:
            cmd = [float(pos[nombre]) for nombre in NOMBRES_FLIPPERS]
        except KeyError:
            return  # mensaje sin los 4 flippers (p. ej. GUI aún cargando el URDF)
        # La GUI publica a ~10 Hz aunque nada cambie; solo se reenvía lo nuevo
        # (flipper_node retiene el último comando, no necesita refresco).
        if cmd != self.ultimo_enviado:
            self.cmd_pub.publish(Float64MultiArray(data=cmd))
            self.ultimo_enviado = cmd


def main(args=None):
    rclpy.init(args=args)
    node = GuiAComandos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
