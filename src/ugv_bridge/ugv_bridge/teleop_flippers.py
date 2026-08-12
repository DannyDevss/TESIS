#!/usr/bin/env python3
"""teleop_flippers.py — Mover los flippers "a mano" desde Foxglove, sin sliders.

POR QUÉ ESTE NODO EXISTE
------------------------
Foxglove tiene un panel `Variable Slider`, pero NO sirve para esto: escribe una
variable global del layout, y esas variables solo se interpolan en expresiones
del panel Plot. El panel `Publish` manda un JSON FIJO (sus claves son topicName,
datatype, buttonText, advancedView, value): no sustituye variables. O sea que en
Foxglove no hay ningún slider capaz de publicar en un tópico.

El único panel que publica de forma CONTINUA es `Teleop`: manda un
geometry_msgs/Twist a `publishRate` Hz mientras mantienes pulsado un botón, y un
Twist en cero al soltar (autoSendStopOnRelease).

Este nodo aprovecha eso: interpreta cada Twist como "mueve este flipper a tal
VELOCIDAD" e integra esa velocidad en un ángulo, que publica en /cmd_flippers.
Mantener pulsado mueve el flipper; soltar lo deja quieto donde está. En la
práctica se maneja como un joystick y llega a cualquier ángulo, que es lo que un
slider daría.

    Foxglove [Teleop]  --Twist-->  este nodo  --/cmd_flippers-->  flipper_node

TÓPICOS DE ENTRADA (uno por flipper, más uno para los cuatro a la vez)
    /teleop/flipper_fl   /teleop/flipper_fr
    /teleop/flipper_rl   /teleop/flipper_rr
    /teleop/flippers     (mueve los cuatro en bloque)

De cada Twist solo se usa `linear.x`: su SIGNO decide el sentido y su magnitud
escala la velocidad. Con la configuración por defecto del panel (valor 1.0), el
flipper gira a `velocidad_rad_s`.

PARÁMETROS
    velocidad_rad_s (0.6)  velocidad de giro con un Twist de magnitud 1.0.
    frecuencia_hz   (30.0) ritmo de integración y de publicación.
    timeout_s       (0.35) sin Twist nuevo en este tiempo, el flipper se detiene.
                           Es la red de seguridad por si se pierde el mensaje de
                           paro que el panel manda al soltar: sin esto, un botón
                           soltado en el momento justo dejaría el flipper girando
                           sin parar.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Orden de /cmd_flippers, el mismo de flipper_node: fl, fr, rl, rr.
FLIPPERS = ('fl', 'fr', 'rl', 'rr')
NOMBRE_JUNTA = {n: f'flipper_{n}' for n in FLIPPERS}


class TeleopFlippers(Node):
    def __init__(self):
        super().__init__('teleop_flippers')

        self.declare_parameter('velocidad_rad_s', 0.6)
        self.declare_parameter('frecuencia_hz', 30.0)
        self.declare_parameter('timeout_s', 0.35)

        self.velocidad = float(self.get_parameter('velocidad_rad_s').value)
        self.timeout = float(self.get_parameter('timeout_s').value)
        frecuencia = float(self.get_parameter('frecuencia_hz').value)

        # Objetivo por flipper. Arranca en None a propósito: hasta que no se sabe
        # dónde ESTÁ el robot no se puede comandar, o el primer toque lo haría
        # saltar desde 0 rad. Se rellena con la primera lectura de /joint_states.
        self.objetivo = {n: None for n in FLIPPERS}
        self.velocidad_pedida = {n: 0.0 for n in FLIPPERS}
        self.t_ultimo_cmd = {n: 0.0 for n in FLIPPERS}
        self.hay_pose = False

        self.pub = self.create_publisher(Float64MultiArray, '/cmd_flippers', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        for n in FLIPPERS:
            self.create_subscription(
                Twist, f'/teleop/flipper_{n}',
                lambda msg, nombre=n: self.on_teleop(msg, [nombre]), 10)
        self.create_subscription(
            Twist, '/teleop/flippers',
            lambda msg: self.on_teleop(msg, list(FLIPPERS)), 10)

        self.dt = 1.0 / frecuencia
        self.create_timer(self.dt, self.integrar)

        self.get_logger().info(
            f'teleop_flippers listo: /teleop/flipper_* -> /cmd_flippers '
            f'({self.velocidad:.2f} rad/s, {frecuencia:.0f} Hz). '
            f'Esperando /joint_states para conocer la pose inicial...')

    # ------------------------------------------------------------------ #
    def on_joint_states(self, msg: JointState):
        """Semilla de la pose: solo la PRIMERA vez.

        Después no se vuelve a leer: el objetivo lo manda este nodo y el flipper
        lo persigue. Si se resincronizara en cada mensaje, el error de
        seguimiento del motor se realimentaría y el flipper se arrastraría solo.
        """
        if self.hay_pose:
            return
        posiciones = dict(zip(msg.name, msg.position))
        leidos = 0
        for n in FLIPPERS:
            valor = posiciones.get(NOMBRE_JUNTA[n])
            if valor is not None:
                self.objetivo[n] = float(valor)
                leidos += 1
        if leidos == len(FLIPPERS):
            self.hay_pose = True
            grados = {n: round(math.degrees(self.objetivo[n]), 1) for n in FLIPPERS}
            self.get_logger().info(f'Pose inicial tomada de /joint_states: {grados} grados')

    def on_teleop(self, msg: Twist, nombres):
        ahora = time.monotonic()
        for n in nombres:
            self.velocidad_pedida[n] = float(msg.linear.x)
            self.t_ultimo_cmd[n] = ahora

    # ------------------------------------------------------------------ #
    def integrar(self):
        if not self.hay_pose:
            return

        ahora = time.monotonic()
        movio = False
        for n in FLIPPERS:
            v = self.velocidad_pedida[n]
            # Sin mensajes recientes, se considera botón soltado.
            if v != 0.0 and (ahora - self.t_ultimo_cmd[n]) > self.timeout:
                self.velocidad_pedida[n] = 0.0
                v = 0.0
            if v == 0.0:
                continue
            self.objetivo[n] += v * self.velocidad * self.dt
            movio = True

        # Se publica SOLO cuando algo cambió: flipper_node retiene el último
        # comando, así que repetirlo a 30 Hz sería ruido en el bus para nada.
        if movio:
            msg = Float64MultiArray()
            msg.data = [self.objetivo[n] for n in FLIPPERS]
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = TeleopFlippers()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
