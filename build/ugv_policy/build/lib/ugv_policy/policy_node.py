#!/usr/bin/env python3
"""
policy_node.py — Nodo de política (el "cerebro") del UGV para ROS2.

Escucha /scan (5 distancias de los telémetros), decide una acción con el
cerebro activo (HeuristicBrain por defecto, RLBrain con un modelo entrenado)
y publica /cmd_vel. Es el equivalente ROS2 del brain del MapeoWorker.

La gracia sim-to-real: este nodo NO sabe si /scan viene de la simulación o del
robot real. Su entrada y salida son idénticas en ambos casos.

Para usar el modelo RL entrenado, lánzalo con el parámetro model_path:
    ros2 run ugv_policy policy_node --ros-args -p model_path:=/ruta/modelo_robot.zip
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from ugv_core.robot_env import HeuristicBrain, RLBrain

# Velocidades de salida (m/s y rad/s). El signo de angular.z es lo que
# ugv_sim interpreta como giro; estos valores también sirven al robot real.
LINEAR = 0.25
ANGULAR = 0.5


class PolicyNode(Node):
    def __init__(self):
        super().__init__('ugv_policy')

        # Parámetro: ruta al modelo RL. Vacío -> heurística (cambio de 1 línea).
        self.declare_parameter('model_path', '')
        ruta = self.get_parameter('model_path').get_parameter_value().string_value

        if ruta:
            self.brain = RLBrain(model_path=ruta)
            self.get_logger().info(f'Cerebro: RLBrain ({ruta})')
        else:
            self.brain = HeuristicBrain()
            self.get_logger().info('Cerebro: HeuristicBrain (sin modelo RL)')
        self.brain.reset()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Reacciona a cada /scan: la frecuencia la marca ugv_sim (20 Hz).
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10)
        self.get_logger().info('ugv_policy iniciado: /scan -> /cmd_vel')

    def on_scan(self, msg: LaserScan):
        # /scan (metros) -> obs normalizada [0,1] que espera el cerebro.
        rmax = msg.range_max if msg.range_max > 0 else 2.5
        obs = np.clip(np.array(msg.ranges, dtype=np.float32) / rmax, 0.0, 1.0)

        accion = int(self.brain.decidir(obs))   # 0 izq, 1 recto, 2 der
        self.publicar_cmd(accion)

    def publicar_cmd(self, accion):
        cmd = Twist()
        cmd.linear.x = LINEAR
        if accion == 0:
            cmd.angular.z = ANGULAR      # izquierda (REP-103: z>0)
        elif accion == 2:
            cmd.angular.z = -ANGULAR     # derecha
        else:
            cmd.angular.z = 0.0          # recto
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
