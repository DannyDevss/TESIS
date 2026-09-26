#!/usr/bin/env python3
"""pi3hat_imu_node.py — Nodo SUELTO para probar la IMU física del pi3hat.

Publica /imu/data y, opcionalmente, imprime roll/pitch/yaw en la terminal. Sirve
para validar la placa (permisos SPI, montaje, offsets) sin levantar todo el
sistema de motores.

CUÁNDO USAR ESTE NODO Y CUÁNDO NO
---------------------------------
  - Banco de pruebas / verificar la placa  -> este nodo (pi_view.launch.py).
  - Sistema en marcha                      -> NO: usar flipper_node con
    `imu_fuente:=pi3hat_real`, que publica /imu/data_raw (el tópico que consume
    el EKF) por el mismo camino que los motores y con las covarianzas puestas.

Correr los dos a la vez es redundante, pero no conflictivo: son tópicos distintos
(/imu/data aquí, /imu/data_raw en flipper_node). Eso sí, ambos abren la placa, y
el pi3hat admite un solo dueño del bus SPI por proceso: el segundo en arrancar
fallará. Uno u otro.

La lectura real vive en pi3hat_backend.LectorImuPi3Hat, compartida con el driver:
un solo lugar donde arreglar la IMU si cambia la librería o el montaje.

Parámetros:
    imu_montaje_roll|pitch|yaw (double, 0.0) rotación de la placa respecto a
        base_link (rad). Fuente: config/geometria_robot.yaml.
    frecuencia_hz (double, 50.0) tasa de publicación.
    imprimir      (bool, true)   eco de roll/pitch/yaw en la terminal.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from ugv_bridge.pi3hat_backend import LectorImuPi3Hat

# Covarianzas de la IMU del pi3hat. NO se dejan en cero (cero significa
# "medición perfecta" y degenera cualquier filtro que la consuma) ni en -1
# (que significa "este dato no existe" y haría que robot_localization descarte
# la orientación, justo lo que necesita el EKF para inclinar el modelo 3D).
# TODO(caracterizar): medir el ruido real de la placa en reposo y ajustar.
VAR_ORIENTACION = 0.01     # rad^2  (~5.7 grados de sigma)
VAR_VEL_ANGULAR = 0.01     # (rad/s)^2
VAR_ACELERACION = 0.05     # (m/s^2)^2


class Pi3HatImuNode(Node):
    def __init__(self):
        super().__init__('pi3hat_imu_node')

        self.declare_parameter('imu_montaje_roll', 0.0)
        self.declare_parameter('imu_montaje_pitch', 0.0)
        self.declare_parameter('imu_montaje_yaw', 0.0)
        self.declare_parameter('frecuencia_hz', 50.0)
        self.declare_parameter('imprimir', True)

        montaje = (
            self.get_parameter('imu_montaje_roll').value,
            self.get_parameter('imu_montaje_pitch').value,
            self.get_parameter('imu_montaje_yaw').value,
        )
        self.imprimir = self.get_parameter('imprimir').value
        frecuencia = self.get_parameter('frecuencia_hz').value

        # Foxglove reconoce el tipo Imu en este tópico.
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        self.lector = LectorImuPi3Hat(montaje_rpy=montaje)
        self.get_logger().info(
            f'IMU del pi3hat lista a {frecuencia:.0f} Hz -> /imu/data '
            f'(montaje rpy={tuple(round(math.degrees(a), 1) for a in montaje)} grados)')

        self.create_timer(1.0 / frecuencia, self.publicar)

    def publicar(self):
        try:
            d = self.lector.leer()
        except Exception as e:
            self.get_logger().error(f'Error leyendo la IMU: {e}', throttle_duration_sec=2.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'  # crucial para el 3D en Foxglove

        x, y, z, w = d['quat']
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)
        msg.orientation.w = float(w)
        msg.linear_acceleration.x = float(d['accel_x'])
        msg.linear_acceleration.y = float(d['accel_y'])
        msg.linear_acceleration.z = float(d['accel_z'])

        for i in (0, 4, 8):
            msg.orientation_covariance[i] = VAR_ORIENTACION
            msg.angular_velocity_covariance[i] = VAR_VEL_ANGULAR
            msg.linear_acceleration_covariance[i] = VAR_ACELERACION

        self.publisher_.publish(msg)

        if self.imprimir:
            print(f"\r[IMU] Roll:{math.degrees(d['roll']): 6.1f} | "
                  f"Pitch:{math.degrees(d['pitch']): 6.1f} | "
                  f"Yaw:{math.degrees(d['yaw']): 6.1f}   ", end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Pi3HatImuNode()
    except Exception as e:
        # Sin placa (p.ej. dentro de Distrobox en el PC) esto es lo esperado.
        print(f'[pi3hat_imu_node] No se pudo abrir la IMU del pi3hat: {e}')
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nTest finalizado.')
    finally:
        node.lector.cerrar()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
