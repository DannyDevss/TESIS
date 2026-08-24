#!/usr/bin/env python3
"""flipper_node.py — Puente ROS 2 <-> hardware de motores (orugas + flippers) + IMU.

Bucle de control de alta frecuencia (por defecto 100 Hz) que:
  1. lee los últimos comandos de la política (RL o teleop),
  2. los envía a los 8 motores vía `RMD_Hardware` (real o gemelo digital),
  3. publica el estado como `sensor_msgs/JointState` y la IMU como `sensor_msgs/Imu`.

Tópicos
-------
Suscribe:
  /cmd_tracks   (std_msgs/Float64MultiArray, 4 velocidades rad/s)  -> orugas  IDs 1-4
  /cmd_flippers (std_msgs/Float64MultiArray, 4 posiciones rad)     -> flippers IDs 5-8
Publica:
  /joint_states (sensor_msgs/JointState)  estado de los 8 motores
  /imu/data_raw (sensor_msgs/Imu)         actitud + aceleración (cuaternión)

Parámetros
----------
  frecuencia_hz  (double, 100.0)  frecuencia del bucle de control.
  modo           (string, '')      'gemelo' | 'can' | 'pi3hat'. Vacío = decidir
                                   por modo_simulacion (compatibilidad).
  modo_simulacion (bool, True)     True = gemelo digital; False = pi3hat real.
  can_canal      (string, 'vcan0') interfaz SocketCAN para modo 'can'.
  imu_fuente     (string, 'sintetica') 'sintetica' | 'pi3hat_real'. INDEPENDIENTE
                                   del modo de motor.
  mapa_buses     (int[], [1,1,1,1,2,2,2,2]) bus del pi3hat de cada motor 1..8.
  motores_presentes (int[], [])    IDs cableados de verdad; vacío = los 8. Para
                                   probar en el banco con un motor suelto.
  radio_oruga / ancho_orugas / imu_montaje_roll|pitch|yaw
                                   -> config/geometria_robot.yaml.

Notas de sim-to-real: el mismo nodo sirve para simulación y hardware; solo cambia
`modo` ('gemelo' -> 'can' con motor_emulator -> 'pi3hat'). Como `imu_fuente` es un
parámetro aparte, hoy se puede correr con motores emulados e IMU física real:

    ros2 launch ugv_bridge can_sim.launch.py imu_fuente:=pi3hat_real

Para RL determinista, subir a 200 Hz y afinar QoS/prioridad de CPU.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray

from ugv_bridge.driver_movimiento import (
    RMD_Hardware,
    IDS_ORUGAS,
    IDS_FLIPPERS,
    ID_A_NOMBRE,
)


# Covarianzas de la IMU publicada en /imu/data_raw.
#
# Cero significa "medición perfecta" para robot_localization (satura a 1e-9) y
# -1 significa "este dato no existe", con lo que el EKF DESCARTA la orientación
# y nunca emitiría la inclinación del chasis. Ninguno de los dos sirve: hay que
# poner varianzas reales para que el filtro pueda fusionar IMU y odometría.
# TODO(caracterizar): medir el ruido de la placa en reposo y ajustar.
VAR_ORIENTACION_RP = 0.01   # rad^2, roll/pitch: los estabiliza la gravedad
VAR_ORIENTACION_YAW = 0.05  # rad^2, yaw: deriva (no hay magnetómetro fiable)
VAR_ACELERACION = 0.05      # (m/s^2)^2
VAR_VEL_ANGULAR = 0.01      # (rad/s)^2


def euler_a_quaternion(roll, pitch, yaw):
    """Euler (rad, XYZ) -> cuaternión (x, y, z, w). ROS no usa ángulos de Euler."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class FlipperNode(Node):
    def __init__(self):
        super().__init__('flipper_node')

        self.declare_parameter('frecuencia_hz', 100.0)
        self.declare_parameter('modo', '')
        self.declare_parameter('modo_simulacion', True)
        self.declare_parameter('can_canal', 'vcan0')
        # Geometría: config/geometria_robot.yaml (misma fuente que URDF y guardián).
        self.declare_parameter('radio_oruga', 0.05)
        self.declare_parameter('ancho_orugas', 0.30)
        self.declare_parameter('imu_montaje_roll', 0.0)
        self.declare_parameter('imu_montaje_pitch', 0.0)
        self.declare_parameter('imu_montaje_yaw', 0.0)
        # Fuente de la IMU, independiente del modo de motor.
        self.declare_parameter('imu_fuente', 'sintetica')
        # Bus del pi3hat de cada motor, en orden de ID 1..8.
        self.declare_parameter('mapa_buses', [1, 1, 1, 1, 2, 2, 2, 2])
        # Banco de pruebas: IDs realmente cableados. Vacío = los 8.
        # Se declara por TIPO y sin valor por defecto: de una lista vacía rclpy
        # deduciría BYTE_ARRAY y rechazaría los enteros que manda el launch.
        self.declare_parameter('motores_presentes',
                               rclpy.Parameter.Type.INTEGER_ARRAY)
        frecuencia = self.get_parameter('frecuencia_hz').value
        modo = self.get_parameter('modo').value
        modo_sim = self.get_parameter('modo_simulacion').value
        can_canal = self.get_parameter('can_canal').value

        # 'modo' explícito manda; si viene vacío, se respeta el flag antiguo.
        if not modo:
            modo = 'gemelo' if modo_sim else 'pi3hat'

        imu_fuente = self.get_parameter('imu_fuente').value
        buses = list(self.get_parameter('mapa_buses').value)
        mapa_buses = {mid: buses[i] for i, mid in enumerate(IDS_ORUGAS + IDS_FLIPPERS)
                      if i < len(buses)}

        self.robot = RMD_Hardware(
            modo=modo,
            canal_can=can_canal,
            radio_oruga=self.get_parameter('radio_oruga').value,
            ancho_orugas=self.get_parameter('ancho_orugas').value,
            imu_fuente=imu_fuente,
            imu_montaje_rpy=(
                self.get_parameter('imu_montaje_roll').value,
                self.get_parameter('imu_montaje_pitch').value,
                self.get_parameter('imu_montaje_yaw').value,
            ),
            mapa_buses=mapa_buses,
            # Sin el parámetro (o vacío) -> None -> los 8 motores.
            motores_presentes=list(
                self.get_parameter('motores_presentes').value or []) or None,
        )
        self._fallo_avisado = False

        # Últimos comandos recibidos (por ID de motor). Arranque seguro: quieto.
        self.cmd_orugas = {mid: 0.0 for mid in IDS_ORUGAS}
        self.cmd_flippers = {mid: 0.0 for mid in IDS_FLIPPERS}

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.create_subscription(Float64MultiArray, '/cmd_tracks', self.on_cmd_tracks, 10)
        self.create_subscription(Float64MultiArray, '/cmd_flippers', self.on_cmd_flippers, 10)

        self.timer = self.create_timer(1.0 / frecuencia, self.loop_control)
        etiqueta = {
            'gemelo': 'GEMELO DIGITAL',
            'can': f'BUS CAN "{can_canal}"',
            'pi3hat': 'HARDWARE REAL (pi3hat)',
        }[modo]
        etiqueta_imu = {
            'sintetica': 'IMU sintética',
            'pi3hat_real': 'IMU REAL del pi3hat',
        }[imu_fuente]
        self.get_logger().info(
            f'flipper_node iniciado a {frecuencia:.0f} Hz '
            f'(motores: {etiqueta} | {etiqueta_imu})')

    # ------------------------------------------------------------------ #
    # Entrada de comandos
    # ------------------------------------------------------------------ #
    def on_cmd_tracks(self, msg: Float64MultiArray):
        for mid, val in zip(IDS_ORUGAS, msg.data):
            self.cmd_orugas[mid] = float(val)

    def on_cmd_flippers(self, msg: Float64MultiArray):
        for mid, val in zip(IDS_FLIPPERS, msg.data):
            self.cmd_flippers[mid] = float(val)

    # ------------------------------------------------------------------ #
    # Bucle de control de alta frecuencia
    # ------------------------------------------------------------------ #
    def loop_control(self):
        estado = self.robot.enviar_y_leer_estado(self.cmd_orugas, self.cmd_flippers)
        imu = self.robot.leer_imu()
        stamp = self.get_clock().now().to_msg()
        self.publicar_joint_state(estado, stamp)
        self.publicar_imu(imu, stamp)
        self.revisar_comunicacion()

    def revisar_comunicacion(self):
        """Eleva a log de ROS el watchdog del driver (visible en Foxglove/rqt)."""
        if self.robot.fallo_comunicacion and not self._fallo_avisado:
            self._fallo_avisado = True
            self.get_logger().error(
                'FALLO DE COMUNICACIÓN con los motores: paro de emergencia activo. '
                '¿Está corriendo motor_emulator / está energizado el bus?')
        elif not self.robot.fallo_comunicacion and self._fallo_avisado:
            self._fallo_avisado = False
            self.get_logger().info('Comunicación con los motores restablecida.')

    def publicar_joint_state(self, estado, stamp):
        js = JointState()
        js.header.stamp = stamp
        for mid in IDS_ORUGAS + IDS_FLIPPERS:
            m = estado[mid]
            js.name.append(ID_A_NOMBRE[mid])
            js.position.append(m['posicion_rad'])
            js.velocity.append(m['velocidad_rad_s'])
            js.effort.append(m['torque_nm'])
        self.js_pub.publish(js)

    def publicar_imu(self, imu, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        # La IMU real entrega el cuaternión directo: se publica tal cual, sin
        # pasar por Euler (que degenera cerca de pitch = ±90°, o sea con el robot
        # apoyado de canto — un caso perfectamente posible en rescate).
        if 'quat' in imu:
            x, y, z, w = imu['quat']
        else:
            x, y, z, w = euler_a_quaternion(imu['roll'], imu['pitch'], imu['yaw'])
        msg.orientation.x = x
        msg.orientation.y = y
        msg.orientation.z = z
        msg.orientation.w = w
        msg.linear_acceleration.x = imu['accel_x']
        msg.linear_acceleration.y = imu['accel_y']
        msg.linear_acceleration.z = imu['accel_z']

        # Diagonales de las 3x3 (fila-mayor): índices 0, 4, 8.
        msg.orientation_covariance[0] = VAR_ORIENTACION_RP    # roll
        msg.orientation_covariance[4] = VAR_ORIENTACION_RP    # pitch
        msg.orientation_covariance[8] = VAR_ORIENTACION_YAW   # yaw
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = VAR_VEL_ANGULAR
            msg.linear_acceleration_covariance[i] = VAR_ACELERACION

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlipperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.cerrar()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
