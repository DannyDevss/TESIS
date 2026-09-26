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
  /diagnostics  (diagnostic_msgs/DiagnosticArray, 1 Hz)  una fila por motor:
                cableado o no, si responde y si está en lazo cerrado. Se ve en
                el panel Diagnostics de Foxglove (y en rqt_robot_monitor).

Parámetros
----------
  frecuencia_hz  (double, 100.0)  frecuencia del bucle de control.
  modo           (string, '')      'gemelo' | 'can' | 'pi3hat'. Vacío = decidir
                                   por modo_simulacion (compatibilidad).
  modo_simulacion (bool, True)     True = gemelo digital; False = pi3hat real.
  can_canal      (string, 'vcan0') interfaz SocketCAN para modo 'can'.
  imu_fuente     (string, 'sintetica') 'sintetica' | 'pi3hat_real'. INDEPENDIENTE
                                   del modo de motor.
  mapa_buses     (int[], [1,2,3,4,1,2,3,4]) bus del pi3hat de cada motor 1..8.
                 Dos motores por bus, emparejados por esquina del robot.
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
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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
        self.declare_parameter('mapa_buses', [1, 2, 3, 4, 1, 2, 3, 4])
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
            motores_presentes=self._motores_presentes(),
        )
        self._fallo_avisado = False
        # Ciclos seguidos con algún eje fuera de lazo cerrado. Al arrancar
        # siempre hay unos pocos (hasta el primer heartbeat, que llega cada
        # 100 ms), así que sólo se avisa si la cosa PERSISTE.
        self._ciclos_desarmado = 0
        self._desarme_avisado = False
        self._ciclos_para_avisar_desarme = max(1, int(frecuencia))
        # Último estado leído del bucle de control; lo usa el diagnóstico.
        self.ultimo_estado = {}

        # Últimos comandos recibidos (por ID de motor). Arranque seguro: quieto.
        #
        # Las orugas arrancan en 0 rad/s, que es "quieto" de verdad.
        #
        # Los flippers arrancan VACÍOS, y eso NO es lo mismo que 0.0. El comando
        # de un flipper es una POSICIÓN: mandar 0.0 no es "quieto", es "vete al
        # cero del encoder", y el motor sale disparado hasta allí en cuanto se
        # arma en lazo cerrado. Con la reducción de 8:1 eso es un golpe seco, y
        # ocurría ANTES de que teleop_flippers llegara a publicar nada (ese nodo
        # sí se siembra con la pose real de /joint_states, pero para entonces el
        # cero ya iba camino del bus).
        #
        # Dejando el dict vacío, los dos backends caen en su valor por defecto,
        # que es "quédate donde estás":
        #   driver_movimiento._estado_via_bus -> _ultimo_estado[mid]['posicion_rad']
        #   driver_movimiento._simular_estado -> self._pos[mid]
        # A partir del primer /cmd_flippers manda la consigna, como siempre.
        self.cmd_orugas = {mid: 0.0 for mid in IDS_ORUGAS}
        self.cmd_flippers = {}

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.create_subscription(Float64MultiArray, '/cmd_tracks', self.on_cmd_tracks, 10)
        self.create_subscription(Float64MultiArray, '/cmd_flippers', self.on_cmd_flippers, 10)

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.timer = self.create_timer(1.0 / frecuencia, self.loop_control)
        # El diagnóstico va aparte del bucle de control y MUCHO más lento: es
        # para que lo lea una persona, no el lazo.
        self.timer_diag = self.create_timer(1.0, self.publicar_diagnostico)
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
    def _motores_presentes(self):
        """IDs cableados de verdad, o None si están los 8.

        `motores_presentes` se declara por TIPO y sin valor por defecto (de una
        lista vacía rclpy deduciría BYTE_ARRAY y rechazaría los enteros que manda
        el launch). El precio es que, si nadie lo pasa —por ejemplo con
        `ros2 run ugv_bridge flipper_node` a pelo—, `get_parameter` NO devuelve
        vacío: lanza ParameterUninitializedException.
        """
        try:
            valor = self.get_parameter('motores_presentes').value
        except ParameterUninitializedException:
            return None
        return list(valor) or None

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
        # Lo guarda para el diagnóstico, que corre en su propio timer a 1 Hz.
        self.ultimo_estado = estado
        imu = self.robot.leer_imu()
        stamp = self.get_clock().now().to_msg()
        self.publicar_joint_state(estado, stamp)
        self.publicar_imu(imu, stamp)
        self.revisar_comunicacion()
        self.revisar_armado()

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

    def publicar_diagnostico(self):
        """Una fila por motor en /diagnostics: quién está y quién no.

        Es la "ventanita" de estado del banco de pruebas. Distingue los tres
        casos que desde fuera se parecen demasiado:

          - AUSENTE  no está en `motores_presentes`: no se le habla ni se le
                     exige nada. Sale como STALE, no como error, porque es una
                     decisión deliberada y no una avería.
          - MUDO     está declarado y no emite: cable, alimentación o node_id.
          - EN IDLE  emite pero está fuera de lazo cerrado: acepta las órdenes y
                     no se mueve. El fallo silencioso de ODrive.

        Se publica a 1 Hz porque lo lee una persona en Foxglove, no el lazo de
        control.
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        resumen = self.robot.resumen_motores()
        for mid, info in sorted(resumen.items()):
            st = DiagnosticStatus()
            st.hardware_id = f'motor {mid}'
            st.name = f'motores/{info["nombre"]}'
            if not info['presente']:
                st.level = DiagnosticStatus.STALE
                st.message = 'AUSENTE: no declarado en motores_presentes'
            elif not info['responde']:
                st.level = DiagnosticStatus.ERROR
                st.message = 'MUDO: no emite en el bus (¿cable, alimentación, node_id?)'
            elif not info['armado']:
                st.level = DiagnosticStatus.WARN
                st.message = 'FUERA DE LAZO CERRADO: acepta las órdenes y no se mueve'
            elif info['error_eje']:
                st.level = DiagnosticStatus.WARN
                st.message = f'en lazo cerrado, con error 0x{info["error_eje"]:X}'
            else:
                st.level = DiagnosticStatus.OK
                st.message = 'en lazo cerrado'

            estado = self.ultimo_estado.get(mid, {})
            st.values = [
                KeyValue(key='id', value=str(mid)),
                KeyValue(key='tipo', value=info['tipo']),
                KeyValue(key='bus', value=str(info['bus'])),
                KeyValue(key='cableado', value='sí' if info['presente'] else 'no'),
                KeyValue(key='estado_eje', value=str(info['estado_eje'])),
                KeyValue(key='error_eje', value=f'0x{info["error_eje"]:X}'),
                KeyValue(key='ciclos_sin_emitir', value=str(info['ciclos_mudo'])),
                KeyValue(key='posicion_rad',
                         value=f'{estado.get("posicion_rad", 0.0):+.4f}'),
                KeyValue(key='velocidad_rad_s',
                         value=f'{estado.get("velocidad_rad_s", 0.0):+.4f}'),
            ]
            msg.status.append(st)

        # Fila de resumen, para no tener que contar filas a ojo.
        cableados = [i['nombre'] for i in resumen.values() if i['presente']]
        problemas = [i['nombre'] for i in resumen.values()
                     if i['presente'] and not (i['responde'] and i['armado'])]
        total = DiagnosticStatus()
        total.hardware_id = 'ugv'
        total.name = 'motores/RESUMEN'
        if self.robot.fallo_comunicacion:
            total.level = DiagnosticStatus.ERROR
            total.message = 'PARO DE EMERGENCIA: fallo de comunicación'
        elif problemas:
            total.level = DiagnosticStatus.WARN
            total.message = f'{len(problemas)} de {len(cableados)} con problemas: ' \
                            f'{", ".join(problemas)}'
        else:
            total.level = DiagnosticStatus.OK
            total.message = f'{len(cableados)} de 8 motores cableados y moviéndose'
        total.values = [
            KeyValue(key='cableados', value=', '.join(cableados) or '(ninguno)'),
            KeyValue(key='ausentes',
                     value=', '.join(i['nombre'] for i in resumen.values()
                                     if not i['presente']) or '(ninguno)'),
        ]
        msg.status.append(total)

        self.diag_pub.publish(msg)

    def revisar_armado(self):
        """Avisa si algún eje se quedó fuera de lazo cerrado.

        Es el fallo silencioso de ODrive: el eje acepta la consigna, no se mueve
        y no devuelve error, así que sin este aviso el síntoma es "mando
        /cmd_flippers y no pasa nada". El driver ya reintenta armarlo solo; esto
        es para que se VEA (panel RosOut de Foxglove, o rqt).
        """
        desarmados = getattr(self.robot, 'motores_desarmados', [])
        if desarmados:
            self._ciclos_desarmado += 1
        else:
            self._ciclos_desarmado = 0

        if (self._ciclos_desarmado >= self._ciclos_para_avisar_desarme
                and not self._desarme_avisado):
            self._desarme_avisado = True
            nombres = ', '.join(ID_A_NOMBRE.get(m, str(m)) for m in desarmados)
            self.get_logger().warn(
                f'Motores FUERA DE LAZO CERRADO: {nombres}. Aceptan las órdenes '
                f'y no se mueven (fallo silencioso de ODrive). Reintentando '
                f'armado; si no se arregla: ¿energizados? ¿enable_can_a y '
                f'baud_rate bien puestos? ¿motor_emulator corriendo?')
        elif not desarmados and self._desarme_avisado:
            self._desarme_avisado = False
            self.get_logger().info('Todos los ejes en lazo cerrado.')

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
