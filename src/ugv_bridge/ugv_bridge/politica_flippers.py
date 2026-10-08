#!/usr/bin/env python3
"""politica_flippers.py — Ejecuta en la Raspberry el modelo entrenado en el PC.

SOLO INFERENCIA. Aquí no hay gymnasium, ni stable-baselines3, ni torch, ni
entorno de entrenamiento, ni recompensa. El entrenamiento vive en el PC; a la Pi
llega un único archivo `.onnx` más su `.json` de contrato, y este nodo lo
ejecuta. Esa separación es lo que permite que la Pi no necesite más que
`onnxruntime` (unos pocos MB) en vez de la pila entera de PyTorch.

    PC (alta gama)                      Raspberry Pi
    ---------------                     -------------
    simulador + RL                      este nodo
    entrena PPO/SAC          .onnx      lee /joint_states, /imu/data_raw
    exportar_onnx.py  ───────────────▶  construye la observación
                             .json      ejecuta la red
                                        publica /cmd_flippers

OBJETIVO: PERCEPCIÓN TÁCTIL
---------------------------
El robot tiene que notar las imperfecciones del suelo y acomodar los flippers
solo. Los sensores son los propios motores: el error de seguimiento y el par de
cada flipper dicen si está tocando algo y con cuánta fuerza. La explicación
completa y el layout exacto del vector están en `contrato_politica.py`, que es
la única fuente de verdad y la comparten el entrenamiento y este nodo.

SIRVE AUNQUE TODAVÍA NO HAYA MODELO
-----------------------------------
Sin el parámetro `modelo`, el nodo arranca igual y NO publica ningún comando,
pero sí publica `/politica/observacion`: las 40 señales normalizadas, en vivo.
Eso es justo lo que hace falta AHORA, antes de entrenar nada: conducir el robot
por encima de un obstáculo mirando en Foxglove cómo responden
`flipper_fl/error` y `flipper_fl/esfuerzo`, para saber si la señal táctil
existe de verdad y con qué amplitud, y de ahí sacar las escalas y la
recompensa. Entrenar antes de haber visto esas curvas es entrenar a ciegas.

SEGURIDAD
---------
Una red neuronal mandando ángulos a ocho motores de 48 V merece frenos. Hay
cuatro, todos activos a la vez:

  1. ARRANCA DESACTIVADA. Hasta que no llega `true` por `/politica/activa`
     (std_msgs/Bool), no sale un solo comando. En Foxglove es un panel Publish.
  2. SALIDA EN VELOCIDAD, NO EN POSICIÓN. La red pide velocidades acotadas que
     se integran sobre la posición MEDIDA. El peor comando posible mueve el
     flipper `VEL_ACCION_MAX / frecuencia_hz` radianes. Ver integrar_accion().
  3. WATCHDOG DE ESTADO. Si /joint_states se calla más de `timeout_estado_s`,
     deja de publicar. Sin esto la política seguiría comandando a ciegas sobre
     una foto vieja del robot, que es como se rompen los flippers.
  4. CONTRATO VERIFICADO AL CARGAR. Si la versión del contrato del modelo no es
     la de este código, o la entrada de la red no mide OBS_DIM, el nodo se
     niega a arrancar en vez de alimentar la red con el vector equivocado. Ese
     fallo, si se dejara pasar, no da ningún error: solo ángulos plausibles y
     erróneos.

TÓPICOS
-------
Suscribe:
    /joint_states      (sensor_msgs/JointState)   posición, velocidad, esfuerzo
    /imu/data_raw      (sensor_msgs/Imu)          actitud, giro, aceleración
    /cmd_tracks        (std_msgs/Float64MultiArray) consigna de orugas, para el
                                                   deslizamiento
    /politica/activa   (std_msgs/Bool)            interruptor de seguridad
Publica:
    /cmd_flippers         (std_msgs/Float64MultiArray) 4 posiciones rad
    /politica/observacion (std_msgs/Float64MultiArray) las 40 señales, en vivo
    /diagnostics          (diagnostic_msgs/DiagnosticArray) fila politica/flippers

PARÁMETROS
----------
    modelo            (string, '')   ruta al .onnx. Vacío = solo observar.
    frecuencia_hz     (double, 50.0) ritmo de inferencia y de publicación.
    activa_al_inicio  (bool, False)  true solo en banco, nunca con el robot en
                                     el suelo sin vigilancia.
    timeout_estado_s  (double, 0.5)  sin /joint_states en este tiempo, se para.
    limite_min_rad    (double, nan)  tope inferior opcional de los flippers.
    limite_max_rad    (double, nan)  tope superior opcional. Las juntas son
                                     `continuous`, así que por defecto no hay.

USO
---
    # Solo mirar las señales táctiles (sin modelo, no mueve nada):
    ros2 run ugv_bridge politica_flippers

    # Con el modelo entrenado en el PC y copiado a la Pi:
    ros2 run ugv_bridge politica_flippers --ros-args -p modelo:=/home/ros2/politica.onnx

    # Dentro del sistema completo:
    ros2 launch ugv_bridge flipper.launch.py use_politica:=true \\
        modelo_politica:=/home/ros2/politica.onnx

    # Activar desde Foxglove: panel Publish -> /politica/activa -> {"data": true}

En la Raspberry:  pip3 install onnxruntime
"""
import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray

from ugv_bridge import contrato_politica as contrato


def _rpy_de_cuaternion(x, y, z, w):
    """Cuaternión -> (roll, pitch). El yaw no entra en la observación a propósito."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


class MotorOnnx:
    """Carga y ejecuta el .onnx, verificando antes que cumple el contrato."""

    def __init__(self, ruta):
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise RuntimeError(
                'Falta onnxruntime. En la Raspberry: pip3 install onnxruntime'
            ) from e

        if not os.path.isfile(ruta):
            raise RuntimeError(f'No existe el modelo: {ruta}')

        # El sidecar es opcional pero MUY recomendable: sin él no se puede
        # comprobar la versión del contrato y solo queda validar la forma.
        self.meta = None
        ruta_json = os.path.splitext(ruta)[0] + '.json'
        if os.path.isfile(ruta_json):
            with open(ruta_json, encoding='utf-8') as f:
                self.meta = json.load(f)
            v = self.meta.get('version_contrato')
            if v != contrato.VERSION_CONTRATO:
                raise RuntimeError(
                    f'El modelo se exportó con el contrato v{v} y este código usa '
                    f'v{contrato.VERSION_CONTRATO}. Reexporta el modelo o vuelve a '
                    f'la versión correcta del código: mezclarlos no da ningún '
                    f'error, solo ángulos equivocados.')

        # Un hilo: la Pi tiene que repartir CPU con el bucle de control a 100 Hz.
        opciones = ort.SessionOptions()
        opciones.intra_op_num_threads = 1
        opciones.inter_op_num_threads = 1
        self.sesion = ort.InferenceSession(
            ruta, sess_options=opciones, providers=['CPUExecutionProvider'])

        entrada = self.sesion.get_inputs()[0]
        self.nombre_entrada = entrada.name
        dim = entrada.shape[-1]
        if isinstance(dim, int) and dim != contrato.OBS_DIM:
            raise RuntimeError(
                f'La red espera {dim} valores de entrada y el contrato produce '
                f'{contrato.OBS_DIM}. No se carga.')
        self.nombre_salida = self.sesion.get_outputs()[0].name
        self.ruta = ruta

    def inferir(self, obs):
        salida = self.sesion.run(
            [self.nombre_salida],
            {self.nombre_entrada: obs.reshape(1, -1).astype(np.float32)})[0]
        return np.asarray(salida, dtype=np.float64).ravel()[:contrato.ACC_DIM]


class PoliticaFlippers(Node):

    def __init__(self):
        super().__init__('politica_flippers')

        self.declare_parameter('modelo', '')
        self.declare_parameter('frecuencia_hz', 50.0)
        self.declare_parameter('activa_al_inicio', False)
        self.declare_parameter('timeout_estado_s', 0.5)
        self.declare_parameter('limite_min_rad', float('nan'))
        self.declare_parameter('limite_max_rad', float('nan'))

        ruta = str(self.get_parameter('modelo').value)
        self.frecuencia = float(self.get_parameter('frecuencia_hz').value)
        self.dt = 1.0 / max(self.frecuencia, 1.0)
        self.activa = bool(self.get_parameter('activa_al_inicio').value)
        self.timeout = float(self.get_parameter('timeout_estado_s').value)

        lo = float(self.get_parameter('limite_min_rad').value)
        hi = float(self.get_parameter('limite_max_rad').value)
        self.limites = None if (math.isnan(lo) or math.isnan(hi)) else (lo, hi)

        # --- Modelo (opcional) ---
        self.motor = None
        self.error_modelo = None
        if ruta:
            try:
                self.motor = MotorOnnx(ruta)
                self.get_logger().info(f'Modelo cargado: {ruta}')
            except RuntimeError as e:
                # No se aborta: sin modelo el nodo sigue siendo útil como
                # observador, y en un robot a medio montar eso vale más que un
                # arranque fallido que se lleva el launch entero por delante.
                self.error_modelo = str(e)
                self.get_logger().error(f'No se cargó el modelo: {e}')
        else:
            self.get_logger().info(
                'Sin parámetro `modelo`: modo SOLO OBSERVACIÓN. No se publicará '
                'ningún comando; mira /politica/observacion en Foxglove.')

        # --- Estado medido ---
        self.pos_flipper = [0.0] * 4
        self.vel_flipper = [0.0] * 4
        self.esf_flipper = [0.0] * 4
        self.vel_oruga = [0.0] * 4
        self.esf_oruga = [0.0] * 4
        self.cmd_oruga = [0.0] * 4
        self.roll = 0.0
        self.pitch = 0.0
        self.giro = [0.0, 0.0, 0.0]
        self.acel_z = 0.0
        self.t_estado = None
        self.hay_estado = False
        self.ciclos = 0
        self.ciclos_publicados = 0

        # La consigna vigente: de dónde parte la integración y qué se compara
        # con la posición real para sacar el error de seguimiento.
        self.consigna = [0.0] * 4
        self.consigna_sembrada = False

        self.pub_cmd = self.create_publisher(Float64MultiArray, '/cmd_flippers', 10)
        self.pub_obs = self.create_publisher(
            Float64MultiArray, '/politica/observacion', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 10)
        self.create_subscription(
            Float64MultiArray, '/cmd_tracks', self.on_cmd_tracks, 10)
        self.create_subscription(Bool, '/politica/activa', self.on_activa, 10)

        self.create_timer(self.dt, self.ciclo)
        self.create_timer(1.0, self.publicar_diagnostico)

        estado = 'ACTIVA' if self.activa else 'en espera (/politica/activa)'
        self.get_logger().info(
            f'politica_flippers lista a {self.frecuencia:.0f} Hz, {estado}. '
            f'Contrato v{contrato.VERSION_CONTRATO}, obs={contrato.OBS_DIM}, '
            f'acc={contrato.ACC_DIM}.')

    # ------------------------------------------------------------------ #
    def on_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}

        def _leer(seq, i, por_defecto=0.0):
            return float(seq[i]) if i is not None and i < len(seq) else por_defecto

        for k, junta in enumerate(contrato.JUNTAS_FLIPPER):
            i = idx.get(junta)
            if i is None:
                continue
            self.pos_flipper[k] = _leer(msg.position, i)
            self.vel_flipper[k] = _leer(msg.velocity, i)
            self.esf_flipper[k] = _leer(msg.effort, i)
        for k, junta in enumerate(contrato.JUNTAS_ORUGA):
            i = idx.get(junta)
            if i is None:
                continue
            self.vel_oruga[k] = _leer(msg.velocity, i)
            self.esf_oruga[k] = _leer(msg.effort, i)

        # La primera consigna es la pose ACTUAL, no cero. Si se sembrara en cero,
        # el primer ciclo con la política activa pediría llevar los flippers al
        # cero mecánico de golpe, estuvieran donde estuvieran.
        if not self.consigna_sembrada:
            self.consigna = list(self.pos_flipper)
            self.consigna_sembrada = True

        self.t_estado = self.get_clock().now().nanoseconds * 1e-9
        self.hay_estado = True

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch = _rpy_de_cuaternion(q.x, q.y, q.z, q.w)
        self.giro = [msg.angular_velocity.x, msg.angular_velocity.y,
                     msg.angular_velocity.z]
        self.acel_z = msg.linear_acceleration.z

    def on_cmd_tracks(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.cmd_oruga = [float(v) for v in msg.data[:4]]

    def on_activa(self, msg: Bool):
        nueva = bool(msg.data)
        if nueva == self.activa:
            return
        self.activa = nueva
        if nueva:
            # Al activarse, la consigna se resincroniza con la pose real: si no,
            # la política arrancaría integrando desde donde quedó la vez
            # anterior y el primer comando sería un salto.
            self.consigna = list(self.pos_flipper)
            self.get_logger().warn('Política ACTIVADA: ya comanda los flippers.')
        else:
            self.get_logger().info(
                'Política desactivada: deja de publicar. Los flippers se quedan '
                'en su última consigna (flipper_node la retiene).')

    # ------------------------------------------------------------------ #
    def _observacion(self):
        return contrato.construir_observacion(
            pos_flipper=self.pos_flipper,
            vel_flipper=self.vel_flipper,
            esfuerzo_flipper=self.esf_flipper,
            consigna_flipper=self.consigna,
            vel_oruga=self.vel_oruga,
            esfuerzo_oruga=self.esf_oruga,
            consigna_oruga=self.cmd_oruga,
            roll=self.roll, pitch=self.pitch,
            giro=self.giro, acel_z=self.acel_z,
        )

    def _estado_fresco(self):
        if not self.hay_estado or self.t_estado is None:
            return False
        ahora = self.get_clock().now().nanoseconds * 1e-9
        return (ahora - self.t_estado) <= self.timeout

    def ciclo(self):
        self.ciclos += 1
        if not self.hay_estado:
            return

        obs = self._observacion()
        # La observación se publica SIEMPRE, activa o no, con modelo o sin él.
        # Es el instrumento para diseñar la recompensa antes de entrenar.
        self.pub_obs.publish(Float64MultiArray(data=[float(v) for v in obs]))

        if self.motor is None or not self.activa:
            return
        if not self._estado_fresco():
            # Watchdog: sin estado fresco no se comanda. Se avisa una vez por
            # segundo, no en cada ciclo, para no inundar /rosout.
            if self.ciclos % int(max(self.frecuencia, 1)) == 0:
                self.get_logger().warn(
                    '/joint_states lleva demasiado tiempo callado: la política '
                    'no comanda hasta que vuelva.')
            return

        accion = self.motor.inferir(obs)
        self.consigna = contrato.integrar_accion(
            accion, self.pos_flipper, self.dt, self.limites)
        self.pub_cmd.publish(Float64MultiArray(data=self.consigna))
        self.ciclos_publicados += 1

    # ------------------------------------------------------------------ #
    def publicar_diagnostico(self):
        """Fila `politica/flippers` en /diagnostics, para el panel de Foxglove."""
        st = DiagnosticStatus()
        st.hardware_id = 'politica'
        st.name = 'politica/flippers'

        if self.error_modelo:
            st.level = DiagnosticStatus.ERROR
            st.message = f'modelo NO cargado: {self.error_modelo}'
        elif self.motor is None:
            st.level = DiagnosticStatus.OK
            st.message = 'solo observación (sin modelo): no comanda nada'
        elif not self.activa:
            st.level = DiagnosticStatus.WARN
            st.message = 'modelo cargado pero DESACTIVADA (/politica/activa)'
        elif not self._estado_fresco():
            st.level = DiagnosticStatus.ERROR
            st.message = 'activa pero /joint_states está mudo: watchdog frenando'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'comandando los flippers'

        st.values = [
            KeyValue(key='modelo',
                     value=self.motor.ruta if self.motor else '—'),
            KeyValue(key='contrato', value=f'v{contrato.VERSION_CONTRATO}'),
            KeyValue(key='obs_dim', value=str(contrato.OBS_DIM)),
            KeyValue(key='activa', value='sí' if self.activa else 'no'),
            KeyValue(key='frecuencia_hz', value=f'{self.frecuencia:.0f}'),
            KeyValue(key='ciclos', value=str(self.ciclos)),
            KeyValue(key='comandos_publicados', value=str(self.ciclos_publicados)),
            KeyValue(key='consigna_deg',
                     value=', '.join(f'{math.degrees(v):+.1f}'
                                     for v in self.consigna)),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(st)
        self.pub_diag.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = PoliticaFlippers()
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
