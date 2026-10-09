#!/usr/bin/env python3
r"""politica_flippers.py — Ejecuta en el robot la política entrenada fuera de él.

SOLO INFERENCIA. Aquí no hay gymnasium, ni stable-baselines3, ni torch, ni
recompensa. El entrenamiento vive en otro equipo; al robot llega un `.onnx`
más su `.json` con el CONTRATO, y este nodo los ejecuta con `onnxruntime`.

    PC de entrenamiento                 Robot (Raspberry / Jetson)
    -------------------                 --------------------------
    simulador + RL           .onnx      este nodo
    exportar_onnx.py  ───────────────▶  lee /joint_states, /imu/data_raw
                             .json      arma la observación según el contrato
                                        ejecuta la red
                                        publica /cmd_flippers y /cmd_tracks

La política es AUTÓNOMA: maneja flippers y orugas. Foxglove queda para
activarla, pararla y mirar.

ESTE NODO ES SOLO PEGAMENTO ROS
-------------------------------
  - QUÉ entra a la red y QUÉ significa lo que sale: `contrato_politica.py`. Es
    un dato que viaja con el modelo; adaptarse a otro modelo es escribir su
    .json, no tocar este archivo.
  - CUÁNDO se le deja comandar y qué se hace si algo falla:
    `ejecutor_politica.py`, que se prueba sin ROS. Arranca desactivada, tiene
    watchdog, topes duros propios y PARADA SEGURA: al desactivarse o fallar
    manda orugas a 0 y flippers quietos (flipper_node retiene el último
    comando, así que sin eso el robot seguiría andando).

SIRVE AUNQUE TODAVÍA NO HAYA MODELO
-----------------------------------
Sin `modelo`, no comanda nada pero publica `/politica/observacion`: las señales
normalizadas del contrato, en vivo (el provisional, o el de `contrato`). Sirve
para mirar en Foxglove cómo responden el error de seguimiento y el esfuerzo de
los flippers al pasar por un obstáculo, antes de entrenar nada.

TÓPICOS
-------
Suscribe:
    /joint_states      (sensor_msgs/JointState)      posición, velocidad, esfuerzo
    /imu/data_raw      (sensor_msgs/Imu)             actitud, giro, aceleración
    /cmd_flippers      (std_msgs/Float64MultiArray)  consigna vigente (de otro o eco)
    /cmd_tracks        (std_msgs/Float64MultiArray)  ídem, para el deslizamiento
    /politica/activa   (std_msgs/Bool)               interruptor de seguridad
Publica:
    /cmd_flippers         (std_msgs/Float64MultiArray) 4 posiciones rad
    /cmd_tracks           (std_msgs/Float64MultiArray) 4 velocidades rad/s
    /politica/observacion (std_msgs/Float64MultiArray) la entrada de la red, en vivo
    /diagnostics          (diagnostic_msgs/DiagnosticArray) fila politica/flippers

PARÁMETROS
----------
    modelo            (string, '')   ruta al .onnx; su contrato es el .json de al
                                     lado. Vacío = solo observar.
    contrato          (string, '')   sin modelo: contrato con el que observar.
                                     Vacío = el provisional.
    frecuencia_hz     (double, 0.0)  0 = la del contrato (o 50 si no la dice). La
                                     red se entrenó a un ritmo: conviene respetarlo.
    activa_al_inicio  (bool, False)  true solo en banco, nunca con el robot en
                                     el suelo sin vigilancia.
    timeout_estado_s  (double, 0.5)  sin /joint_states en este tiempo, para.
    vel_max_oruga     (double, 6.0)  tope duro rad/s, gane lo que gane el contrato.
    vel_max_flipper   (double, 1.5)  tope duro rad/s de cada flipper.
    limite_min_rad    (double, nan)  tope inferior opcional de los flippers.
    limite_max_rad    (double, nan)  tope superior opcional. Las juntas son
                                     `continuous`, así que por defecto no hay.

USO
---
    # Solo mirar las señales (sin modelo, no mueve nada):
    ros2 run ugv_bridge politica_flippers

    # Con un modelo (el .json tiene que estar al lado):
    ros2 run ugv_bridge politica_flippers --ros-args -p modelo:=/home/ros2/politica.onnx

    # Dentro del sistema completo, por ejemplo con el modelo falso:
    python3 src/ugv_bridge/scripts/modelo_falso.py -s /tmp/falso
    ros2 launch ugv_bridge can_sim.launch.py use_foxglove:=true \
        use_politica:=true modelo_politica:=/tmp/falso.onnx

    # Activar desde Foxglove: panel Publish -> /politica/activa -> {"data": true}

Hace falta onnxruntime donde corra este nodo (PC, Pi o contenedor). En Ubuntu
24.04 pip se niega a instalar en el sistema (PEP 668), y un venv no sirve
porque ros2 usa el python3 del sistema:
    pip3 install --user --break-system-packages onnxruntime "numpy<2"
"numpy<2" conserva el numpy 1.26 contra el que están compilados los paquetes
de ROS.
"""
import math
import signal

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray

from ugv_bridge import contrato_politica
from ugv_bridge.ejecutor_politica import EjecutorPolitica, huella_texto, ModeloOnnx


def _rpy_de_cuaternion(x, y, z, w):
    """Cuaternión -> (roll, pitch). El yaw no se usa: depende de dónde está el robot."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


class PoliticaFlippers(Node):

    def __init__(self):
        super().__init__('politica_flippers')

        self.declare_parameter('modelo', '')
        self.declare_parameter('contrato', '')
        self.declare_parameter('frecuencia_hz', 0.0)
        self.declare_parameter('activa_al_inicio', False)
        self.declare_parameter('timeout_estado_s', 0.5)
        self.declare_parameter('vel_max_oruga', 6.0)
        self.declare_parameter('vel_max_flipper', 1.5)
        self.declare_parameter('limite_min_rad', float('nan'))
        self.declare_parameter('limite_max_rad', float('nan'))

        # --- Modelo y contrato ---
        modelo, contrato = None, None
        self.error_modelo = None
        ruta = str(self.get_parameter('modelo').value)
        if ruta:
            try:
                modelo = ModeloOnnx(ruta)
                contrato = modelo.contrato
                self.get_logger().info(
                    f'Modelo cargado: {ruta}, contrato {huella_texto(contrato)}')
            except RuntimeError as e:
                # No se aborta: sin modelo el nodo sigue siendo útil como
                # observador, y en un robot a medio montar eso vale más que un
                # arranque fallido que se lleva el launch entero por delante.
                self.error_modelo = str(e)
                self.get_logger().error(f'No se cargó el modelo: {e}')
        if contrato is None:
            contrato = self._contrato_de_observacion()
            if not ruta:
                self.get_logger().info(
                    'Sin parámetro `modelo`: modo SOLO OBSERVACIÓN. No se publicará '
                    'ningún comando; mira /politica/observacion en Foxglove.')

        frecuencia = float(self.get_parameter('frecuencia_hz').value)
        if frecuencia <= 0.0:
            frecuencia = contrato.frecuencia_hz or 50.0
        elif contrato.frecuencia_hz and abs(frecuencia - contrato.frecuencia_hz) > 1e-6:
            self.get_logger().warn(
                f'frecuencia_hz={frecuencia:g} y el contrato dice '
                f'{contrato.frecuencia_hz:g}: la red se entrenó a otro ritmo.')
        self.frecuencia = frecuencia

        lo = float(self.get_parameter('limite_min_rad').value)
        hi = float(self.get_parameter('limite_max_rad').value)
        self.ej = EjecutorPolitica(
            contrato, modelo, dt=1.0 / frecuencia,
            timeout_estado=float(self.get_parameter('timeout_estado_s').value),
            vel_max_oruga=float(self.get_parameter('vel_max_oruga').value),
            vel_max_flipper=float(self.get_parameter('vel_max_flipper').value),
            limites_flipper=None if (math.isnan(lo) or math.isnan(hi)) else (lo, hi))
        if contrato.senales_extra:
            self.get_logger().warn(
                f'El contrato pide sensores que este nodo todavía no lee: '
                f'{contrato.senales_extra}. La política no podrá comandar.')

        self.ciclos = 0
        self.error_avisado = None

        self.pub_flippers = self.create_publisher(Float64MultiArray, '/cmd_flippers', 10)
        self.pub_orugas = self.create_publisher(Float64MultiArray, '/cmd_tracks', 10)
        self.pub_obs = self.create_publisher(
            Float64MultiArray, '/politica/observacion', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, 10)
        self.create_subscription(
            Float64MultiArray, '/cmd_flippers', self.on_cmd_flippers, 10)
        self.create_subscription(
            Float64MultiArray, '/cmd_tracks', self.on_cmd_tracks, 10)
        self.create_subscription(Bool, '/politica/activa', self.on_activa, 10)

        if bool(self.get_parameter('activa_al_inicio').value):
            self.ej.activar(True)

        self.create_timer(1.0 / frecuencia, self.ciclo)
        self.create_timer(1.0, self.publicar_diagnostico)

        estado = 'ACTIVA' if self.ej.activa else 'en espera (/politica/activa)'
        self.get_logger().info(
            f'politica_flippers lista a {frecuencia:.0f} Hz, {estado}. '
            f'obs={contrato.obs_dim}, acc={contrato.acc_dim}.')

    def _contrato_de_observacion(self):
        ruta = str(self.get_parameter('contrato').value)
        if ruta:
            try:
                return contrato_politica.Contrato.desde_json(ruta)
            except (OSError, ValueError) as e:
                self.get_logger().error(
                    f'No se pudo leer el contrato {ruta}: {e}. Uso el provisional.')
        return contrato_politica.contrato_provisional()

    # ------------------------------------------------------------------ #
    def _ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        e = self.ej.estado

        def _leer(seq, i):
            return float(seq[i]) if i < len(seq) else 0.0

        for k, junta in enumerate(contrato_politica.JUNTAS_FLIPPER):
            i = idx.get(junta)
            if i is None:
                continue
            e.pos_flipper[k] = _leer(msg.position, i)
            e.vel_flipper[k] = _leer(msg.velocity, i)
            e.esf_flipper[k] = _leer(msg.effort, i)
        for k, junta in enumerate(contrato_politica.JUNTAS_ORUGA):
            i = idx.get(junta)
            if i is None:
                continue
            e.vel_oruga[k] = _leer(msg.velocity, i)
            e.esf_oruga[k] = _leer(msg.effort, i)
        self.ej.estado_actualizado(self._ahora())

    def on_imu(self, msg: Imu):
        e = self.ej.estado
        q = msg.orientation
        e.roll, e.pitch = _rpy_de_cuaternion(q.x, q.y, q.z, q.w)
        e.giro = [msg.angular_velocity.x, msg.angular_velocity.y,
                  msg.angular_velocity.z]
        e.acel = [msg.linear_acceleration.x, msg.linear_acceleration.y,
                  msg.linear_acceleration.z]

    def on_cmd_flippers(self, msg: Float64MultiArray):
        self.ej.consigna_externa(flippers=list(msg.data))

    def on_cmd_tracks(self, msg: Float64MultiArray):
        self.ej.consigna_externa(orugas=list(msg.data))

    def on_activa(self, msg: Bool):
        if bool(msg.data) and self.ej.modelo is None:
            self.get_logger().error('No hay modelo cargado: no se puede activar.')
            return
        if not self.ej.activar(msg.data):
            return
        if self.ej.activa:
            self.get_logger().warn('Política ACTIVADA: comanda flippers y orugas.')
        else:
            self.get_logger().info(
                'Política desactivada: orugas a 0 y flippers quietos donde están.')

    # ------------------------------------------------------------------ #
    def ciclo(self):
        self.ciclos += 1
        s = self.ej.paso(self._ahora())
        if s.observacion is not None:
            self.pub_obs.publish(Float64MultiArray(data=[float(v) for v in s.observacion]))
        if s.flippers is not None:
            self.pub_flippers.publish(Float64MultiArray(data=s.flippers))
        if s.orugas is not None:
            self.pub_orugas.publish(Float64MultiArray(data=s.orugas))

        if self.ej.error and self.ej.error != self.error_avisado:
            self.get_logger().error(f'Política DESACTIVADA y robot parado: {self.ej.error}')
            self.error_avisado = self.ej.error
        if (self.ej.frenando and self.ej.comandando
                and self.ciclos % int(max(self.frecuencia, 1)) == 0):
            # Una vez por segundo, no en cada ciclo, para no inundar /rosout.
            self.get_logger().warn(
                '/joint_states lleva demasiado tiempo callado: robot parado hasta '
                'que vuelva.')

    def parar_al_salir(self):
        """Si se cierra comandando, deja el robot parado: flipper_node retiene el comando.

        Es lo mejor que se puede hacer desde aquí, no una garantía: si el
        contexto de rclpy ya se cerró, el mensaje no sale. Un proceso que muere
        de golpe (kill -9, corte de luz de la Pi) tampoco llega hasta aquí.
        """
        if not self.ej.comandando:
            return
        try:
            self.pub_orugas.publish(Float64MultiArray(data=[0.0] * 4))
            self.pub_flippers.publish(
                Float64MultiArray(data=[float(v) for v in self.ej.estado.pos_flipper]))
        except Exception:  # noqa: B902 - al cerrar, cualquier fallo da igual
            pass

    def publicar_diagnostico(self):
        """Fila `politica/flippers` en /diagnostics, para el panel de Foxglove."""
        ej = self.ej
        st = DiagnosticStatus()
        st.hardware_id = 'politica'
        st.name = 'politica/flippers'

        if self.error_modelo:
            st.level = DiagnosticStatus.ERROR
            st.message = f'modelo NO cargado: {self.error_modelo}'
        elif ej.error:
            st.level = DiagnosticStatus.ERROR
            st.message = f'desactivada por fallo: {ej.error}'
        elif ej.modelo is None:
            st.level = DiagnosticStatus.OK
            st.message = 'solo observación (sin modelo): no comanda nada'
        elif not ej.activa:
            st.level = DiagnosticStatus.WARN
            st.message = 'modelo cargado pero DESACTIVADA (/politica/activa)'
        elif ej.frenando:
            st.level = DiagnosticStatus.ERROR
            st.message = 'activa pero /joint_states está mudo: robot parado'
        else:
            st.level = DiagnosticStatus.OK
            st.message = 'comandando flippers y orugas'
        if ej.ultimo_error_obs and not ej.error:
            st.level = max(st.level, DiagnosticStatus.WARN)
            st.message += f' | observación incompleta: {ej.ultimo_error_obs}'

        e = ej.estado
        st.values = [
            KeyValue(key='modelo', value=ej.modelo.ruta if ej.modelo else '—'),
            KeyValue(key='contrato', value=huella_texto(ej.contrato)),
            KeyValue(key='obs_dim', value=str(ej.contrato.obs_dim)),
            KeyValue(key='acc_dim', value=str(ej.contrato.acc_dim)),
            KeyValue(key='activa', value='sí' if ej.activa else 'no'),
            KeyValue(key='frecuencia_hz', value=f'{self.frecuencia:.0f}'),
            KeyValue(key='ciclos', value=str(self.ciclos)),
            KeyValue(key='comandos_publicados', value=str(ej.ciclos_publicados)),
            KeyValue(key='consigna_flippers_deg',
                     value=', '.join(f'{math.degrees(v):+.1f}'
                                     for v in e.consigna_flipper)),
            KeyValue(key='consigna_orugas_rad_s',
                     value=', '.join(f'{v:+.2f}' for v in e.consigna_oruga)),
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
        # Un Ctrl+C llega dos veces: el de la terminal y el que reenvía
        # ros2 launch. Si el segundo cae aquí, corta la limpieza a medias.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        nodo.parar_al_salir()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
