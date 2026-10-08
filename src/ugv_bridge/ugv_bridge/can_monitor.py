#!/usr/bin/env python3
"""can_monitor.py — Espía del bus CAN. SOLO LEE. Se mira desde Foxglove.

QUÉ CAMBIÓ Y POR QUÉ (lee esto antes de tocar nada)
---------------------------------------------------
La versión anterior de este archivo era una ventana Qt (PySide6) que, además de
mirar el bus, PUBLICABA `/cmd_flippers` y `/cmd_tracks` desde sus sliders. Eso
convertía al monitor en un MANDO más, y con Foxglove abierto había dos fuentes
peleando por los mismos tópicos:

    can_monitor (sliders Qt) ──┐
                               ├──▶ /cmd_flippers ──▶ flipper_node ──▶ CAN
    Foxglove (Teleop/Publish) ─┘

`flipper_node` retiene el ÚLTIMO comando que le llega, así que el resultado era
el clásico "mando el ángulo desde Foxglove y el flipper se vuelve solo": el
monitor reenviaba su propio valor (el de sus sliders, que seguían donde los
dejaste) en cuanto alguien tocaba algo, y pisaba la orden de Foxglove. Peor aún
cuando la ventana quedaba viva en segundo plano tras cerrar su launch: seguía
publicando sin que hubiera nada visible en pantalla que lo explicara.

Ahora este nodo NO PUBLICA NINGÚN COMANDO. No tiene publicadores de
`/cmd_flippers` ni de `/cmd_tracks`, no importa PySide6, no abre ventana y no
necesita servidor gráfico (corre igual en la Raspberry headless). Es un espía de
solo lectura: escucha el bus y vuelca lo que ve en tópicos ROS para que los
dibuje Foxglove.

El MANDO vive únicamente en Foxglove: paneles `Publish` (poses fijas) y paneles
`Teleop` sobre `/teleop/flipper_*` (ver `teleop_flippers.py`).

QUÉ PUBLICA Y CON QUÉ PANEL SE MIRA
-----------------------------------
  /diagnostics            diagnostic_msgs/DiagnosticArray   (1 Hz)
        Una fila por motor, nombre `can/<junta>`, más la fila `can/BUS` con el
        resumen del canal. Panel: *Diagnostics – Summary / Detail*.
        Convive con las filas `motores/...` de flipper_node: son nombres
        distintos dentro del mismo tópico estándar, que es el uso normal.

  /can/posicion_rad       std_msgs/Float64MultiArray  (8 valores, IDs 1..8)
  /can/velocidad_rad_s    std_msgs/Float64MultiArray
  /can/iq_a               std_msgs/Float64MultiArray
  /can/torque_nm          std_msgs/Float64MultiArray
  /can/cmd_posicion_rad   std_msgs/Float64MultiArray   última consigna de posición
  /can/cmd_velocidad_rad_s std_msgs/Float64MultiArray  última consigna de velocidad
        Todo en el EJE DE SALIDA y en orden de ID: índices 0..3 = orugas
        (track_fl, fr, rl, rr), 4..7 = flippers (flipper_fl, fr, rl, rr).
        Panel: *Plot*, con series tipo `/can/velocidad_rad_s.data[0]`.

  /can/tasa_tramas_hz     std_msgs/Float64    carga del bus, tramas/s
        Panel: *Plot* o *Gauge*. Si cae a 0 con el robot encendido, el bus murió.

  /can/trafico            std_msgs/String     (10 Hz)
        Las últimas N tramas decodificadas, una por línea, en texto. Es el
        equivalente a `candump` dentro de Foxglove. Panel: *Raw Messages*
        apuntando a `/can/trafico.data`.

LO QUE ESTE NODO NO PUEDE VER
-----------------------------
La TEMPERATURA no está: el protocolo ODrive la expone por un mensaje que viene
APAGADO de fábrica (como el de Iq, ver `protocolo_can.py`). La versión anterior
tenía una columna "temp" que nunca se rellenaba porque decodificaba mensajes
`RESP_ESTADO`/`RESP_MULTIV` del protocolo RMD viejo, que ya no existe. Igual con
Iq/torque: salen en 0 salvo que se active `iq_rate_ms` por USB con odrivetool.

PARÁMETROS
----------
  canal              (string, 'vcan0') interfaz SocketCAN a espiar.
  frecuencia_estado_hz (double, 10.0)  ritmo de /can/* numéricos y del tráfico.
  frecuencia_diag_hz   (double, 1.0)   ritmo de /diagnostics (lo lee una persona).
  lineas_trafico     (int, 25)         tramas por mensaje de /can/trafico.
  publicar_trafico   (bool, True)      False = ahorra ancho de banda del puente.
  timeout_motor_s    (double, 2.0)     sin tramas en este tiempo, el motor es STALE.

USO
---
    ros2 run ugv_bridge can_monitor --ros-args -p canal:=vcan0
    ros2 run ugv_bridge can_monitor --canal can0        # atajo equivalente
    ros2 launch ugv_bridge can_studio.launch.py         # simulación + espía + Foxglove

Requisitos: python3-can y la interfaz arriba (`scripts/setup_vcan.sh` para vcan0).
"""
import argparse
import collections
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64, Float64MultiArray, String

from ugv_bridge import protocolo_can as proto
from ugv_bridge.driver_movimiento import (
    ID_A_NOMBRE, IDS_ORUGAS, IDS_FLIPPERS,
)

# Orden canónico de los arrays que se publican: primero orugas, luego flippers.
TODOS = IDS_ORUGAS + IDS_FLIPPERS

# Nombre legible de cada cmd_id (los 5 bits bajos del ID de arbitraje).
NOMBRE_CMD = {
    proto.CMD_HEARTBEAT: 'HEARTBEAT',
    proto.CMD_ESTOP: 'ESTOP',
    proto.CMD_SET_AXIS_STATE: 'SET_ESTADO',
    proto.CMD_GET_ENCODER_ESTIMATES: 'ENCODER',
    proto.CMD_SET_CONTROLLER_MODE: 'SET_MODO',
    proto.CMD_SET_INPUT_POS: 'POSICION',
    proto.CMD_SET_INPUT_VEL: 'VELOCIDAD',
    proto.CMD_SET_LIMITS: 'SET_LIMITES',
    proto.CMD_GET_IQ: 'IQ',
    proto.CMD_CLEAR_ERRORS: 'CLEAR_ERR',
}

# Qué mensajes van del host al motor; el resto los emite el motor.
CMDS_AL_MOTOR = {
    proto.CMD_ESTOP, proto.CMD_SET_AXIS_STATE, proto.CMD_SET_CONTROLLER_MODE,
    proto.CMD_SET_INPUT_POS, proto.CMD_SET_INPUT_VEL, proto.CMD_SET_LIMITS,
    proto.CMD_CLEAR_ERRORS,
}

NOMBRE_ESTADO = {
    proto.ESTADO_IDLE: 'IDLE',
    proto.ESTADO_CLOSED_LOOP: 'LAZO CERRADO',
}


def _rad2deg(r):
    return r * 180.0 / math.pi


def decodificar(arb_id, data):
    """Trama CAN cruda -> dict con lo que se pueda sacar de ella.

    Devuelve None si el ID no corresponde a ningún motor conocido (así el espía
    ignora tráfico ajeno en un bus compartido en vez de ensuciar la tabla).

    Claves siempre presentes: 'sentido' ('->' host->motor, '<-' motor->host),
    'id', 'tipo', 'texto'. Las demás son opcionales y solo aparecen cuando la
    trama realmente las trae: 'posicion_rad', 'velocidad_rad_s', 'iq_a',
    'cmd_posicion_rad', 'cmd_velocidad_rad_s', 'estado_eje', 'error_eje'.
    """
    nodo = proto.nodo_de_id(arb_id)
    if nodo not in TODOS:
        return None
    cmd_id = proto.cmd_de_id(arb_id)
    out = {'id': nodo, 'tipo': NOMBRE_CMD.get(cmd_id, f'cmd 0x{cmd_id:03X}')}

    # ------------------------------------------------ host -> motor (consignas)
    if cmd_id in CMDS_AL_MOTOR:
        out['sentido'] = '->'
        info = proto.parsear_comando(cmd_id, data)
        if info is None:
            out['texto'] = '—'
        elif cmd_id == proto.CMD_SET_INPUT_VEL:
            out['cmd_velocidad_rad_s'] = info['velocidad_rad_s']
            out['texto'] = f"{info['velocidad_rad_s']:+.2f} rad/s"
        elif cmd_id == proto.CMD_SET_INPUT_POS:
            out['cmd_posicion_rad'] = info['posicion_rad']
            out['texto'] = f"{_rad2deg(info['posicion_rad']):+.1f}°"
        elif cmd_id == proto.CMD_SET_AXIS_STATE:
            out['texto'] = NOMBRE_ESTADO.get(info['estado'], str(info['estado']))
        elif cmd_id == proto.CMD_SET_CONTROLLER_MODE:
            out['texto'] = f"control={info['control_mode']} entrada={info['input_mode']}"
        elif cmd_id == proto.CMD_SET_LIMITS:
            out['texto'] = (f"vel<={info['vel_limite_rad_s']:.2f} rad/s  "
                            f"I<={info['corriente_limite_a']:.1f} A")
        else:
            out['texto'] = '—'
        return out

    # ------------------------------------------------ motor -> host (telemetría)
    out['sentido'] = '<-'
    info = proto.parsear(cmd_id, data)
    if info is None:
        out['texto'] = '—'
        return out

    if cmd_id == proto.CMD_GET_ENCODER_ESTIMATES:
        out['posicion_rad'] = info['posicion_rad']
        out['velocidad_rad_s'] = info['velocidad_rad_s']
        out['texto'] = (f"pos={_rad2deg(info['posicion_rad']):+.1f}°  "
                        f"v={info['velocidad_rad_s']:+.2f} rad/s")
    elif cmd_id == proto.CMD_HEARTBEAT:
        out['estado_eje'] = info['estado_eje']
        out['error_eje'] = info['error_eje']
        nombre = NOMBRE_ESTADO.get(info['estado_eje'], str(info['estado_eje']))
        err = '' if not info['error_eje'] else f"  ERROR 0x{info['error_eje']:X}"
        out['texto'] = f'{nombre}{err}'
    elif cmd_id == proto.CMD_GET_IQ:
        out['iq_a'] = info['iq_medido_a']
        out['texto'] = f"Iq={info['iq_medido_a']:+.2f} A"
    else:
        out['texto'] = '—'
    return out


def _estado_vacio():
    """Fila por motor. None = "todavía no se ha visto", distinto de 0.0."""
    return {
        'posicion_rad': None,
        'velocidad_rad_s': None,
        'iq_a': None,
        'cmd_posicion_rad': None,
        'cmd_velocidad_rad_s': None,
        'ultimo_cmd': '—',
        'valor_cmd': '—',
        'estado_eje': None,
        'error_eje': 0,
        'tramas': 0,
        't_ultima': None,       # time.monotonic() de la última trama DEL motor
    }


class CanMonitor(Node):
    """Espía de solo lectura del bus CAN, con salida pensada para Foxglove."""

    def __init__(self, canal_cli=None):
        super().__init__('can_monitor')

        self.declare_parameter('canal', 'vcan0')
        self.declare_parameter('frecuencia_estado_hz', 10.0)
        self.declare_parameter('frecuencia_diag_hz', 1.0)
        self.declare_parameter('lineas_trafico', 25)
        self.declare_parameter('publicar_trafico', True)
        self.declare_parameter('timeout_motor_s', 2.0)

        # El atajo --canal de la línea de comandos gana sobre el parámetro, que
        # es lo que espera quien escribe `ros2 run ... --canal can0`.
        self.canal = canal_cli or self.get_parameter('canal').value
        self.lineas_trafico = int(self.get_parameter('lineas_trafico').value)
        self.publicar_trafico = bool(self.get_parameter('publicar_trafico').value)
        self.timeout_motor = float(self.get_parameter('timeout_motor_s').value)
        f_estado = float(self.get_parameter('frecuencia_estado_hz').value)
        f_diag = float(self.get_parameter('frecuencia_diag_hz').value)

        # --- Publicadores. NINGUNO es de comando: este nodo no manda nada. ---
        self.pub_pos = self.create_publisher(Float64MultiArray, '/can/posicion_rad', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/can/velocidad_rad_s', 10)
        self.pub_iq = self.create_publisher(Float64MultiArray, '/can/iq_a', 10)
        self.pub_torque = self.create_publisher(Float64MultiArray, '/can/torque_nm', 10)
        self.pub_cmd_pos = self.create_publisher(
            Float64MultiArray, '/can/cmd_posicion_rad', 10)
        self.pub_cmd_vel = self.create_publisher(
            Float64MultiArray, '/can/cmd_velocidad_rad_s', 10)
        self.pub_tasa = self.create_publisher(Float64, '/can/tasa_tramas_hz', 10)
        self.pub_trafico = self.create_publisher(String, '/can/trafico', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # --- Estado interno ---
        self.estado = {mid: _estado_vacio() for mid in TODOS}
        self.cola = collections.deque(maxlen=8000)
        self.trafico = collections.deque(maxlen=self.lineas_trafico)
        self.total = 0
        self._cuenta_ventana = 0
        self._t0 = time.monotonic()
        self._t_tasa = self._t0
        self.tasa_hz = 0.0
        self._parar = threading.Event()

        # --- Socket del bus ---
        try:
            import can
        except ImportError:
            raise RuntimeError(
                'Falta python3-can. Instalar: sudo apt install python3-can')
        try:
            self.bus = can.interface.Bus(channel=self.canal, interface='socketcan')
        except OSError as e:
            raise RuntimeError(
                f'No se pudo abrir {self.canal}: {e}. '
                f'¿Existe la interfaz? Crear vcan0 con scripts/setup_vcan.sh')

        self._hilo = threading.Thread(target=self._lector, daemon=True)
        self._hilo.start()

        self.create_timer(1.0 / max(f_estado, 0.1), self.publicar_estado)
        self.create_timer(1.0 / max(f_diag, 0.1), self.publicar_diagnostico)

        self.get_logger().info(
            f'can_monitor (SOLO LECTURA) espiando {self.canal}. '
            f'No publica comandos: el mando es Foxglove. '
            f'Tópicos: /can/* y /diagnostics (filas can/...).')

    # ------------------------------------------------------------------ #
    def _lector(self):
        """Hilo que vacía el socket. Solo recibe; nunca transmite."""
        while not self._parar.is_set():
            try:
                msg = self.bus.recv(timeout=0.2)
            except OSError:
                break
            if msg is not None:
                self.cola.append((time.monotonic(), msg.arbitration_id,
                                  bytes(msg.data)))

    def _drenar(self):
        """Pasa lo acumulado por el hilo lector al estado por motor."""
        while self.cola:
            t, arb_id, data = self.cola.popleft()
            self.total += 1
            self._cuenta_ventana += 1
            info = decodificar(arb_id, data)
            if info is None:
                continue
            e = self.estado[info['id']]
            e['tramas'] += 1
            e['t_ultima'] = t
            if info['sentido'] == '->':
                e['ultimo_cmd'] = info['tipo']
                e['valor_cmd'] = info['texto']
            for clave in ('posicion_rad', 'velocidad_rad_s', 'iq_a',
                          'cmd_posicion_rad', 'cmd_velocidad_rad_s',
                          'estado_eje', 'error_eje'):
                if clave in info:
                    e[clave] = info[clave]
            if self.publicar_trafico:
                self.trafico.append(
                    f"{t - self._t0:8.2f}  {info['sentido']}  "
                    f"0x{arb_id:03X}  {ID_A_NOMBRE[info['id']]:<11}  "
                    f"{info['tipo']:<11}  {info['texto']:<28}  {data.hex()}")

    def _vector(self, clave, factor=1.0):
        """Array de 8 en orden de ID. Lo no visto va como 0.0 (Plot no come NaN)."""
        return [0.0 if self.estado[m][clave] is None
                else float(self.estado[m][clave]) * factor
                for m in TODOS]

    # ------------------------------------------------------------------ #
    def publicar_estado(self):
        self._drenar()

        ahora = time.monotonic()
        if ahora - self._t_tasa >= 1.0:
            self.tasa_hz = self._cuenta_ventana / (ahora - self._t_tasa)
            self._cuenta_ventana = 0
            self._t_tasa = ahora

        self.pub_pos.publish(Float64MultiArray(data=self._vector('posicion_rad')))
        self.pub_vel.publish(Float64MultiArray(data=self._vector('velocidad_rad_s')))
        self.pub_iq.publish(Float64MultiArray(data=self._vector('iq_a')))
        self.pub_torque.publish(Float64MultiArray(
            data=self._vector('iq_a', proto.KT_NM_POR_A)))
        self.pub_cmd_pos.publish(Float64MultiArray(
            data=self._vector('cmd_posicion_rad')))
        self.pub_cmd_vel.publish(Float64MultiArray(
            data=self._vector('cmd_velocidad_rad_s')))
        self.pub_tasa.publish(Float64(data=float(self.tasa_hz)))

        if self.publicar_trafico and self.trafico:
            self.pub_trafico.publish(String(data='\n'.join(self.trafico)))

    # ------------------------------------------------------------------ #
    def publicar_diagnostico(self):
        """Una fila por motor vista DESDE EL BUS, más el resumen del canal.

        Ojo con la diferencia respecto a las filas `motores/...` que publica
        flipper_node: aquellas son lo que CREE el driver; estas son lo que de
        verdad circula por el cable. Cuando las dos discrepan, el problema está
        entre el driver y el bus (interfaz caída, bitrate, node_id).
        """
        ahora = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        vistos = 0

        for mid in TODOS:
            e = self.estado[mid]
            st = DiagnosticStatus()
            st.hardware_id = f'motor {mid}'
            st.name = f'can/{ID_A_NOMBRE[mid]}'
            edad = None if e['t_ultima'] is None else ahora - e['t_ultima']

            if e['t_ultima'] is None:
                st.level = DiagnosticStatus.ERROR
                st.message = 'MUDO: nunca emitió en el bus'
            elif edad > self.timeout_motor:
                st.level = DiagnosticStatus.STALE
                st.message = f'SIN TRAMAS desde hace {edad:.1f} s'
            else:
                vistos += 1
                if e['error_eje']:
                    st.level = DiagnosticStatus.WARN
                    st.message = f"emite, con error 0x{e['error_eje']:X}"
                elif e['estado_eje'] not in (None, proto.ESTADO_CLOSED_LOOP):
                    st.level = DiagnosticStatus.WARN
                    st.message = ('FUERA DE LAZO CERRADO: acepta las órdenes '
                                  'y no se mueve')
                else:
                    st.level = DiagnosticStatus.OK
                    st.message = 'emitiendo en el bus'

            def _f(v, fmt='{:+.4f}'):
                return '—' if v is None else fmt.format(v)

            st.values = [
                KeyValue(key='id', value=str(mid)),
                KeyValue(key='tipo',
                         value='oruga' if mid in IDS_ORUGAS else 'flipper'),
                KeyValue(key='tramas', value=str(e['tramas'])),
                KeyValue(key='edad_s', value='—' if edad is None else f'{edad:.2f}'),
                KeyValue(key='ultimo_cmd', value=e['ultimo_cmd']),
                KeyValue(key='valor_cmd', value=e['valor_cmd']),
                KeyValue(key='posicion_deg',
                         value=_f(None if e['posicion_rad'] is None
                                  else _rad2deg(e['posicion_rad']), '{:+.1f}')),
                KeyValue(key='velocidad_rad_s', value=_f(e['velocidad_rad_s'])),
                KeyValue(key='iq_a', value=_f(e['iq_a'], '{:+.2f}')),
                KeyValue(key='estado_eje',
                         value='—' if e['estado_eje'] is None else
                               NOMBRE_ESTADO.get(e['estado_eje'], str(e['estado_eje']))),
                KeyValue(key='error_eje', value=f"0x{e['error_eje']:X}"),
            ]
            msg.status.append(st)

        # Fila de resumen del canal: responde "¿el bus está vivo?" sin contar filas.
        resumen = DiagnosticStatus()
        resumen.hardware_id = self.canal
        resumen.name = 'can/BUS'
        if self.total == 0:
            resumen.level = DiagnosticStatus.ERROR
            resumen.message = f'{self.canal}: SIN TRÁFICO desde el arranque'
        elif self.tasa_hz < 1.0:
            resumen.level = DiagnosticStatus.WARN
            resumen.message = f'{self.canal}: tráfico casi parado ({self.tasa_hz:.1f}/s)'
        else:
            resumen.level = DiagnosticStatus.OK
            resumen.message = (f'{self.canal}: {vistos}/{len(TODOS)} motores '
                               f'emitiendo, {self.tasa_hz:.0f} tramas/s')
        resumen.values = [
            KeyValue(key='canal', value=self.canal),
            KeyValue(key='tramas_totales', value=str(self.total)),
            KeyValue(key='tramas_por_segundo', value=f'{self.tasa_hz:.1f}'),
            KeyValue(key='motores_emitiendo', value=f'{vistos}/{len(TODOS)}'),
            KeyValue(key='publica_comandos', value='no (espía de solo lectura)'),
        ]
        msg.status.append(resumen)

        self.pub_diag.publish(msg)

    # ------------------------------------------------------------------ #
    def cerrar(self):
        self._parar.set()
        # Esperar al lector antes de cerrar el bus: si se cierra con el hilo
        # dentro de recv(), python-can lanza CanOperationError (que no es
        # OSError) y el Ctrl+C deja un traceback en la consola. ros2 launch
        # reenvía el SIGINT, así que un segundo Ctrl+C puede caer aquí mismo.
        try:
            self._hilo.join(timeout=1.0)
        except KeyboardInterrupt:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Espía de solo lectura del bus CAN; se mira desde Foxglove')
    parser.add_argument('--canal', default=None,
                        help='interfaz SocketCAN (si se omite, usa el parámetro '
                             'ROS `canal`, por defecto vcan0)')
    args, resto = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=resto)
    try:
        nodo = CanMonitor(canal_cli=args.canal)
    except RuntimeError as e:
        print(f'[can_monitor] {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
