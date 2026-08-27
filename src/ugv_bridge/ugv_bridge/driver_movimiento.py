#!/usr/bin/env python3
"""driver_movimiento.py — Gemelo digital del hardware de motores + IMU.

Capa de abstracción sobre la placa mjbots pi3hat (SPI -> CAN + IMU) y los 8
motores SteadyWin (GIM6010-8 / GDS68) del UGV con orugas articuladas:

    - IDs 1-4  -> ORUGAS   (se comandan por VELOCIDAD, rad/s)
    - IDs 5-8  -> FLIPPERS  (se comandan por POSICIÓN, radianes)

El nodo de ROS (`flipper_node`) llama a `enviar_y_leer_estado(...)` y `leer_imu()`
a alta frecuencia (100-200 Hz) sin saber qué backend hay detrás. Hay tres modos:

    modo='gemelo'  -> gemelo digital: modelo cinemático con ruido, sin hardware
                      ni bus. Todo en memoria (el modo original).
    modo='can'     -> tramas CAN REALES vía SocketCAN (python3-can). Pensado para
                      la interfaz virtual vcan0 con `motor_emulator` al otro lado:
                      ejercita el empaquetado/desempaquetado de bytes del
                      protocolo (protocolo_can.py) sin hardware. Sirve igual con
                      un adaptador CAN físico (can0).
    modo='pi3hat'  -> motores reales por los buses CAN del pi3hat (SPI).

Los modos 'can' y 'pi3hat' comparten EXACTAMENTE el mismo código de protocolo
(`_estado_via_bus`): sólo cambia el transporte que lleva los bytes. Lo que se
valida hoy contra `motor_emulator` es lo que correrá contra los motores.

--------------------------------------------------------------------------
FUENTE DE LA IMU (independiente del modo de motor)
--------------------------------------------------------------------------
La IMU vive en el pi3hat y llega por SPI, NO por el bus CAN. Por eso su fuente es
un parámetro aparte, combinable con cualquier modo de motor:

    imu_fuente='sintetica'   -> IMU modelada desde el estado de los motores.
    imu_fuente='pi3hat_real' -> IMU física de la placa (moteus_pi3hat).

Combinaciones útiles:

    modo='can'    + imu_fuente='sintetica'    -> todo emulado (desarrollo en PC).
    modo='can'    + imu_fuente='pi3hat_real'  -> SITUACIÓN ACTUAL: el pi3hat ya
                                                 está montado y su IMU funciona,
                                                 pero los motores no han llegado.
    modo='pi3hat' + imu_fuente='pi3hat_real'  -> estado final, todo real.

La IMU sintética NO es ruido suelto: su actitud (roll/pitch/yaw) se integra a
partir del estado de los motores (`_actualizar_actitud`), de modo que el rumbo
(yaw) coincide con la odometría de las orugas y el cabeceo/alabeo (pitch/roll)
responden a la asimetría de los flippers. Así el pipeline (EKF, RViz, GCS) ve una
actitud que refleja el movimiento de verdad.

La clase devuelve **diccionarios Python planos**, NO mensajes ROS. El mapeo a
`sensor_msgs/JointState` e `Imu` (con conversión Euler->cuaternión) vive en el nodo.
"""
import math
import random
import time

from ugv_bridge import protocolo_can as proto

# IDs fijos acordados con el equipo de hardware.
IDS_ORUGAS = [1, 2, 3, 4]
IDS_FLIPPERS = [5, 6, 7, 8]

# Nombres de junta usados en JointState / URDF / TF2 (mismo orden que los IDs).
NOMBRES_ORUGAS = ['track_fl', 'track_fr', 'track_rl', 'track_rr']
NOMBRES_FLIPPERS = ['flipper_fl', 'flipper_fr', 'flipper_rl', 'flipper_rr']

ID_A_NOMBRE = dict(zip(IDS_ORUGAS + IDS_FLIPPERS, NOMBRES_ORUGAS + NOMBRES_FLIPPERS))

MODOS_VALIDOS = ('gemelo', 'can', 'pi3hat')
FUENTES_IMU_VALIDAS = ('sintetica', 'pi3hat_real')

# Cableado de los motores a los buses CAN del pi3hat: DOS MOTORES POR BUS,
# emparejados POR ESQUINA del robot. La oruga y el flipper de una misma esquina
# comparten bus porque ahí es donde cae el empalme del arnés, y así los cuatro
# ramales salen idénticos y cortos.
# TODO(hardware): confirmar contra el cableado físico antes de energizar.
MAPA_BUSES_POR_DEFECTO = {1: 1, 5: 1,   # esquina delantera izquierda
                          2: 2, 6: 2,   # esquina delantera derecha
                          3: 3, 7: 3,   # esquina trasera izquierda
                          4: 4, 8: 4}   # esquina trasera derecha

# Ciclos consecutivos sin respuesta de un motor antes de declarar fallo de
# comunicación y forzar el paro. A 100 Hz, 10 ciclos = 0.1 s.
CICLOS_WATCHDOG = 10

# --- Modelo de actitud de la IMU sintética (gemelo/can) ---
# La IMU real vive en el pi3hat; en 'gemelo'/'can' se sintetiza a partir del
# estado de los motores para que roll/pitch/yaw reflejen de verdad el movimiento.
#
# OJO: radio y ancho de orugas son GEOMETRÍA, no constantes del modelo: su fuente
# de verdad es config/geometria_robot.yaml y `flipper_node` los inyecta por el
# constructor. Los valores de abajo son sólo el respaldo para usar la clase suelta
# (tests, scripts) sin ROS.
RADIO_ORUGA_M = 0.05        # radio efectivo de la oruga (coincide con la odometría)
ANCHO_ORUGAS_M = 0.30       # separación entre orugas izq/der (idem odometría)
GANANCIA_PITCH = 0.6        # rad de cabeceo por rad de asimetría flippers frente-atrás
GANANCIA_ROLL = 0.6         # rad de alabeo  por rad de asimetría flippers izq-der
INCLINACION_MAX = 0.6       # saturación de roll/pitch (~34°)
GRAVEDAD = 9.81             # m/s^2, para proyectar la gravedad en el acelerómetro


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RMD_Hardware:
    """Interfaz única al hardware de movimiento (real o simulado).

    Parameters
    ----------
    modo_simulacion : bool
        Compatibilidad con el API anterior: True -> 'gemelo', False -> 'pi3hat'.
        Se ignora si se pasa `modo` explícito.
    modo : str | None
        'gemelo' (default), 'can' (SocketCAN, p.ej. vcan0) o 'pi3hat' (TODO).
    canal_can : str
        Interfaz SocketCAN para modo='can' (default 'vcan0'; con hardware: 'can0').
    radio_oruga : float
        Radio efectivo de la rueda motriz (m). Fuente: config/geometria_robot.yaml.
    ancho_orugas : float
        Separación entre orugas izquierda y derecha (m). Misma fuente.
    imu_fuente : str
        'sintetica' (default) o 'pi3hat_real'. Independiente de `modo`.
    imu_montaje_rpy : tuple[float, float, float]
        Rotación de la placa pi3hat respecto a base_link (rad). Sólo aplica a
        imu_fuente='pi3hat_real'.
    mapa_buses : dict[int, int] | None
        {id_motor: bus del pi3hat} para modo='pi3hat'. None -> MAPA_BUSES_POR_DEFECTO.
    motores_presentes : list[int] | None
        IDs realmente cableados. None -> los 8. Para probar en el banco con uno
        o dos motores sueltos sin que el watchdog declare fallo por los ausentes.
    ciclos_watchdog : int
        Ciclos consecutivos sin respuesta de un motor antes de declarar fallo de
        comunicación y forzar el paro.
    transporte : objeto con intercambiar()/cerrar() | None
        Inyección de dependencia para modo='pi3hat'. None -> se abre la placa de
        verdad. Los tests le pasan un transporte en memoria para ejercitar el
        protocolo completo sin hardware.
    """

    def __init__(self, modo_simulacion=True, modo=None, canal_can='vcan0',
                 radio_oruga=RADIO_ORUGA_M, ancho_orugas=ANCHO_ORUGAS_M,
                 imu_fuente='sintetica', imu_montaje_rpy=(0.0, 0.0, 0.0),
                 mapa_buses=None, ciclos_watchdog=CICLOS_WATCHDOG,
                 motores_presentes=None, transporte=None):
        if modo is None:
            modo = 'gemelo' if modo_simulacion else 'pi3hat'
        if modo not in MODOS_VALIDOS:
            raise ValueError(f"modo '{modo}' inválido; usar uno de {MODOS_VALIDOS}")
        if imu_fuente not in FUENTES_IMU_VALIDAS:
            raise ValueError(
                f"imu_fuente '{imu_fuente}' inválida; usar una de {FUENTES_IMU_VALIDAS}")
        self.modo = modo
        self.imu_fuente = imu_fuente
        self.modo_simulacion = (modo != 'pi3hat')  # compat con código existente
        self.canal_can = canal_can
        self.mapa_buses = dict(mapa_buses or MAPA_BUSES_POR_DEFECTO)
        self.ciclos_watchdog = int(ciclos_watchdog)
        self.radio_oruga = float(radio_oruga)
        self.ancho_orugas = float(ancho_orugas)
        self.ids_orugas = list(IDS_ORUGAS)
        self.ids_flippers = list(IDS_FLIPPERS)

        # BANCO DE PRUEBAS: motores realmente cableados. El resto sigue existiendo
        # en /joint_states (el URDF necesita las 8 juntas) pero NO se le habla ni
        # se le exige respuesta. Sin esto, probar con un motor suelto es imposible:
        # los 7 ausentes nunca contestan, el watchdog declara fallo de comunicación
        # y fuerza TODOS los comandos a cero, incluido el motor que sí está.
        todos = self.ids_orugas + self.ids_flippers
        if motores_presentes:
            self.ids_presentes = [m for m in todos if m in set(motores_presentes)]
            if not self.ids_presentes:
                raise ValueError(
                    f'motores_presentes={motores_presentes} no contiene ningún ID '
                    f'válido (los IDs son {todos})')
        else:
            self.ids_presentes = list(todos)

        # Estado interno del gemelo digital: posición/velocidad por motor.
        self._pos = {mid: 0.0 for mid in self.ids_orugas + self.ids_flippers}
        self._vel = {mid: 0.0 for mid in self.ids_orugas + self.ids_flippers}
        self._t_prev = time.monotonic()

        # Actitud sintética de la IMU (roll/pitch/yaw, rad). Se integra en
        # _actualizar_actitud a partir del estado de los motores, de modo que
        # sea coherente con la odometría y responda a orugas y flippers.
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._t_imu = time.monotonic()

        # Modo 'can': último estado conocido por motor (respaldo ante timeouts).
        self._bus = None
        self._ultimo_estado = {
            # temperatura_c se queda en 0: ODrive no la publica por CAN (sólo por
            # USB). estado_eje/error_eje los rellena el heartbeat del motor.
            mid: {'posicion_rad': 0.0, 'velocidad_rad_s': 0.0,
                  'torque_nm': 0.0, 'temperatura_c': 0.0,
                  'estado_eje': 0, 'error_eje': 0}
            for mid in self.ids_orugas + self.ids_flippers
        }
        self.timeouts_can = 0          # contador acumulado de respuestas perdidas
        self._t_ultimo_aviso = 0.0

        # Watchdog de seguridad: ciclos consecutivos sin respuesta, por motor.
        self._sin_respuesta = {mid: 0 for mid in self.ids_orugas + self.ids_flippers}
        self.fallo_comunicacion = False

        # Transporte del pi3hat (sólo modo='pi3hat'; en 'can' se usa el socket).
        self._transporte = None
        # Lector de la IMU física (None si imu_fuente='sintetica').
        self._imu_real = None

        if self.modo == 'gemelo':
            print('[HARDWARE] Motores en MODO GEMELO DIGITAL (sin bus)')
        elif self.modo == 'can':
            print(f'[HARDWARE] Motores vía SocketCAN "{canal_can}" (tramas reales)')
            self._init_can()
        elif transporte is not None:
            self._transporte = transporte   # inyectado (tests)
        else:
            print('[HARDWARE] Motores REALES por los buses CAN del pi3hat')
            self._init_hardware_real()

        if len(self.ids_presentes) != len(self.ids_orugas + self.ids_flippers):
            ausentes = [m for m in self.ids_orugas + self.ids_flippers
                        if m not in self.ids_presentes]
            print(f'[HARDWARE] MODO BANCO: sólo se habla con '
                  f'{", ".join(f"{m}({ID_A_NOMBRE[m]})" for m in self.ids_presentes)}. '
                  f'Ausentes (ignorados por el watchdog): '
                  f'{", ".join(f"{m}({ID_A_NOMBRE[m]})" for m in ausentes)}.')

        # Con ODrive no basta con abrir el bus: hay que armar los ejes o los
        # comandos se ignoran en silencio.
        if self.modo in ('can', 'pi3hat'):
            self._armar_motores()

        if self.imu_fuente == 'pi3hat_real':
            self._init_imu_real(imu_montaje_rpy)
        else:
            print('[HARDWARE] IMU SINTÉTICA (modelada desde el estado de los motores)')

    # ------------------------------------------------------------------ #
    # Backend SocketCAN (vcan0 con emulador, o can0 con adaptador físico)
    # ------------------------------------------------------------------ #
    def _init_can(self):
        try:
            import can
        except ImportError as e:
            raise RuntimeError(
                'modo="can" requiere python3-can (sudo apt install python3-can)'
            ) from e
        try:
            self._bus = can.interface.Bus(channel=self.canal_can, interface='socketcan')
        except OSError as e:
            raise RuntimeError(
                f'No se pudo abrir la interfaz CAN "{self.canal_can}": {e}. '
                f'¿Está creada? (scripts/setup_vcan.sh)'
            ) from e
        self._can = can
        # Drenar tramas viejas que hayan quedado en el buffer del socket.
        while self._bus.recv(timeout=0.0) is not None:
            pass

    def _intercambiar(self, peticiones, timeout=0.004):
        """Manda un LOTE de mensajes y devuelve lo que llegó, por motor.

        Parameters
        ----------
        peticiones : list[tuple[int, int, bytes]]
            [(node_id, cmd_id, datos), ...]

        Returns
        -------
        dict[int, dict[int, bytes]]
            {node_id: {cmd_id: datos}}. Diccionario vacío = ese motor no dijo nada.

        Con ODrive esto NO es pregunta-respuesta: el motor emite posición y
        velocidad cada 10 ms por su cuenta (ver protocolo_can). Lo que se recoge
        aquí es "lo que haya llegado desde el ciclo anterior", que a 100 Hz es
        justo una trama de encoder por motor.

        Es el único punto donde 'can' y 'pi3hat' difieren: SocketCAN manda por el
        socket y drena el buffer; el pi3hat lo hace todo en UN ciclo SPI.
        """
        if self.modo == 'pi3hat':
            return self._transporte.intercambiar(peticiones, timeout=timeout)
        return self._intercambiar_socketcan(peticiones, timeout=timeout)

    def _intercambiar_socketcan(self, peticiones, timeout=0.004):
        """Versión SocketCAN: enviar todo y luego escuchar el bus `timeout` segundos."""
        for node_id, cmd_id, datos in peticiones:
            self._bus.send(self._can.Message(
                arbitration_id=proto.id_arbitraje(node_id, cmd_id),
                data=datos,
                is_extended_id=False,
            ))

        recibido = {node_id: {} for node_id, _, _ in peticiones}
        limite = time.monotonic() + timeout
        while True:
            restante = limite - time.monotonic()
            if restante <= 0:
                break
            msg = self._bus.recv(timeout=restante)
            if msg is None:
                continue
            nodo = proto.nodo_de_id(msg.arbitration_id)
            if nodo is None:
                continue
            recibido.setdefault(nodo, {})[proto.cmd_de_id(msg.arbitration_id)] = \
                bytes(msg.data)

        self.timeouts_can += sum(1 for m in recibido.values() if not m)
        return recibido

    def _armar_motores(self):
        """Deja cada motor sin errores, en su modo de control y en LAZO CERRADO.

        Es OBLIGATORIO con ODrive y no lo era con el protocolo anterior: un eje en
        IDLE acepta los comandos de velocidad/posición y NO HACE NADA, sin
        devolver ningún error. Sin este paso el robot parece muerto aunque el bus
        esté perfecto.

        Orugas en control de VELOCIDAD (passthrough) y flippers en POSICIÓN (con
        filtro de entrada, que suaviza el salto cuando llega una consigna nueva).
        """
        if self.modo not in ('can', 'pi3hat'):
            return

        orugas = set(self.ids_orugas)
        peticiones = []
        for mid in self.ids_presentes:
            if mid in orugas:
                control, entrada = proto.CONTROL_VELOCIDAD, proto.ENTRADA_PASSTHROUGH
            else:
                control, entrada = proto.CONTROL_POSICION, proto.ENTRADA_POS_FILTER
            peticiones += [
                (mid, proto.CMD_CLEAR_ERRORS, proto.trama_clear_errors()),
                (mid, proto.CMD_SET_CONTROLLER_MODE,
                 proto.trama_set_controller_mode(control, entrada)),
                (mid, proto.CMD_SET_AXIS_STATE,
                 proto.trama_set_axis_state(proto.ESTADO_CLOSED_LOOP)),
            ]
        try:
            self._intercambiar(peticiones, timeout=0.005)
            print(f'[HARDWARE] {len(self.ids_presentes)} motor(es) armados '
                  f'en lazo cerrado.')
        except Exception as e:
            print(f'[HARDWARE] No se pudieron armar los motores: {e}')

    def _estado_via_bus(self, comandos_orugas, comandos_flippers):
        """Ciclo de comunicación con los motores (común a 'can' y 'pi3hat')."""
        estado = {}

        # Si el watchdog detectó fallo de comunicación, no se sigue mandando
        # velocidad a ciegas: las orugas van a cero y los flippers se quedan
        # donde están hasta que el bus vuelva.
        if self.fallo_comunicacion:
            comandos_orugas = {mid: 0.0 for mid in self.ids_orugas}
            comandos_flippers = {
                mid: self._ultimo_estado[mid]['posicion_rad'] for mid in self.ids_flippers}

        # Comandar. Sólo a los presentes: en modo banco los ausentes ni siquiera
        # se consultan. Ya no hay una segunda ronda para leer la posición: la
        # trama de encoder de ODrive es multivuelta y llega sola.
        presentes = set(self.ids_presentes)
        peticiones = [
            (mid, proto.CMD_SET_INPUT_VEL,
             proto.trama_set_input_vel(float(comandos_orugas.get(mid, 0.0))))
            for mid in self.ids_orugas if mid in presentes
        ]
        peticiones += [
            (mid, proto.CMD_SET_INPUT_POS,
             proto.trama_set_input_pos(float(comandos_flippers.get(
                 mid, self._ultimo_estado[mid]['posicion_rad']))))
            for mid in self.ids_flippers if mid in presentes
        ]

        recibido = self._intercambiar(peticiones)
        for mid, mensajes in recibido.items():
            self._aplicar_mensajes(mid, mensajes)

        for mid in self.ids_orugas + self.ids_flippers:
            estado[mid] = dict(self._ultimo_estado[mid])

        self._vigilar_comunicacion(recibido)
        self._avisar_timeouts()
        return estado

    def _vigilar_comunicacion(self, recibido):
        """Watchdog de seguridad: detecta motores que dejaron de emitir.

        Un motor mudo con una consigna de velocidad ya aceptada seguiría girando:
        por eso, al superar `ciclos_watchdog` ciclos sin oír nada de él se para
        todo y se entra en modo fallo (comandos forzados a cero) hasta que el bus
        se recupere.

        Con ODrive "responder" es emitir: el motor manda su trama de encoder cada
        10 ms sin que nadie se la pida, así que un silencio de varios ciclos
        significa de verdad que ese motor no está.
        """
        mudos = []
        for mid in self.ids_presentes:
            if not recibido.get(mid):
                self._sin_respuesta[mid] += 1
                if self._sin_respuesta[mid] >= self.ciclos_watchdog:
                    mudos.append(mid)
            else:
                self._sin_respuesta[mid] = 0

        if mudos and not self.fallo_comunicacion:
            self.fallo_comunicacion = True
            nombres = ', '.join(f'{m}({ID_A_NOMBRE[m]})' for m in mudos)
            print(f'[HARDWARE] ¡FALLO DE COMUNICACIÓN! Sin señal de: {nombres}. '
                  f'Paro de emergencia; comandos forzados a cero.')
            self.parar_motores()
        elif not mudos and self.fallo_comunicacion:
            self.fallo_comunicacion = False
            print(f'[HARDWARE] Comunicación restablecida con '
                  f'{len(self.ids_presentes)} motor(es).')
            # El paro los dejó en IDLE: sin rearmar, ignorarían los comandos.
            self._armar_motores()

    def parar_motores(self):
        """Paro de emergencia: consigna cero y ejes a IDLE (best effort).

        IDLE deja el driver energizado pero sin par, que es el equivalente al
        'paro' del protocolo anterior. Para volver a mover hay que rearmar
        (lo hace `_armar_motores`, que llama el watchdog al recuperarse).
        """
        orugas = set(self.ids_orugas)
        try:
            peticiones = [
                (mid, proto.CMD_SET_INPUT_VEL, proto.trama_set_input_vel(0.0))
                for mid in self.ids_presentes if mid in orugas
            ]
            peticiones += [
                (mid, proto.CMD_SET_AXIS_STATE,
                 proto.trama_set_axis_state(proto.ESTADO_IDLE))
                for mid in self.ids_presentes
            ]
            self._intercambiar(peticiones, timeout=0.002)
        except Exception as e:
            print(f'[HARDWARE] No se pudo enviar el paro: {e}')

    def _aplicar_mensajes(self, id_motor, mensajes):
        """Vuelca en el último estado conocido lo que haya emitido un motor."""
        u = self._ultimo_estado.get(id_motor)
        if not u or not mensajes:
            return

        datos = mensajes.get(proto.CMD_GET_ENCODER_ESTIMATES)
        if datos is not None:
            enc = proto.parsear_encoder(datos)
            if enc is not None:
                u['posicion_rad'] = enc['posicion_rad']
                u['velocidad_rad_s'] = enc['velocidad_rad_s']

        datos = mensajes.get(proto.CMD_GET_IQ)
        if datos is not None:
            iq = proto.parsear_iq(datos)
            if iq is not None:
                u['torque_nm'] = iq['iq_medido_a'] * proto.KT_NM_POR_A

        datos = mensajes.get(proto.CMD_HEARTBEAT)
        if datos is not None:
            hb = proto.parsear_heartbeat(datos)
            if hb is not None:
                u['estado_eje'] = hb['estado_eje']
                u['error_eje'] = hb['error_eje']

    def _avisar_timeouts(self):
        """Advierte (máx. 1 vez/s) si hay respuestas perdidas en el bus."""
        if self.modo == 'pi3hat' and self._transporte is not None:
            # En pi3hat el contador lo lleva el transporte (un ciclo SPI puede
            # perder varias respuestas a la vez).
            self.timeouts_can = self._transporte.timeouts
            donde = 'los buses del pi3hat'
            pista = '(¿motores energizados? ¿mapa_buses correcto?)'
        else:
            donde = f'"{self.canal_can}"'
            pista = '(¿está corriendo motor_emulator?)'

        if self.timeouts_can and time.monotonic() - self._t_ultimo_aviso > 1.0:
            self._t_ultimo_aviso = time.monotonic()
            print(f'[HARDWARE] AVISO: {self.timeouts_can} timeouts CAN acumulados '
                  f'en {donde} {pista}')

    # ------------------------------------------------------------------ #
    # Hardware real: motores por los buses CAN del pi3hat
    # ------------------------------------------------------------------ #
    def _init_hardware_real(self):
        """Abre el Pi3HatRouter y deja listo el transporte CAN de los motores.

        El mapa de buses sale de `mapa_buses` (config/geometria_robot.yaml o
        parámetro del nodo), NO está clavado en el código: si cambia el cableado,
        se cambia el YAML.
        """
        from ugv_bridge.pi3hat_backend import TransportePi3Hat

        self._transporte = TransportePi3Hat(self.mapa_buses)
        por_bus = {}
        for mid, bus in sorted(self.mapa_buses.items()):
            por_bus.setdefault(bus, []).append(f'{mid}({ID_A_NOMBRE.get(mid, "?")})')
        for bus, motores in sorted(por_bus.items()):
            print(f'[HARDWARE]   bus CAN {bus}: {", ".join(motores)}')

    def _init_imu_real(self, montaje_rpy):
        """Abre la IMU física de la placa (comparte router con los motores)."""
        from ugv_bridge.pi3hat_backend import LectorImuPi3Hat

        # Si los motores también van por el pi3hat, se comparte la MISMA placa:
        # se le pasa el mapa de buses para que el router se abra una sola vez.
        mapa = self.mapa_buses if self.modo == 'pi3hat' else None
        self._imu_real = LectorImuPi3Hat(montaje_rpy=montaje_rpy, mapa_buses=mapa)
        grados = tuple(round(math.degrees(a), 1) for a in montaje_rpy)
        print(f'[HARDWARE] IMU REAL del pi3hat (montaje rpy={grados} grados)')

    # ------------------------------------------------------------------ #
    # API que consume el nodo de ROS
    # ------------------------------------------------------------------ #
    def enviar_y_leer_estado(self, comandos_orugas, comandos_flippers):
        """Función MAESTRA (llamada ~100 Hz): comanda los 8 motores y devuelve su estado.

        Parameters
        ----------
        comandos_orugas : dict[int, float]
            {id_oruga: velocidad_rad_s} para IDs 1-4.
        comandos_flippers : dict[int, float]
            {id_flipper: posicion_rad} para IDs 5-8.

        Returns
        -------
        dict[int, dict]
            {id: {posicion_rad, velocidad_rad_s, torque_nm, temperatura_c}}
        """
        if self.modo in ('can', 'pi3hat'):
            estado = self._estado_via_bus(comandos_orugas, comandos_flippers)
        else:
            estado = self._simular_estado(comandos_orugas, comandos_flippers)
        # Con el estado de los 8 motores ya resuelto, integramos la actitud del
        # chasis. Sólo hace falta si la IMU es sintética: con la IMU real la
        # actitud la mide la placa.
        if self.imu_fuente == 'sintetica':
            self._actualizar_actitud(estado)
        return estado

    def _actualizar_actitud(self, estado):
        """Sintetiza la actitud del chasis (roll/pitch/yaw) desde el estado de los motores.

        - yaw: integra el giro skid-steer de las orugas (mismo modelo que la
          odometría), así el rumbo de la IMU es COHERENTE con /odom en vez de
          quedarse clavado en 0 y pelear con el EKF.
        - roll/pitch: modelo cinemático demostrativo — la asimetría de los
          flippers (frente vs. atrás -> cabeceo; izq vs. der -> alabeo) inclina
          el chasis, y este se asienta hacia esa actitud con un primer orden.
          En hardware real la actitud la mide la IMU del pi3hat; aquí deja que el
          pipeline (EKF/RViz/GCS) vea pitch/roll/yaw que responden al movimiento.
        """
        ahora = time.monotonic()
        dt = min(ahora - self._t_imu, 0.1)
        self._t_imu = ahora
        if dt <= 0.0:
            return

        # yaw: velocidad angular skid-steer de las orugas -> integrar el rumbo.
        # IDs: izq = fl(1), rl(3);  der = fr(2), rr(4).
        v_izq = self.radio_oruga * (
            estado[1]['velocidad_rad_s'] + estado[3]['velocidad_rad_s']) / 2.0
        v_der = self.radio_oruga * (
            estado[2]['velocidad_rad_s'] + estado[4]['velocidad_rad_s']) / 2.0
        self._yaw += (v_der - v_izq) / self.ancho_orugas * dt

        # roll/pitch: asimetría de los flippers -> actitud objetivo saturada.
        # IDs: frente = fl(5), fr(6);  atrás = rl(7), rr(8);
        #      izq = fl(5), rl(7);     der = fr(6), rr(8).
        frente = (estado[5]['posicion_rad'] + estado[6]['posicion_rad']) / 2.0
        atras = (estado[7]['posicion_rad'] + estado[8]['posicion_rad']) / 2.0
        izq = (estado[5]['posicion_rad'] + estado[7]['posicion_rad']) / 2.0
        der = (estado[6]['posicion_rad'] + estado[8]['posicion_rad']) / 2.0

        # Signos según REP-103 (X adelante, Y IZQUIERDA, Z arriba), que NO es la
        # convención aeronáutica: como Y apunta a la izquierda, un pitch POSITIVO
        # es morro ABAJO, y un roll POSITIVO baja el costado DERECHO.
        # Un flipper con ángulo positivo gira su punta hacia abajo (rotación sobre
        # +Y), o sea empuja contra el suelo y LEVANTA ese extremo del chasis:
        #   flippers delanteros arriba -> morro arriba  -> pitch NEGATIVO
        #   flippers derechos  arriba -> costado der. arriba -> roll NEGATIVO
        pitch_obj = _clamp(GANANCIA_PITCH * (atras - frente), -INCLINACION_MAX, INCLINACION_MAX)
        roll_obj = _clamp(GANANCIA_ROLL * (izq - der), -INCLINACION_MAX, INCLINACION_MAX)

        # El chasis se asienta hacia la actitud objetivo (1er orden, ~0.2 s).
        alpha = min(dt * 5.0, 1.0)
        self._pitch += (pitch_obj - self._pitch) * alpha
        self._roll += (roll_obj - self._roll) * alpha

    def _simular_estado(self, comandos_orugas, comandos_flippers):
        # dt real entre llamadas -> el modelo respeta la frecuencia del bucle.
        ahora = time.monotonic()
        dt = min(ahora - self._t_prev, 0.1)  # cap para evitar saltos tras pausas
        self._t_prev = ahora

        estado = {}

        # Orugas: comando de VELOCIDAD -> integramos posición.
        for mid in self.ids_orugas:
            v_cmd = float(comandos_orugas.get(mid, 0.0))
            self._vel[mid] = v_cmd + random.uniform(-0.02, 0.02)  # ruido de encoder
            self._pos[mid] += self._vel[mid] * dt
            estado[mid] = {
                'posicion_rad': self._pos[mid],
                'velocidad_rad_s': self._vel[mid],
                'torque_nm': abs(v_cmd) * 0.3 + random.uniform(0.0, 0.2),
                'temperatura_c': 35.0,
            }

        # Flippers: comando de POSICIÓN -> nos acercamos al objetivo (1er orden).
        for mid in self.ids_flippers:
            p_obj = float(comandos_flippers.get(mid, self._pos[mid]))
            error = p_obj - self._pos[mid]
            paso = error * min(dt * 8.0, 1.0)  # constante de tiempo ~0.12 s
            self._pos[mid] += paso
            self._vel[mid] = (paso / dt) if dt > 0 else 0.0
            estado[mid] = {
                'posicion_rad': self._pos[mid],
                'velocidad_rad_s': self._vel[mid],
                'torque_nm': abs(error) * 2.0 + random.uniform(0.0, 0.3),
                'temperatura_c': 35.0,
            }

        return estado

    def leer_imu(self):
        """Lee la IMU. Devuelve dict con actitud + aceleración.

        Ángulos en RADIANES. Con la IMU real se incluye además 'quat' (x,y,z,w),
        que es lo que el nodo publica directamente: evita el rodeo
        cuaternión -> Euler -> cuaternión, que pierde información cerca de
        pitch = ±90° (el robot volcado sobre un flanco es un caso real aquí).

        La fuente la decide `imu_fuente`, NO el modo de motor: con el pi3hat ya
        montado se puede usar la IMU física mientras los motores siguen emulados.

        Con IMU sintética la actitud no es ruido: se integra en
        `_actualizar_actitud` (yaw de las orugas, pitch/roll de los flippers), así
        refleja el movimiento real. Aquí sólo se le suma un pequeño ruido de
        sensor y se proyecta la gravedad en el acelerómetro según la actitud.
        """
        if self.imu_fuente == 'pi3hat_real':
            return self._imu_real.leer()

        roll = self._roll + random.uniform(-0.003, 0.003)
        pitch = self._pitch + random.uniform(-0.003, 0.003)
        yaw = self._yaw + random.uniform(-0.003, 0.003)

        # Acelerómetro en reposo: mide FUERZA ESPECÍFICA (la reacción que lo
        # sostiene), no el vector gravedad. Nivelado da (0, 0, +9.81), no
        # (0, 0, -9.81). Proyectada al cuerpo (REP-103: X adelante, Y izquierda,
        # Z arriba) queda:
        #     f = ( -g*sin(pitch), +g*sin(roll)*cos(pitch), +g*cos(roll)*cos(pitch) )
        # Los signos de X e Y estaban invertidos: con pitch positivo (morro abajo)
        # el modelo entregaba accel_x positiva, cuando un acelerómetro real
        # entrega negativa. La Z sí estaba bien, y por eso en reposo nivelado no
        # se notaba. Corregido para que la IMU sintética y la real coincidan en
        # convención el día que se cambie imu_fuente.
        accel_x = -GRAVEDAD * math.sin(pitch)
        accel_y = GRAVEDAD * math.sin(roll) * math.cos(pitch)
        accel_z = GRAVEDAD * math.cos(roll) * math.cos(pitch)
        return {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'accel_x': accel_x + random.uniform(-0.02, 0.02),
            'accel_y': accel_y + random.uniform(-0.02, 0.02),
            'accel_z': accel_z + random.uniform(-0.02, 0.02),
        }

    def cerrar(self):
        """Libera el hardware. Antes de soltar el bus, DETIENE los motores."""
        if self.modo in ('can', 'pi3hat'):
            self.parar_motores()

        if self.modo == 'can' and self._bus is not None:
            self._bus.shutdown()
            self._bus = None
        elif self.modo == 'pi3hat' and self._transporte is not None:
            self._transporte.cerrar()
            self._transporte = None

        if self._imu_real is not None:
            self._imu_real.cerrar()
            self._imu_real = None
