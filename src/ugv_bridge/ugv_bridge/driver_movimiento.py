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

# Cableado por defecto de los motores a los buses CAN del pi3hat.
# TODO(hardware): confirmar contra el cableado físico antes de energizar.
MAPA_BUSES_POR_DEFECTO = {1: 1, 2: 1, 3: 1, 4: 1,   # orugas  -> bus 1
                          5: 2, 6: 2, 7: 2, 8: 2}   # flippers -> bus 2

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
                 transporte=None):
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
            mid: {'posicion_rad': 0.0, 'velocidad_rad_s': 0.0,
                  'torque_nm': 0.0, 'temperatura_c': 0.0}
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

    def _transferir(self, id_motor, datos, timeout=0.004):
        """Envía una trama al motor y espera SU respuesta (0x240+id).

        Devuelve los bytes de datos de la respuesta, o None si venció el timeout
        (motor/emulador ausente). Descartar respuestas ajenas es seguro porque
        el ciclo es estrictamente pregunta-respuesta.
        """
        self._bus.send(self._can.Message(
            arbitration_id=proto.id_comando(id_motor),
            data=datos,
            is_extended_id=False,
        ))
        limite = time.monotonic() + timeout
        while True:
            restante = limite - time.monotonic()
            if restante <= 0:
                self.timeouts_can += 1
                return None
            msg = self._bus.recv(timeout=restante)
            if msg is not None and msg.arbitration_id == proto.id_respuesta(id_motor):
                return msg.data

    def _intercambiar(self, peticiones, timeout=0.004):
        """Envía un LOTE de tramas y devuelve {id_motor: datos_respuesta | None}.

        Es el único punto donde 'can' y 'pi3hat' difieren:
          - SocketCAN: pregunta-respuesta motor por motor sobre el socket.
          - pi3hat:    las 8 tramas viajan en UN ciclo SPI (para eso existe la placa).
        El protocolo que va dentro de `datos` es el mismo en ambos casos.
        """
        if self.modo == 'pi3hat':
            return self._transporte.intercambiar(peticiones, timeout=timeout)
        return {mid: self._transferir(mid, datos, timeout=timeout)
                for mid, datos in peticiones}

    def _estado_via_bus(self, comandos_orugas, comandos_flippers):
        """Ciclo de comunicación con los 8 motores (común a 'can' y 'pi3hat')."""
        estado = {}

        # Si el watchdog detectó fallo de comunicación, no se sigue mandando
        # velocidad a ciegas: las orugas van a cero y los flippers se quedan
        # donde están hasta que el bus vuelva.
        if self.fallo_comunicacion:
            comandos_orugas = {mid: 0.0 for mid in self.ids_orugas}
            comandos_flippers = {
                mid: self._ultimo_estado[mid]['posicion_rad'] for mid in self.ids_flippers}

        # 1. Comandar cada motor y leer su trama de estado (temp/torque/vel).
        peticiones = [
            (mid, proto.trama_cmd_velocidad(float(comandos_orugas.get(mid, 0.0))))
            for mid in self.ids_orugas
        ]
        peticiones += [
            (mid, proto.trama_cmd_posicion(float(comandos_flippers.get(
                mid, self._ultimo_estado[mid]['posicion_rad']))))
            for mid in self.ids_flippers
        ]
        for mid, datos in self._intercambiar(peticiones).items():
            self._aplicar_respuesta_estado(mid, datos)

        # 2. Leer posición multivuelta (la de 1 vuelta de la respuesta anterior
        #    no sirve para odometría: se enrolla cada 360°).
        todos = self.ids_orugas + self.ids_flippers
        respuestas = self._intercambiar(
            [(mid, proto.trama_leer_multivuelta()) for mid in todos])
        for mid in todos:
            datos = respuestas.get(mid)
            resp = proto.parsear_respuesta(datos) if datos is not None else None
            if resp is not None and 'posicion_rad' in resp:
                self._ultimo_estado[mid]['posicion_rad'] = resp['posicion_rad']
            estado[mid] = dict(self._ultimo_estado[mid])

        self._vigilar_comunicacion(respuestas)
        self._avisar_timeouts()
        return estado

    def _vigilar_comunicacion(self, respuestas):
        """Watchdog de seguridad: detecta motores que dejaron de responder.

        Un motor mudo con un comando de velocidad ya aceptado seguiría girando:
        por eso, al superar `ciclos_watchdog` ciclos sin respuesta se envía un
        paro a TODOS los motores y se entra en modo fallo (comandos forzados a
        cero) hasta que el bus se recupere.
        """
        mudos = []
        for mid in self.ids_orugas + self.ids_flippers:
            if respuestas.get(mid) is None:
                self._sin_respuesta[mid] += 1
                if self._sin_respuesta[mid] >= self.ciclos_watchdog:
                    mudos.append(mid)
            else:
                self._sin_respuesta[mid] = 0

        if mudos and not self.fallo_comunicacion:
            self.fallo_comunicacion = True
            nombres = ', '.join(f'{m}({ID_A_NOMBRE[m]})' for m in mudos)
            print(f'[HARDWARE] ¡FALLO DE COMUNICACIÓN! Sin respuesta de: {nombres}. '
                  f'Paro de emergencia; comandos forzados a cero.')
            self.parar_motores()
        elif not mudos and self.fallo_comunicacion:
            self.fallo_comunicacion = False
            print('[HARDWARE] Comunicación restablecida con los 8 motores.')

    def parar_motores(self):
        """Manda la trama de paro a los 8 motores (best effort, sin excepciones)."""
        try:
            self._intercambiar(
                [(mid, proto.trama_paro()) for mid in self.ids_orugas + self.ids_flippers],
                timeout=0.002,
            )
        except Exception as e:
            print(f'[HARDWARE] No se pudo enviar el paro: {e}')

    def _aplicar_respuesta_estado(self, id_motor, datos):
        if datos is None:
            return
        resp = proto.parsear_respuesta(datos)
        if resp is None or 'velocidad_rad_s' not in resp:
            return
        u = self._ultimo_estado[id_motor]
        u['velocidad_rad_s'] = resp['velocidad_rad_s']
        u['torque_nm'] = resp['torque_nm']
        u['temperatura_c'] = resp['temperatura_c']

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
        v_izq = self.radio_oruga * (estado[1]['velocidad_rad_s'] + estado[3]['velocidad_rad_s']) / 2.0
        v_der = self.radio_oruga * (estado[2]['velocidad_rad_s'] + estado[4]['velocidad_rad_s']) / 2.0
        self._yaw += (v_der - v_izq) / self.ancho_orugas * dt

        # roll/pitch: asimetría de los flippers -> actitud objetivo saturada.
        # IDs: frente = fl(5), fr(6);  atrás = rl(7), rr(8);
        #      izq = fl(5), rl(7);     der = fr(6), rr(8).
        frente = (estado[5]['posicion_rad'] + estado[6]['posicion_rad']) / 2.0
        atras = (estado[7]['posicion_rad'] + estado[8]['posicion_rad']) / 2.0
        izq = (estado[5]['posicion_rad'] + estado[7]['posicion_rad']) / 2.0
        der = (estado[6]['posicion_rad'] + estado[8]['posicion_rad']) / 2.0
        pitch_obj = _clamp(GANANCIA_PITCH * (frente - atras), -INCLINACION_MAX, INCLINACION_MAX)
        roll_obj = _clamp(GANANCIA_ROLL * (der - izq), -INCLINACION_MAX, INCLINACION_MAX)

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

        # Acelerómetro en reposo = gravedad proyectada al cuerpo (x adelante,
        # y izquierda, z arriba, REP-103). Coherente con la actitud estimada.
        accel_x = GRAVEDAD * math.sin(pitch)
        accel_y = -GRAVEDAD * math.sin(roll) * math.cos(pitch)
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
