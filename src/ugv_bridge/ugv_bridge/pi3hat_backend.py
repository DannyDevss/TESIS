#!/usr/bin/env python3
"""pi3hat_backend.py — Acceso al hardware mjbots pi3hat: buses CAN + IMU.

Este módulo concentra TODO lo que depende de `moteus_pi3hat`, para que el resto
del código (driver_movimiento, flipper_node, pi3hat_imu_node) no sepa de SPI ni
de asyncio. Ofrece dos piezas independientes:

    TransportePi3Hat  -> envía/recibe tramas CAN de los motores (protocolo RMD,
                         el mismo de protocolo_can.py que valida motor_emulator).
    LectorImuPi3Hat   -> lee la actitud (y la aceleración, si el firmware la
                         expone) de la IMU integrada en la placa.

Las dos pueden compartir un MISMO `Pi3HatRouter`: la placa se abre una sola vez
por proceso. Si se intenta abrir dos veces, el segundo intento falla (el
descriptor SPI está tomado). Por eso `abrir_router()` cachea la instancia.

---------------------------------------------------------------------------
POR QUÉ TRAMAS CRUDAS Y NO moteus.Controller
---------------------------------------------------------------------------
Los motores son SteadyWin (GIM6010-8 / GDS68) y hablan protocolo estilo RMD V3
(IDs 0x140+id / 0x240+id, comandos 0xA2/0xA4/0x92...), NO el protocolo moteus.
`moteus.Controller.set_position()` produciría tramas que estos motores ignoran.

Aquí el pi3hat se usa sólo como TRANSPORTE CAN, y las tramas las arma
`protocolo_can.py` — exactamente las mismas que ya se validan contra
`motor_emulator` sobre vcan0. Un solo protocolo, probado de punta a punta.

---------------------------------------------------------------------------
CÓMO SE EMITE UN ID DE ARBITRAJE ARBITRARIO (verificado el 12/08/2026)
---------------------------------------------------------------------------
Contra la librería instalada en la Raspberry (moteus/transport.py), una Command
con `raw = True` se convierte en trama así:

    def _command_to_frame(self, command):
        if getattr(command, 'raw', False):
            return Frame(arbitration_id=command.arbitration_id, ...)
        arbitration_id = (destination | (0x8000 if reply_required else 0)
                          | (source << 8) | (can_prefix << 16))

Es decir: con `raw=True` la librería NO compone nada a partir de source y
destination, sino que emite `command.arbitration_id` TAL CUAL. Eso es justo lo
que necesita este proyecto, que manda tramas de un protocolo ajeno (RMD): se
asigna el ID completo y listo.

    cmd.arbitration_id = 0x141      # 0x140 + id_motor

OJO con la versión anterior de este archivo: repartía el ID en
source/destination y NO asignaba arbitration_id, que por defecto vale 0. El
resultado comprobado era que TODAS las tramas salían con ID 0x0 y ningún motor
habría respondido nunca. El sintoma habria sido "los motores no contestan", que
es facil de confundir con un problema de cableado o de IDs mal configurados.

Sigue pendiente de verificar con los motores en la mano que el ID que aparece en
el bus es el esperado (scripts/verificar_pi3hat.py --motores).

---------------------------------------------------------------------------
CAN-FD vs CAN 2.0 CLÁSICO (la otra causa de "ningún motor responde")
---------------------------------------------------------------------------
Por defecto moteus_pi3hat deja los buses configurados para motores moteus:
CAN-FD con bitrate switch, 1 Mbps de arbitraje y 5 Mbps de datos. Los SteadyWin
hablan CAN 2.0A clásico a 1 Mbps y descartan las tramas FD como error de forma,
así que no contestan NUNCA — dando el mismo síntoma que un cable suelto. Ver
`kwargs_can_clasico()`, que apaga FD/BRS antes de abrir la placa.
"""
import asyncio
import math

# La importación real se hace perezosa: este módulo se importa también en el PC
# de desarrollo (sin pi3hat), donde moteus_pi3hat no existe.
_moteus = None
_moteus_pi3hat = None
_router_cacheado = None
_loop_cacheado = None


def _importar():
    """Importa moteus/moteus_pi3hat con un mensaje claro si faltan."""
    global _moteus, _moteus_pi3hat
    if _moteus_pi3hat is not None:
        return
    try:
        import moteus
        import moteus_pi3hat
    except ImportError as e:
        raise RuntimeError(
            'Falta la librería del pi3hat. En la Raspberry:\n'
            '    pip3 install moteus moteus-pi3hat\n'
            f'(detalle: {e})'
        ) from e
    _moteus = moteus
    _moteus_pi3hat = moteus_pi3hat


def obtener_loop():
    """Event loop persistente para las llamadas async de moteus_pi3hat.

    Se crea UNO solo y se reutiliza: `asyncio.run()` por ciclo de control
    (100 Hz) construiría y destruiría un loop 100 veces por segundo.
    """
    global _loop_cacheado
    if _loop_cacheado is None:
        _loop_cacheado = asyncio.new_event_loop()
    return _loop_cacheado


# Velocidad de los buses CAN de los motores SteadyWin (CAN 2.0A clásico).
BITRATE_CAN = 1000000


def kwargs_can_clasico(buses):
    """Argumentos de Pi3HatRouter para hablar CAN 2.0 CLÁSICO a 1 Mbps.

    POR QUÉ HACE FALTA
    ------------------
    moteus_pi3hat configura los buses PARA MOTORES MOTEUS: tramas CAN-FD con
    bitrate switch (1 Mbps de arbitraje, 5 Mbps de datos). Los SteadyWin de esta
    tesis son CAN 2.0A clásico: una trama FD no la entienden, la descartan como
    error de forma y NUNCA contestan. El síntoma es exactamente "0 de 8 motores
    responden" aunque el cableado, los IDs y la alimentación estén bien.

    El nombre del parámetro cambia entre versiones de la librería, así que se
    inspecciona la firma en vez de suponerla, y se avisa por consola de qué se
    pudo aplicar.
    """
    import inspect

    params = inspect.signature(_moteus_pi3hat.Pi3HatRouter.__init__).parameters
    Config = getattr(_moteus_pi3hat, 'CanConfiguration', None)

    if 'can' in params and Config is not None:
        cfg = {}
        for bus in sorted(buses):
            c = Config()
            c.slow_bitrate = BITRATE_CAN
            c.fast_bitrate = BITRATE_CAN
            c.fdcan_frame = False          # CAN 2.0 clásico, no FD
            c.bitrate_switch = False       # sin fase de datos rápida
            c.automatic_retransmission = True
            cfg[int(bus)] = c
        print(f'[pi3hat] Buses {sorted(buses)} en CAN 2.0 clásico a '
              f'{BITRATE_CAN // 1000} kbps (FD y BRS apagados).')
        return {'can': cfg}

    if 'disable_brs' in params:
        print('[pi3hat] AVISO: esta versión de moteus_pi3hat no expone '
              'CanConfiguration. Sólo se puede apagar el bitrate switch; las '
              'tramas siguen siendo CAN-FD y los motores RMD podrían no '
              'responder. Actualizar la librería: pip3 install -U moteus-pi3hat')
        return {'disable_brs': True}

    print('[pi3hat] AVISO: no se pudo configurar el bus en CAN 2.0 clásico con '
          'esta versión de moteus_pi3hat; queda en CAN-FD a 5 Mbps y los '
          'motores SteadyWin no responderán.')
    return {}


def abrir_router(mapa_buses=None):
    """Abre (una sola vez por proceso) el Pi3HatRouter y lo devuelve.

    Parameters
    ----------
    mapa_buses : dict[int, int] | None
        {id_motor: numero_de_bus_del_pi3hat}. None -> sólo IMU, sin motores.

    La placa admite UN solo dueño del bus SPI, así que motores e IMU comparten
    esta instancia.
    """
    global _router_cacheado
    if _router_cacheado is not None:
        return _router_cacheado

    _importar()

    # moteus_pi3hat quiere el mapa al revés: {bus: [ids...]}.
    servo_bus_map = {}
    for id_motor, bus in (mapa_buses or {}).items():
        servo_bus_map.setdefault(int(bus), []).append(int(id_motor))

    extra = kwargs_can_clasico(servo_bus_map.keys()) if servo_bus_map else {}
    _router_cacheado = _moteus_pi3hat.Pi3HatRouter(
        servo_bus_map=servo_bus_map, **extra)
    return _router_cacheado


def cerrar_router():
    """Suelta la instancia cacheada (para tests o reinicio limpio)."""
    global _router_cacheado
    _router_cacheado = None


# ====================================================================== #
# TRANSPORTE CAN DE LOS MOTORES
# ====================================================================== #
# Suelo del techo de tiempo de un ciclo del bus. Una transacción SPI con los 8
# motores respondiendo es de milisegundos; 25 ms deja margen de sobra y aun así
# mantiene el bucle de control por encima de los 40 Hz en el peor caso.
TIMEOUT_CICLO_MIN_S = 0.025


class TransportePi3Hat:
    """Envía tramas CAN a los motores por el pi3hat y devuelve sus respuestas.

    Expone la MISMA interfaz `intercambiar()` que el transporte SocketCAN, de
    modo que `driver_movimiento` usa el mismo código de protocolo en modo 'can'
    (emulador) y en modo 'pi3hat' (hardware real).

    Parameters
    ----------
    mapa_buses : dict[int, int]
        {id_motor: numero_de_bus}. Ej. orugas 1-4 en el bus 1, flippers 5-8 en el 2.
    """

    def __init__(self, mapa_buses):
        _importar()
        self.mapa_buses = {int(k): int(v) for k, v in mapa_buses.items()}
        self.router = abrir_router(self.mapa_buses)
        self.loop = obtener_loop()
        self.timeouts = 0
        self.ciclos_vencidos = 0   # ciclos que se cortaron por el techo de tiempo
        # Máscara de buses a revisar por respuestas entrantes en cada ciclo.
        self._mascara_buses = 0
        for bus in set(self.mapa_buses.values()):
            self._mascara_buses |= (1 << bus)

    def intercambiar(self, peticiones, timeout=0.004):
        """Envía todas las tramas en UN ciclo del pi3hat y recoge las respuestas.

        Parameters
        ----------
        peticiones : list[tuple[int, bytes]]
            [(id_motor, datos_de_8_bytes), ...]
        timeout : float
            Techo de tiempo del ciclo. Se respeta con un suelo de
            TIMEOUT_CICLO_MIN_S, porque una transacción SPI legítima con los 8
            motores tarda más que el timeout que usa el transporte SocketCAN.

            NO es decorativo: comprobado en la placa (12/08/2026), si se piden
            respuestas con `force_can_check` y NINGÚN motor contesta, el ciclo
            se queda colgado INDEFINIDAMENTE (más de 5 minutos en una sola
            llamada). Sin este techo, un motor sin alimentar o un bus mal
            terminado congela el bucle de control entero: no se publica
            /joint_states, el watchdog no llega a contar y no hay paro de
            emergencia. Con el techo, el ciclo vuelve sin respuestas, el
            watchdog cuenta ciclos mudos y fuerza el paro, que es justo el
            comportamiento para el que se diseñó.

        Returns
        -------
        dict[int, bytes | None]
            {id_motor: datos_de_respuesta}. None si ese motor no respondió.

        Un solo ciclo para los 8 motores es la razón de ser del pi3hat: hacer
        pregunta-respuesta motor por motor costaría 8 transacciones SPI por
        iteración del bucle de control.
        """
        from ugv_bridge import protocolo_can as proto

        comandos = []
        for id_motor, datos in peticiones:
            bus = self.mapa_buses.get(id_motor, 1)
            cmd = _moteus.Command()
            # raw=True: la librería emite arbitration_id tal cual, sin componerlo
            # desde source/destination (ver la cabecera del módulo). Es lo que
            # permite mandar el ID del protocolo RMD sin tocar la librería.
            cmd.raw = True
            cmd.arbitration_id = proto.id_comando(id_motor)   # 0x140 + id_motor
            # El bus del pi3hat se elige SOLO con `bus`, que la librería resuelve
            # como `[d for d in devices if d.bus() == cmd.bus]`. NO usar
            # `channel`: ahí espera un objeto de dispositivo, no un número, y
            # pasarle un int revienta con "'int' object has no attribute
            # 'parent'" en el primer ciclo (comprobado en la placa).
            cmd.bus = bus
            cmd.data = bytes(datos)
            # OJO: reply_required=True encendería el bit 0x8000 del ID en la rama
            # NO raw. Aquí no aplica, pero se deja en False igual porque las
            # respuestas se recogen con force_can_check sobre los buses en uso.
            cmd.reply_required = False
            comandos.append(cmd)

        techo = max(float(timeout), TIMEOUT_CICLO_MIN_S)
        try:
            resultados = self.loop.run_until_complete(
                asyncio.wait_for(
                    self.router.cycle(comandos, force_can_check=self._mascara_buses),
                    timeout=techo,
                )
            )
        except asyncio.TimeoutError:
            # Nadie contestó a tiempo. NO es un error del transporte: es el caso
            # que el watchdog del driver sabe manejar (cuenta ciclos mudos por
            # motor y fuerza el paro). Se devuelve "ninguna respuesta" y el
            # bucle de control sigue vivo y publicando.
            self.timeouts += len(peticiones)
            self.ciclos_vencidos += 1
            return {id_motor: None for id_motor, _ in peticiones}
        except Exception as e:  # bus caído, SPI ocupado, motor desconectado...
            self.timeouts += len(peticiones)
            raise ErrorTransportePi3Hat(f'fallo en el ciclo del pi3hat: {e}') from e

        # Emparejar cada respuesta (ID 0x240+id) con su motor.
        respuestas = {}
        for r in resultados or []:
            arb = getattr(r, 'arbitration_id', None)
            if arb is None:
                continue
            id_motor = proto.motor_de_id_respuesta(arb)
            if id_motor is not None:
                respuestas[id_motor] = bytes(getattr(r, 'data', b''))

        salida = {}
        for id_motor, _ in peticiones:
            datos = respuestas.get(id_motor)
            if datos is None:
                self.timeouts += 1
            salida[id_motor] = datos
        return salida

    def cerrar(self):
        cerrar_router()


class ErrorTransportePi3Hat(RuntimeError):
    """Fallo de comunicación con la placa (no con un motor puntual)."""


# ====================================================================== #
# IMU DE LA PLACA
# ====================================================================== #
GRAVEDAD = 9.81


def _quat_mult(a, b):
    """(w,x,y,z) x (w,x,y,z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_de_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _quat_a_rpy(w, x, y, z):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _rotar_vector(q, v):
    """Rota el vector v por el cuaternión q=(w,x,y,z)."""
    w, x, y, z = q
    vx, vy, vz = v
    qv = (0.0, vx, vy, vz)
    qc = (w, -x, -y, -z)
    _, rx, ry, rz = _quat_mult(_quat_mult(q, qv), qc)
    return rx, ry, rz


class LectorImuPi3Hat:
    """Lee la IMU real del pi3hat y la entrega ya en el marco del robot.

    La rotación de MONTAJE de la placa (imu_montaje_roll/pitch/yaw de
    config/geometria_robot.yaml) se aplica aquí, de modo que quien consume esta
    clase recibe roll/pitch/yaw del CHASIS, no de la placa. Por eso imu_link se
    define alineado con base_link en el URDF: la corrección ya viene hecha.

    Parameters
    ----------
    montaje_rpy : tuple[float, float, float]
        Rotación de la placa respecto a base_link (rad).
    mapa_buses : dict | None
        Si el mismo proceso también habla con los motores, pasar el mismo mapa
        para que se comparta un único Pi3HatRouter.
    """

    def __init__(self, montaje_rpy=(0.0, 0.0, 0.0), mapa_buses=None):
        _importar()
        self.router = abrir_router(mapa_buses)
        self.loop = obtener_loop()
        self.q_montaje = _quat_de_rpy(*montaje_rpy)
        self.montaje_rpy = tuple(montaje_rpy)
        # Se resuelve en la primera lectura y se recuerda, para no re-inspeccionar
        # la estructura del objeto en cada ciclo.
        self._campo_accel = None
        self._accel_disponible = None
        self._t_ultimo_error = 0.0

        # El primer cycle([]) es el que despierta el bucle de la placa.
        self.loop.run_until_complete(self.router.cycle([]))

    # ------------------------------------------------------------------ #
    def leer(self):
        """Devuelve {'quat': (x,y,z,w), 'roll','pitch','yaw', 'accel_x/y/z'}.

        La estructura del objeto que devuelve `attitude()` cambia entre versiones
        de moteus_pi3hat (a veces `imu.w`, a veces `imu.attitude.w`), igual que ya
        contemplaba pi3hat_imu_node.py.
        """
        self.loop.run_until_complete(self.router.cycle([]))
        imu = self.loop.run_until_complete(self.router.attitude())

        try:
            w, x, y, z = imu.w, imu.x, imu.y, imu.z
        except AttributeError:
            a = imu.attitude
            w, x, y, z = a.w, a.x, a.y, a.z

        # Placa -> chasis.
        qw, qx, qy, qz = _quat_mult((w, x, y, z), self.q_montaje)
        roll, pitch, yaw = _quat_a_rpy(qw, qx, qy, qz)

        ax, ay, az = self._leer_aceleracion(imu, roll, pitch)

        return {
            'quat': (qx, qy, qz, qw),   # orden ROS (x, y, z, w)
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'accel_x': ax,
            'accel_y': ay,
            'accel_z': az,
            'accel_medida': self._accel_disponible,
        }

    # ------------------------------------------------------------------ #
    def _leer_aceleracion(self, imu, roll, pitch):
        """Aceleración lineal cruda si el firmware la expone; si no, la calcula.

        El firmware del pi3hat publica `accel_mps2` en algunas versiones y sólo el
        cuaternión en otras. Se prueba una vez qué hay y se recuerda. Sin dato
        crudo, se proyecta la gravedad según la actitud — el mismo modelo que usa
        la IMU sintética del gemelo digital, así el pipeline (EKF, Foxglove) ve
        siempre la misma forma de mensaje.
        """
        if self._accel_disponible is None:
            self._campo_accel = None
            for nombre in ('accel_mps2', 'acceleration_mps2', 'accel'):
                fuente = getattr(imu, nombre, None)
                if fuente is None and hasattr(imu, 'attitude'):
                    fuente = getattr(imu, nombre, None)
                if fuente is not None and hasattr(fuente, 'x'):
                    self._campo_accel = nombre
                    break
            self._accel_disponible = self._campo_accel is not None

        if self._accel_disponible:
            v = getattr(imu, self._campo_accel)
            # También hay que llevarla al marco del chasis.
            return _rotar_vector(self.q_montaje, (v.x, v.y, v.z))

        # Fallback: fuerza específica proyectada en el cuerpo (REP-103: X
        # adelante, Y izquierda, Z arriba). Misma convención que la IMU sintética
        # del gemelo digital, para que ambas fuentes sean intercambiables.
        return (
            -GRAVEDAD * math.sin(pitch),
            GRAVEDAD * math.sin(roll) * math.cos(pitch),
            GRAVEDAD * math.cos(roll) * math.cos(pitch),
        )

    def cerrar(self):
        cerrar_router()
