#!/usr/bin/env python3
"""protocolo_can.py — Empaquetado/desempaquetado de tramas CAN de los motores.

Única fuente de verdad del protocolo: la usan tanto el driver (`driver_movimiento`,
lado Raspberry/pi3hat) como el emulador (`motor_emulator`, lado "motores").

---------------------------------------------------------------------------
PROTOCOLO ODrive (verificado contra el manual del GIM6010-8 rev1.3, 24/08/2026)
---------------------------------------------------------------------------
Los motores son SteadyWin GIM6010-8. Su driver (CyberBeast BL72, USB 1209:0d32)
es COMPATIBLE ODrive: firmware 0.6.5, `odrivetool` como consola. NO hablan el
protocolo RMD de las controladoras chinas tipo MyActuator.

    ID de arbitraje (11 bits) = (node_id << 5) | cmd_id

    Bit10..Bit5 -> node_id (1..63)      Bit4..Bit0 -> cmd_id (0..31)

OJO: la versión anterior de este archivo implementaba RMD V3 (ID 0x140 + id,
comandos 0xA2/0xA4/0x9C). Era una SUPOSICIÓN, anotada en su día como
"TODO(manual): verificar contra el manual oficial", y resultó falsa: para el
motor, un 0x141 significa node_id=10 / cmd_id=1, un mensaje sin sentido. Ningún
motor respondió jamás, y el síntoma era indistinguible de un cable suelto.

---------------------------------------------------------------------------
EL MOTOR HABLA SOLO (y por eso aquí no hay funciones de "pedir estado")
---------------------------------------------------------------------------
De fábrica el driver emite dos mensajes periódicos sin que nadie los pida
(sección 4.1.5 del manual):

    Heartbeat (0x001)               cada 100 ms
    Get_Encoder_Estimates (0x009)   cada  10 ms   <- posición y velocidad

Los 10 ms encajan con el bucle de control a 100 Hz: el driver manda su comando y
recoge lo que haya llegado, sin round-trip. Las demás lecturas (Iq, temperatura,
tensión de bus) están APAGADAS de fábrica; se encienden por USB con
`odrv0.axis0.config.can.iq_rate_ms = 10` y compañía, no por CAN.

---------------------------------------------------------------------------
UNIDADES
---------------------------------------------------------------------------
ODrive trabaja en REVOLUCIONES y rev/s; el resto de la tesis, en radianes y
rad/s (REP-103). La conversión vive aquí y sólo aquí.

Además hay una reducción mecánica entre el rotor (donde mide el encoder del
driver) y el eje de salida (lo que mueve la oruga o el flipper). Ver
RELACION_REDUCCION, que está SIN VERIFICAR en hardware.
"""
import math
import struct

# ---------------------------------------------------------------------- #
# IDs de arbitraje CAN
# ---------------------------------------------------------------------- #
BITS_CMD = 5
MASCARA_CMD = (1 << BITS_CMD) - 1     # 0x1F
NODE_ID_MAX = 63

# Códigos de mensaje (cmd_id). Sólo los que usa la tesis; la tabla completa está
# en la sección 4.1.2 del manual.
CMD_HEARTBEAT = 0x001            # motor -> host, periódico
CMD_ESTOP = 0x002                # host -> motor, sin datos
CMD_SET_AXIS_STATE = 0x007       # host -> motor, uint32
CMD_GET_ENCODER_ESTIMATES = 0x009  # motor -> host, float32 pos + float32 vel
CMD_SET_CONTROLLER_MODE = 0x00B  # host -> motor, uint32 control + uint32 input
CMD_SET_INPUT_POS = 0x00C        # host -> motor, float32 pos + int16 velff + int16 tqff
CMD_SET_INPUT_VEL = 0x00D        # host -> motor, float32 vel + float32 torque_ff
CMD_SET_LIMITS = 0x00F           # host -> motor, float32 vel_lim + float32 curr_lim
CMD_GET_IQ = 0x014               # motor -> host, float32 setpoint + float32 medido
CMD_CLEAR_ERRORS = 0x018         # host -> motor, sin datos

# Estados del eje (odrv0.axis0.requested_state).
ESTADO_IDLE = 1
ESTADO_CLOSED_LOOP = 8

# Modos del controlador (odrv0.axis0.controller.config.control_mode).
CONTROL_TORQUE = 1
CONTROL_VELOCIDAD = 2
CONTROL_POSICION = 3

# Modos de entrada (odrv0.axis0.controller.config.input_mode).
ENTRADA_PASSTHROUGH = 1
ENTRADA_VEL_RAMP = 2
ENTRADA_POS_FILTER = 3

# ---------------------------------------------------------------------- #
# Escalas y mecánica
# ---------------------------------------------------------------------- #
# Reducción entre el rotor (que es lo que cuenta el encoder del driver y a lo que
# se refieren los comandos por CAN) y el eje de salida.
#
# TODO(hardware): VERIFICAR antes de fiarse de la odometría. El procedimiento son
# dos minutos: con el motor en IDLE, girar el eje de SALIDA exactamente una vuelta
# a mano y mirar cuánto cambia odrv0.axis0.encoder.pos_estimate por USB. Si cambia
# 8.0, este valor es correcto; si cambia 1.0, el encoder ya mide en la salida y
# hay que poner 1.0 aquí.
#
# El nombre del motor (GIM6010-**8**) y el manual apuntan a 8:1, y el manual dice
# que los comandos CAN normales usan la misma referencia que el control por USB,
# o sea el ROTOR (sólo el modo MIT usa el eje de salida). Pero eso es lectura de
# manual, no medición, y en este proyecto ya hubo un protocolo entero deducido de
# una suposición parecida.
RELACION_REDUCCION = 8.0

# Constante de torque vista desde el EJE DE SALIDA (Nm por amperio de Iq).
# Sólo se usa si se activan los mensajes periódicos de Iq (iq_rate_ms, apagados
# de fábrica). TODO(datasheet): Kt del GIM6010-8 multiplicado por la reducción.
KT_NM_POR_A = 1.0

# Escalas de los campos comprimidos de Set_Input_Pos.
REV_S_POR_LSB_VELFF = 0.001    # int16 de Vel_FF
NM_POR_LSB_TQFF = 0.001        # int16 de Torque_FF

_REV_A_RAD = 2.0 * math.pi
_RAD_A_REV = 1.0 / _REV_A_RAD


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------- #
# Conversión de unidades (rotor <-> eje de salida, rev <-> rad)
# ---------------------------------------------------------------------- #
def rad_salida_a_rev_rotor(rad):
    """Radianes en el eje de salida -> revoluciones del rotor (lo que come el CAN)."""
    return rad * _RAD_A_REV * RELACION_REDUCCION


def rev_rotor_a_rad_salida(rev):
    """Revoluciones del rotor -> radianes en el eje de salida (unidades ROS)."""
    return rev * _REV_A_RAD / RELACION_REDUCCION


# ---------------------------------------------------------------------- #
# IDs de arbitraje
# ---------------------------------------------------------------------- #
def id_arbitraje(node_id, cmd_id):
    """ID de 11 bits para un mensaje `cmd_id` dirigido al nodo `node_id`."""
    if not 1 <= node_id <= NODE_ID_MAX:
        raise ValueError(f'node_id fuera de rango: {node_id}')
    if not 0 <= cmd_id <= MASCARA_CMD:
        raise ValueError(f'cmd_id fuera de rango: {cmd_id}')
    return (node_id << BITS_CMD) | cmd_id


def nodo_de_id(id_arb):
    """node_id que emite/recibe una trama, o None si el ID no es plausible."""
    nodo = id_arb >> BITS_CMD
    return nodo if 1 <= nodo <= NODE_ID_MAX else None


def cmd_de_id(id_arb):
    """cmd_id de una trama."""
    return id_arb & MASCARA_CMD


# ====================================================================== #
# LADO DRIVER (Raspberry/pi3hat): armar comandos, leer lo que emite el motor
# ====================================================================== #
def trama_set_input_vel(vel_rad_s, torque_ff_nm=0.0):
    """0x00D: velocidad objetivo. Entrada en rad/s del EJE DE SALIDA."""
    return struct.pack('<ff', rad_salida_a_rev_rotor(vel_rad_s), float(torque_ff_nm))


def trama_set_input_pos(pos_rad, vel_ff_rad_s=0.0, torque_ff_nm=0.0):
    """0x00C: posición objetivo. Entrada en radianes del EJE DE SALIDA.

    Vel_FF y Torque_FF viajan comprimidos en int16 (0.001 rev/s y 0.001 Nm), así
    que su rango útil es +-32.7 rev/s y +-32.7 Nm. Se saturan en vez de desbordar.
    """
    vel_ff = _clamp(
        int(round(rad_salida_a_rev_rotor(vel_ff_rad_s) / REV_S_POR_LSB_VELFF)),
        -(2**15), 2**15 - 1)
    tq_ff = _clamp(int(round(torque_ff_nm / NM_POR_LSB_TQFF)), -(2**15), 2**15 - 1)
    return struct.pack('<fhh', rad_salida_a_rev_rotor(pos_rad), vel_ff, tq_ff)


def trama_set_axis_state(estado):
    """0x007: pedir un estado del eje (ESTADO_IDLE / ESTADO_CLOSED_LOOP)."""
    return struct.pack('<I', int(estado))


def trama_set_controller_mode(control_mode, input_mode):
    """0x00B: modo de control (velocidad/posición) y modo de entrada."""
    return struct.pack('<II', int(control_mode), int(input_mode))


def trama_set_limits(vel_limite_rad_s, corriente_limite_a):
    """0x00F: límites de velocidad (rad/s en la salida) y de corriente (A)."""
    return struct.pack('<ff', rad_salida_a_rev_rotor(abs(vel_limite_rad_s)),
                       float(abs(corriente_limite_a)))


def trama_estop():
    """0x002: paro de emergencia. Desarma el eje; para volver a mover hay que
    limpiar errores y pedir de nuevo CLOSED_LOOP."""
    return b''


def trama_clear_errors():
    """0x018: limpiar los errores acumulados del eje."""
    return b''


def parsear_encoder(data):
    """0x009 -> {'posicion_rad', 'velocidad_rad_s'} en el eje de salida."""
    if len(data) < 8:
        return None
    pos_rev, vel_rev_s = struct.unpack('<ff', bytes(data[:8]))
    return {
        'posicion_rad': rev_rotor_a_rad_salida(pos_rev),
        'velocidad_rad_s': rev_rotor_a_rad_salida(vel_rev_s),
    }


def parsear_heartbeat(data):
    """0x001 -> {'error_eje', 'estado_eje', 'error_motor', 'error_encoder',
    'error_controlador', 'trayectoria_hecha'}.

    Formato de firmware >= 0.5.12: los flags van empaquetados en un byte, y el
    bit vale 1 cuando NO hay error (el manual lo enuncia como "¿es el error 0?").
    """
    if len(data) < 6:
        return None
    error_eje, estado_eje, flags = struct.unpack('<IBB', bytes(data[:6]))
    return {
        'error_eje': error_eje,
        'estado_eje': estado_eje,
        'error_motor': not (flags & 0x01),
        'error_encoder': not (flags & 0x02),
        'error_controlador': not (flags & 0x04),
        'trayectoria_hecha': bool(flags & 0x80),
    }


def parsear_iq(data):
    """0x014 -> {'iq_objetivo_a', 'iq_medido_a'} (apagado de fábrica)."""
    if len(data) < 8:
        return None
    objetivo, medido = struct.unpack('<ff', bytes(data[:8]))
    return {'iq_objetivo_a': objetivo, 'iq_medido_a': medido}


def parsear(cmd_id, data):
    """Despacha al parser que corresponda, o None si el mensaje no se usa aquí."""
    if cmd_id == CMD_GET_ENCODER_ESTIMATES:
        return parsear_encoder(data)
    if cmd_id == CMD_HEARTBEAT:
        return parsear_heartbeat(data)
    if cmd_id == CMD_GET_IQ:
        return parsear_iq(data)
    return None


# ====================================================================== #
# LADO MOTOR (emulador): leer comandos, armar los mensajes periódicos
# ====================================================================== #
def parsear_comando(cmd_id, data):
    """Decodifica un mensaje host->motor -> dict, o None si no se reconoce."""
    if cmd_id == CMD_SET_INPUT_VEL and len(data) >= 8:
        vel_rev, tq = struct.unpack('<ff', bytes(data[:8]))
        return {'cmd': cmd_id,
                'velocidad_rad_s': rev_rotor_a_rad_salida(vel_rev),
                'torque_ff_nm': tq}

    if cmd_id == CMD_SET_INPUT_POS and len(data) >= 8:
        pos_rev, vel_ff, tq_ff = struct.unpack('<fhh', bytes(data[:8]))
        return {'cmd': cmd_id,
                'posicion_rad': rev_rotor_a_rad_salida(pos_rev),
                'vel_ff_rad_s': rev_rotor_a_rad_salida(vel_ff * REV_S_POR_LSB_VELFF),
                'torque_ff_nm': tq_ff * NM_POR_LSB_TQFF}

    if cmd_id == CMD_SET_AXIS_STATE and len(data) >= 4:
        (estado,) = struct.unpack('<I', bytes(data[:4]))
        return {'cmd': cmd_id, 'estado': estado}

    if cmd_id == CMD_SET_CONTROLLER_MODE and len(data) >= 8:
        control, entrada = struct.unpack('<II', bytes(data[:8]))
        return {'cmd': cmd_id, 'control_mode': control, 'input_mode': entrada}

    if cmd_id == CMD_SET_LIMITS and len(data) >= 8:
        vel_lim, corr_lim = struct.unpack('<ff', bytes(data[:8]))
        return {'cmd': cmd_id,
                'vel_limite_rad_s': rev_rotor_a_rad_salida(vel_lim),
                'corriente_limite_a': corr_lim}

    if cmd_id in (CMD_ESTOP, CMD_CLEAR_ERRORS):
        return {'cmd': cmd_id}

    return None


def trama_encoder(pos_rad, vel_rad_s):
    """0x009 tal como lo emite el motor. Entradas en el eje de salida."""
    return struct.pack('<ff', rad_salida_a_rev_rotor(pos_rad),
                       rad_salida_a_rev_rotor(vel_rad_s))


def trama_heartbeat(estado_eje=ESTADO_CLOSED_LOOP, error_eje=0,
                    error_motor=False, error_encoder=False,
                    error_controlador=False, trayectoria_hecha=True):
    """0x001 tal como lo emite el motor (formato de firmware >= 0.5.12)."""
    flags = 0
    if not error_motor:
        flags |= 0x01
    if not error_encoder:
        flags |= 0x02
    if not error_controlador:
        flags |= 0x04
    if trayectoria_hecha:
        flags |= 0x80
    return struct.pack('<IBB2x', int(error_eje), int(estado_eje), flags)


def trama_iq(iq_objetivo_a, iq_medido_a):
    """0x014 tal como lo emite el motor (sólo si se activa iq_rate_ms)."""
    return struct.pack('<ff', float(iq_objetivo_a), float(iq_medido_a))
