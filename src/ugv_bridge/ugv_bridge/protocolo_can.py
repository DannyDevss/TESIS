#!/usr/bin/env python3
"""protocolo_can.py — Empaquetado/desempaquetado de tramas CAN de los motores.

Única fuente de verdad del protocolo: la usan tanto el driver (`driver_movimiento`,
lado Raspberry/pi3hat) como el emulador (`motor_emulator`, lado "motores"). Si al
contrastar con el manual de SteadyWin hay que corregir un byte, se corrige AQUÍ
y ambos lados quedan consistentes.

Protocolo estilo RMD (familia SteadyWin GIM6010-8 / GDS68), tramas CAN 2.0A
estándar de 8 bytes, little-endian:

    Comando  al motor : ID = 0x140 + id_motor   (id_motor 1..8)
    Respuesta del motor: ID = 0x240 + id_motor

Comandos usados:
    0xA2 velocidad lazo cerrado : bytes 4-7 = int32, 0.01 dps/LSB
    0xA4 posición  lazo cerrado : bytes 2-3 = uint16 vel. máx (1 dps/LSB),
                                  bytes 4-7 = int32 ángulo (0.01 grado/LSB)
    0x9C leer estado 2          : sin payload
    0x92 leer ángulo multivuelta: sin payload
    0x81 paro (stop)            : sin payload
    0x80 apagado (motor off)    : sin payload

Respuesta a 0xA2/0xA4/0x9C (estado del motor):
    byte 1   = int8   temperatura (1 °C/LSB)
    bytes 2-3= int16  corriente iq (0.01 A/LSB)  -> torque = iq * KT_NM_POR_A
    bytes 4-5= int16  velocidad (1 dps/LSB)
    bytes 6-7= int16  ángulo de UNA vuelta (1 grado/LSB)

Respuesta a 0x92:
    bytes 4-7= int32  ángulo multivuelta (0.01 grado/LSB)  -> posición para odometría

TODO(manual): verificar escalas, orden de bytes y IDs de respuesta contra el
manual oficial de SteadyWin antes de usar con hardware real. Aquí se siguió el
protocolo RMD V3 estándar, que es el que clonan estos drivers.
"""
import math
import struct

# ---------------------------------------------------------------------- #
# IDs de arbitraje CAN
# ---------------------------------------------------------------------- #
ID_CMD_BASE = 0x140   # comando hacia el motor:   0x140 + id_motor
ID_RESP_BASE = 0x240  # respuesta desde el motor: 0x240 + id_motor

# Códigos de comando (byte 0 de la trama; la respuesta hace eco del código).
CMD_APAGADO = 0x80
CMD_PARO = 0x81
CMD_LEER_MULTIVUELTA = 0x92
CMD_LEER_ESTADO = 0x9C
CMD_VELOCIDAD = 0xA2
CMD_POSICION = 0xA4

# ---------------------------------------------------------------------- #
# Escalas del protocolo (unidades físicas <-> enteros CAN)
# ---------------------------------------------------------------------- #
DPS_POR_LSB_CMD = 0.01     # int32 de 0xA2: 0.01 grados/segundo por cuenta
GRADOS_POR_LSB_POS = 0.01  # int32 de 0xA4 y 0x92: 0.01 grado por cuenta
AMP_POR_LSB_IQ = 0.01      # int16 de corriente iq: 0.01 A por cuenta

# Constante de torque a la SALIDA del reductor (Nm por amperio de iq).
# TODO(datasheet): ajustar con el valor real del GIM6010-8 (Kt motor x 8:1).
KT_NM_POR_A = 1.0

_RAD_A_GRADO = 180.0 / math.pi
_GRADO_A_RAD = math.pi / 180.0


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def id_comando(id_motor):
    """ID de arbitraje para comandar al motor `id_motor`."""
    return ID_CMD_BASE + id_motor


def id_respuesta(id_motor):
    """ID de arbitraje con el que responde el motor `id_motor`."""
    return ID_RESP_BASE + id_motor


def motor_de_id_comando(id_arbitraje):
    """ID de motor a partir del ID de arbitraje de un comando (o None si no es comando)."""
    id_motor = id_arbitraje - ID_CMD_BASE
    return id_motor if 1 <= id_motor <= 32 else None


def motor_de_id_respuesta(id_arbitraje):
    """ID de motor a partir del ID de arbitraje de una respuesta (o None si no lo es).

    Lo usa el transporte pi3hat, que recibe en cada ciclo TODAS las tramas vistas
    en los buses y necesita saber a qué motor pertenece cada una.
    """
    id_motor = id_arbitraje - ID_RESP_BASE
    return id_motor if 1 <= id_motor <= 32 else None


# ====================================================================== #
# LADO DRIVER (Raspberry/pi3hat): armar comandos, leer respuestas
# ====================================================================== #
def trama_cmd_velocidad(vel_rad_s):
    """0xA2: velocidad en lazo cerrado. Entrada en rad/s (unidades ROS)."""
    cuentas = int(round(math.degrees(vel_rad_s) / DPS_POR_LSB_CMD))
    cuentas = _clamp(cuentas, -(2**31), 2**31 - 1)
    return struct.pack('<B3xi', CMD_VELOCIDAD, cuentas)


def trama_cmd_posicion(pos_rad, vel_max_rad_s=2.0 * math.pi):
    """0xA4: posición absoluta en lazo cerrado. Entrada en radianes."""
    vel_max_dps = int(round(math.degrees(abs(vel_max_rad_s))))
    vel_max_dps = _clamp(vel_max_dps, 0, 2**16 - 1)
    cuentas = int(round(math.degrees(pos_rad) / GRADOS_POR_LSB_POS))
    cuentas = _clamp(cuentas, -(2**31), 2**31 - 1)
    return struct.pack('<BxHi', CMD_POSICION, vel_max_dps, cuentas)


def trama_leer_estado():
    """0x9C: solicitar estado (temperatura, iq, velocidad, ángulo 1 vuelta)."""
    return struct.pack('<B7x', CMD_LEER_ESTADO)


def trama_leer_multivuelta():
    """0x92: solicitar ángulo multivuelta (posición acumulada, para odometría)."""
    return struct.pack('<B7x', CMD_LEER_MULTIVUELTA)


def trama_paro():
    """0x81: detener el motor (mantiene el driver energizado)."""
    return struct.pack('<B7x', CMD_PARO)


def parsear_respuesta(data):
    """Decodifica una trama de respuesta del motor -> dict con unidades ROS.

    Devuelve {'cmd': int, ...campos según el comando...} o None si no se reconoce.
    """
    if len(data) != 8:
        return None
    cmd = data[0]

    if cmd in (CMD_VELOCIDAD, CMD_POSICION, CMD_LEER_ESTADO):
        _, temp, iq, vel_dps, ang_grados = struct.unpack('<Bbhhh', bytes(data))
        return {
            'cmd': cmd,
            'temperatura_c': float(temp),
            'torque_nm': iq * AMP_POR_LSB_IQ * KT_NM_POR_A,
            'velocidad_rad_s': vel_dps * _GRADO_A_RAD,
            'posicion_1v_rad': ang_grados * _GRADO_A_RAD,
        }

    if cmd == CMD_LEER_MULTIVUELTA:
        _, cuentas = struct.unpack('<B3xi', bytes(data))
        return {
            'cmd': cmd,
            'posicion_rad': cuentas * GRADOS_POR_LSB_POS * _GRADO_A_RAD,
        }

    if cmd in (CMD_PARO, CMD_APAGADO):
        return {'cmd': cmd}

    return None


# ====================================================================== #
# LADO MOTOR (emulador): leer comandos, armar respuestas
# ====================================================================== #
def parsear_comando(data):
    """Decodifica una trama de comando recibida por el motor -> dict.

    Devuelve {'cmd': int, ...} o None si el comando no se reconoce.
    """
    if len(data) != 8:
        return None
    cmd = data[0]

    if cmd == CMD_VELOCIDAD:
        _, cuentas = struct.unpack('<B3xi', bytes(data))
        return {'cmd': cmd, 'velocidad_rad_s': cuentas * DPS_POR_LSB_CMD * _GRADO_A_RAD}

    if cmd == CMD_POSICION:
        _, vel_max_dps, cuentas = struct.unpack('<BxHi', bytes(data))
        return {
            'cmd': cmd,
            'posicion_rad': cuentas * GRADOS_POR_LSB_POS * _GRADO_A_RAD,
            'vel_max_rad_s': vel_max_dps * _GRADO_A_RAD,
        }

    if cmd in (CMD_LEER_ESTADO, CMD_LEER_MULTIVUELTA, CMD_PARO, CMD_APAGADO):
        return {'cmd': cmd}

    return None


def trama_respuesta_estado(cmd_eco, temperatura_c, torque_nm, vel_rad_s, pos_rad):
    """Respuesta de estado (eco de 0xA2/0xA4/0x9C). Entradas en unidades ROS."""
    iq = _clamp(int(round(torque_nm / KT_NM_POR_A / AMP_POR_LSB_IQ)), -(2**15), 2**15 - 1)
    vel_dps = _clamp(int(round(math.degrees(vel_rad_s))), -(2**15), 2**15 - 1)
    # Ángulo de UNA vuelta en grados enteros [0, 360): así lo reporta el motor.
    ang_1v = int(math.degrees(pos_rad) % 360.0)
    return struct.pack('<Bbhhh', cmd_eco, _clamp(int(temperatura_c), -128, 127),
                       iq, vel_dps, ang_1v)


def trama_respuesta_multivuelta(pos_rad):
    """Respuesta a 0x92: posición acumulada multivuelta. Entrada en radianes."""
    cuentas = int(round(math.degrees(pos_rad) / GRADOS_POR_LSB_POS))
    cuentas = _clamp(cuentas, -(2**31), 2**31 - 1)
    return struct.pack('<B3xi', CMD_LEER_MULTIVUELTA, cuentas)
