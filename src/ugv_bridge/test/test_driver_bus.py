#!/usr/bin/env python3
"""Pruebas del ciclo de bus del driver (modos 'can' y 'pi3hat').

Los dos modos comparten `_estado_via_bus`: lo que se valida aquí es el código que
correrá contra los motores reales. Para no depender de vcan0 ni de la placa, el
transporte se sustituye por uno en memoria que usa el MISMO modelo de motor y las
MISMAS funciones de protocolo que `motor_emulator` — o sea, se ejercitan de
verdad el empaquetado y desempaquetado de bytes, no un mock del protocolo.

    pytest src/ugv_bridge/test/test_driver_bus.py
"""
import math
import time

from ugv_bridge import protocolo_can as proto
from ugv_bridge.driver_movimiento import RMD_Hardware, IDS_ORUGAS, IDS_FLIPPERS
from ugv_bridge.motor_emulator import MotorEmulado, atender


class TransporteMemoria:
    """Los 8 motores emulados al otro lado, sin bus de por medio.

    Implementa la misma interfaz que TransportePi3Hat y que el camino SocketCAN:
    recibe una lista [(id_motor, bytes)] y devuelve {id_motor: bytes | None}.
    """

    def __init__(self, dt=0.01, mudos=()):
        self.motores = {mid: MotorEmulado(mid) for mid in IDS_ORUGAS + IDS_FLIPPERS}
        self.dt = dt
        self.mudos = set(mudos)
        self.timeouts = 0
        self.cerrado = False

    def intercambiar(self, peticiones, timeout=0.004):
        respuestas = {}
        for id_motor, datos in peticiones:
            if id_motor in self.mudos:
                respuestas[id_motor] = None
                self.timeouts += 1
                continue
            motor = self.motores[id_motor]
            motor.integrar(self.dt)
            comando = proto.parsear_comando(bytes(datos))
            assert comando is not None, f'trama no reconocida para el motor {id_motor}'
            respuestas[id_motor] = atender(motor, comando)
        return respuestas

    def cerrar(self):
        self.cerrado = True


def construir(**kwargs):
    """RMD_Hardware en modo 'pi3hat' con el transporte en memoria inyectado."""
    transporte = TransporteMemoria(mudos=kwargs.pop('mudos', ()))
    robot = RMD_Hardware(modo='pi3hat', transporte=transporte, **kwargs)
    return robot, transporte


def ciclar(robot, orugas=None, flippers=None, n=20, pausa=0.0):
    """Ejecuta n ciclos de control.

    `pausa` deja pasar tiempo real entre ciclos: la actitud de la IMU sintética se
    integra con time.monotonic(), así que sin pausa dt~0 y no avanza.
    """
    orugas = orugas or {mid: 0.0 for mid in IDS_ORUGAS}
    flippers = flippers or {mid: 0.0 for mid in IDS_FLIPPERS}
    estado = None
    for _ in range(n):
        estado = robot.enviar_y_leer_estado(orugas, flippers)
        if pausa:
            time.sleep(pausa)
    return estado


# ====================================================================== #
# Protocolo
# ====================================================================== #
def test_comando_de_velocidad_llega_a_las_orugas():
    """Una velocidad comandada vuelve leída desde el motor, ida y vuelta en bytes."""
    robot, transporte = construir()
    estado = ciclar(robot, orugas={mid: 3.0 for mid in IDS_ORUGAS}, n=5)

    for mid in IDS_ORUGAS:
        assert math.isclose(estado[mid]['velocidad_rad_s'], 3.0, abs_tol=0.2), \
            f'motor {mid}: {estado[mid]["velocidad_rad_s"]}'
        # La posición multivuelta debe estar avanzando (es la que usa la odometría).
        assert estado[mid]['posicion_rad'] > 0.0


def test_comando_de_posicion_mueve_los_flippers():
    robot, _ = construir()
    objetivo = 0.78
    estado = ciclar(robot, flippers={mid: objetivo for mid in IDS_FLIPPERS}, n=60)

    for mid in IDS_FLIPPERS:
        assert math.isclose(estado[mid]['posicion_rad'], objetivo, abs_tol=0.05), \
            f'flipper {mid} quedó en {estado[mid]["posicion_rad"]}'


def test_posicion_multivuelta_no_se_enrolla():
    """Más de una vuelta completa: la posición debe seguir creciendo."""
    robot, _ = construir()
    estado = ciclar(robot, orugas={mid: 20.0 for mid in IDS_ORUGAS}, n=100)
    for mid in IDS_ORUGAS:
        assert estado[mid]['posicion_rad'] > 2 * math.pi


# ====================================================================== #
# Watchdog de seguridad
# ====================================================================== #
def test_watchdog_detecta_motor_mudo_y_fuerza_paro():
    """Un motor que deja de responder debe frenar el robot, no dejarlo a ciegas."""
    robot, transporte = construir(ciclos_watchdog=5)

    ciclar(robot, orugas={mid: 5.0 for mid in IDS_ORUGAS}, n=10)
    assert not robot.fallo_comunicacion

    # El motor 3 (track_rl) se queda mudo.
    transporte.mudos.add(3)
    ciclar(robot, orugas={mid: 5.0 for mid in IDS_ORUGAS}, n=10)

    assert robot.fallo_comunicacion, 'el watchdog no detectó el motor mudo'
    # Y los motores que SÍ responden deben haber recibido velocidad cero.
    for mid in IDS_ORUGAS:
        if mid == 3:
            continue
        assert math.isclose(transporte.motores[mid].objetivo, 0.0, abs_tol=1e-6), \
            f'motor {mid} seguía comandado a {transporte.motores[mid].objetivo}'


def test_watchdog_se_recupera_solo():
    robot, transporte = construir(ciclos_watchdog=5)
    transporte.mudos.add(7)
    ciclar(robot, n=10)
    assert robot.fallo_comunicacion

    transporte.mudos.clear()
    ciclar(robot, n=10)
    assert not robot.fallo_comunicacion, 'no se limpió el fallo al volver el motor'


def test_cerrar_detiene_los_motores():
    robot, transporte = construir()
    ciclar(robot, orugas={mid: 8.0 for mid in IDS_ORUGAS}, n=5)
    robot.cerrar()

    for mid in IDS_ORUGAS:
        assert math.isclose(transporte.motores[mid].objetivo, 0.0, abs_tol=1e-6)
    assert transporte.cerrado


# ====================================================================== #
# Fuente de IMU independiente del modo de motor
# ====================================================================== #
def test_imu_sintetica_sigue_el_movimiento():
    """Girando en el sitio, el yaw sintético debe moverse."""
    robot, _ = construir()
    ciclar(robot, orugas={1: -5.0, 3: -5.0, 2: 5.0, 4: 5.0}, n=50, pausa=0.002)
    imu = robot.leer_imu()
    # Girando en el sitio a +-5 rad/s durante ~0.1 s: |yaw| del orden de 0.1 rad.
    assert abs(imu['yaw']) > 0.05
    assert 'quat' not in imu   # la sintética entrega Euler; la real, cuaternión


def test_imu_nivelada_en_reposo():
    """Nivelado, el acelerómetro debe leer (0, 0, +g): fuerza específica, no gravedad."""
    robot, _ = construir()
    ciclar(robot, n=5, pausa=0.002)
    imu = robot.leer_imu()
    assert abs(imu['accel_x']) < 0.1
    assert abs(imu['accel_y']) < 0.1
    assert math.isclose(imu['accel_z'], 9.81, abs_tol=0.1)


def test_signos_de_actitud_segun_rep103():
    """Levantar los flippers delanteros = morro arriba = pitch NEGATIVO.

    En REP-103 el eje Y apunta a la IZQUIERDA, así que un pitch positivo es morro
    abajo (al revés que en la convención aeronáutica). Un flipper con ángulo
    positivo gira su punta hacia el suelo y levanta ese extremo del chasis.
    """
    robot, _ = construir()
    # Flippers delanteros (IDs 5 y 6) arriba, traseros en cero.
    ciclar(robot, flippers={5: 1.0, 6: 1.0, 7: 0.0, 8: 0.0}, n=60, pausa=0.002)
    imu = robot.leer_imu()
    assert imu['pitch'] < -0.05, f'pitch={imu["pitch"]}: el morro debería subir'
    # Y con morro arriba, el acelerómetro ve componente X POSITIVA.
    assert imu['accel_x'] > 0.1

    # Flippers derechos (IDs 6 y 8) arriba -> costado derecho arriba -> roll < 0.
    robot2, _ = construir()
    ciclar(robot2, flippers={5: 0.0, 6: 1.0, 7: 0.0, 8: 1.0}, n=60, pausa=0.002)
    assert robot2.leer_imu()['roll'] < -0.05


def test_imu_fuente_invalida_falla_temprano():
    import pytest
    with pytest.raises(ValueError):
        RMD_Hardware(modo='gemelo', imu_fuente='inventada')
