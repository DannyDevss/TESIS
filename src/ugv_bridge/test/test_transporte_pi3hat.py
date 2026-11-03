#!/usr/bin/env python3
"""Pruebas de TransportePi3Hat sin la placa.

El router falso imita lo que hace moteus_pi3hat 1.1.0 con `force_can_check`
(comprobado en la Raspberry el 03/11/2026): `cycle()` devuelve UNA trama por
bus, la primera que llegó, y deja el resto en `_parent._receive_queue`.

    pytest src/ugv_bridge/test/test_transporte_pi3hat.py
"""
import asyncio
import types

from ugv_bridge import pi3hat_backend as backend
from ugv_bridge import protocolo_can as proto
from ugv_bridge.driver_movimiento import CICLOS_WATCHDOG


class _Trama:
    def __init__(self, nodo, cmd, datos=b'\x00' * 8):
        self.arbitration_id = proto.id_arbitraje(nodo, cmd)
        self.data = datos


class _RouterFalso:
    """Un bus con varios motores que emiten; cycle() entrega sólo la primera."""

    def __init__(self, tramas_por_ciclo, con_cola=True):
        self._tramas_por_ciclo = tramas_por_ciclo   # función -> lista de _Trama
        if con_cola:
            self._parent = types.SimpleNamespace(_receive_queue=[])
        self.ciclo = 0

    async def cycle(self, comandos, force_can_check=0):
        tramas = self._tramas_por_ciclo(self.ciclo)
        self.ciclo += 1
        if hasattr(self, '_parent'):
            self._parent._receive_queue.extend(tramas[1:])
        return tramas[:1]


def _transporte(router):
    backend._moteus = types.SimpleNamespace(Command=types.SimpleNamespace)
    t = object.__new__(backend.TransportePi3Hat)
    t.mapa_buses = {1: 1, 5: 1}
    t.router = router
    t.loop = asyncio.new_event_loop()
    t.timeouts = 0
    t.ciclos_vencidos = 0
    t._mascara_buses = 1 << 1
    return t


def _peticiones():
    return [(1, proto.CMD_SET_INPUT_VEL, b'\x00' * 8),
            (5, proto.CMD_SET_INPUT_POS, b'\x00' * 8)]


def _ambos_emiten(_ciclo):
    # El 5 siempre llega primero: es el peor caso para el 1.
    return [_Trama(5, proto.CMD_GET_ENCODER_ESTIMATES),
            _Trama(1, proto.CMD_GET_ENCODER_ESTIMATES)]


def test_dos_motores_en_un_bus_se_ven_los_dos_cada_ciclo():
    t = _transporte(_RouterFalso(_ambos_emiten))
    for _ in range(3 * CICLOS_WATCHDOG):
        recibido = t.intercambiar(_peticiones())
        assert proto.CMD_GET_ENCODER_ESTIMATES in recibido[1]
        assert proto.CMD_GET_ENCODER_ESTIMATES in recibido[5]
    assert t.timeouts == 0
    assert t.router._parent._receive_queue == []


def test_sin_leer_la_cola_un_motor_parece_mudo():
    """El fallo original: sin cola, el motor 1 no aparece nunca."""
    t = _transporte(_RouterFalso(_ambos_emiten, con_cola=False))
    for _ in range(CICLOS_WATCHDOG):
        recibido = t.intercambiar(_peticiones())
        assert recibido[1] == {}
    assert t.timeouts == CICLOS_WATCHDOG


def test_gana_la_trama_mas_reciente_de_cada_tipo():
    def dos_encoder_del_mismo_motor(_ciclo):
        return [_Trama(1, proto.CMD_GET_ENCODER_ESTIMATES, b'\x01' * 8),
                _Trama(1, proto.CMD_HEARTBEAT, b'\x02' * 8),
                _Trama(1, proto.CMD_GET_ENCODER_ESTIMATES, b'\x03' * 8)]

    t = _transporte(_RouterFalso(dos_encoder_del_mismo_motor))
    recibido = t.intercambiar(_peticiones())
    assert recibido[1][proto.CMD_GET_ENCODER_ESTIMATES] == b'\x03' * 8
    assert recibido[1][proto.CMD_HEARTBEAT] == b'\x02' * 8
    assert recibido[5] == {}
