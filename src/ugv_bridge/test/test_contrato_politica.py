"""Pruebas del contrato de la política. Puras: no necesitan ROS ni onnxruntime.

Correr desde src/ugv_bridge:  python3 -m pytest test/test_contrato_politica.py
"""
import json
import math

import numpy as np
import pytest

from ugv_bridge import contrato_politica as cp


def _estado():
    e = cp.EstadoRobot()
    e.pos_flipper = [0.1, -0.2, 0.3, -0.4]
    e.vel_flipper = [1.5, 0.0, -3.0, 6.0]
    e.esf_flipper = [2.0, 0.0, 0.0, -40.0]
    e.consigna_flipper = [0.2, -0.2, 0.3, -0.4]
    e.vel_oruga = [6.0, 6.0, 5.0, 5.0]
    e.esf_oruga = [1.0, 1.0, 1.0, 1.0]
    e.consigna_oruga = [6.0, 6.0, 6.0, 6.0]
    e.roll, e.pitch = 0.1, -0.2
    e.giro = [0.3, 0.0, -0.6]
    e.acel = [0.0, 0.0, 9.81]
    return e


def _minimo(**cambios):
    datos = {
        'formato': cp.FORMATO_CONTRATO,
        'observacion': [{'senal': 'flipper_fl/pos', 'op': 'crudo'}],
        'accion': [{'destino': 'flipper_fl', 'modo': 'velocidad', 'escala': 1.0}],
    }
    datos.update(cambios)
    return datos


# ---------------------------------------------------------------------- #
# Contrato provisional
# ---------------------------------------------------------------------- #
def test_provisional_dimensiones():
    c = cp.contrato_provisional()
    assert c.obs_dim == 40
    assert c.acc_dim == 6
    assert len(c.descripcion_observacion()) == 40


def test_provisional_layout_conocido():
    """Mismos valores, en el mismo orden, que el contrato v1 escrito a mano."""
    c = cp.contrato_provisional()
    obs = c.observacion(_estado())
    assert obs.dtype == np.float32
    # flipper_fl: sin, cos, vel/3, error/0.35, esfuerzo/20
    assert obs[0] == pytest.approx(math.sin(0.1), abs=1e-6)
    assert obs[1] == pytest.approx(math.cos(0.1), abs=1e-6)
    assert obs[2] == pytest.approx(0.5, abs=1e-6)
    assert obs[3] == pytest.approx(0.1 / 0.35, abs=1e-6)
    assert obs[4] == pytest.approx(0.1, abs=1e-6)
    # flipper_rr satura: vel 6/3 -> 1, esfuerzo -40/20 -> -1
    assert obs[17] == 1.0
    assert obs[19] == -1.0
    # track_rl: vel 5/12, desliz (6-5)/12
    assert obs[26] == pytest.approx(5 / 12, abs=1e-6)
    assert obs[27] == pytest.approx(1 / 12, abs=1e-6)
    # chasis
    assert obs[32] == pytest.approx(math.sin(0.1), abs=1e-6)
    assert obs[39] == pytest.approx(9.81 / 20, abs=1e-6)


def test_huella_estable_y_sensible():
    a = cp.contrato_provisional()
    b = cp.contrato_provisional()
    assert a.huella() == b.huella()
    datos = a.a_dict()
    datos['observacion'][2]['escala'] = 3.5
    assert cp.Contrato(datos).huella() != a.huella()


def test_ida_y_vuelta_json(tmp_path):
    c = cp.contrato_provisional()
    ruta = tmp_path / 'modelo.json'
    ruta.write_text(json.dumps({'contrato': c.a_dict()}), encoding='utf-8')
    leido = cp.Contrato.desde_json(str(ruta))
    assert leido.huella() == c.huella()
    suelto = tmp_path / 'suelto.json'
    suelto.write_text(json.dumps(c.a_dict()), encoding='utf-8')
    assert cp.Contrato.desde_json(str(suelto)).huella() == c.huella()


# ---------------------------------------------------------------------- #
# Observación
# ---------------------------------------------------------------------- #
def test_escala_con_centro_y_sin_saturar():
    c = cp.Contrato(_minimo(observacion=[
        {'senal': 'imu/acel_z', 'op': 'escala', 'escala': 9.81, 'centro': 9.81},
        {'senal': 'track_fl/vel', 'op': 'escala', 'escala': 2.0, 'saturar': False},
    ]))
    obs = c.observacion(_estado())
    assert obs[0] == pytest.approx(0.0, abs=1e-6)
    assert obs[1] == pytest.approx(3.0)


def test_accion_previa_y_extra():
    c = cp.Contrato(_minimo(observacion=[
        {'senal': 'accion_previa/0', 'op': 'crudo'},
        {'senal': 'extra/contacto_fl/posicion', 'op': 'crudo'},
    ]))
    assert c.senales_extra == ['contacto_fl/posicion']
    e = _estado()
    e.accion_previa = [0.7]
    e.extra = {'contacto_fl/posicion': 0.25}
    assert c.observacion(e).tolist() == pytest.approx([0.7, 0.25])


def test_senal_extra_ausente_es_error_no_cero():
    c = cp.Contrato(_minimo(observacion=[{'senal': 'extra/contacto', 'op': 'crudo'}]))
    with pytest.raises(KeyError):
        c.observacion(_estado())


def test_senal_no_finita_es_error():
    c = cp.contrato_provisional()
    e = _estado()
    e.roll = float('nan')
    with pytest.raises(ValueError):
        c.observacion(e)


# ---------------------------------------------------------------------- #
# Acción
# ---------------------------------------------------------------------- #
def test_accion_provisional():
    c = cp.contrato_provisional()
    e = _estado()
    flippers, orugas = c.aplicar_accion([1, 0, -1, 0.5, 0.25, -0.5], e, 0.02)
    # flippers en velocidad: integrados sobre la posición MEDIDA
    assert flippers[0] == pytest.approx(0.1 + 1.2 * 0.02)
    assert flippers[1] == pytest.approx(-0.2)
    assert flippers[2] == pytest.approx(0.3 - 1.2 * 0.02)
    assert flippers[3] == pytest.approx(-0.4 + 0.5 * 1.2 * 0.02)
    # orugas por lado: izq = fl, rl; der = fr, rr
    assert orugas == pytest.approx([1.5, -3.0, 1.5, -3.0])


def test_accion_se_satura():
    c = cp.contrato_provisional()
    _, orugas = c.aplicar_accion([0, 0, 0, 0, 50, -50], _estado(), 0.02)
    assert orugas == pytest.approx([6.0, -6.0, 6.0, -6.0])


def test_accion_posicion_y_por_motor():
    c = cp.Contrato(_minimo(accion=[
        {'destino': 'flipper_rr', 'modo': 'posicion', 'escala': math.pi, 'centro': 0.5},
        {'destino': 'track_fr', 'modo': 'velocidad', 'escala': 4.0},
    ]))
    e = _estado()
    flippers, orugas = c.aplicar_accion([0.5, -1.0], e, 0.02)
    assert flippers[:3] == e.consigna_flipper[:3]   # los no comandados no cambian
    assert flippers[3] == pytest.approx(0.5 + math.pi / 2)
    assert orugas == [0.0, -4.0, 0.0, 0.0]          # las no comandadas quedan en 0


def test_accion_tamano_o_valor_invalido():
    c = cp.contrato_provisional()
    with pytest.raises(ValueError):
        c.aplicar_accion([0.0] * 4, _estado(), 0.02)
    with pytest.raises(ValueError):
        c.aplicar_accion([0.0] * 5 + [float('nan')], _estado(), 0.02)


# ---------------------------------------------------------------------- #
# Validación al cargar
# ---------------------------------------------------------------------- #
@pytest.mark.parametrize('cambio', [
    {'formato': 99},
    {'observacion': []},
    {'accion': []},
    {'observacion': [{'senal': 'flipper_xx/pos', 'op': 'crudo'}]},
    {'observacion': [{'senal': 'imu/yaw', 'op': 'crudo'}]},
    {'observacion': [{'senal': 'accion_previa/1', 'op': 'crudo'}]},
    {'observacion': [{'senal': 'flipper_fl/pos', 'op': 'log'}]},
    {'observacion': [{'senal': 'flipper_fl/pos', 'op': 'escala'}]},
    {'observacion': [{'senal': 'flipper_fl/pos', 'op': 'escala', 'escala': 0}]},
    {'accion': [{'destino': 'flipper_fl', 'modo': 'torque', 'escala': 1}]},
    {'accion': [{'destino': 'track_fl', 'modo': 'posicion', 'escala': 1}]},
    {'accion': [{'destino': 'brazo', 'modo': 'velocidad', 'escala': 1}]},
    {'accion': [{'destino': 'flipper_fl', 'modo': 'velocidad', 'escala': -1}]},
    {'frecuencia_hz': 0},
])
def test_contrato_invalido_se_rechaza(cambio):
    with pytest.raises(cp.ContratoInvalido):
        cp.Contrato(_minimo(**cambio))
