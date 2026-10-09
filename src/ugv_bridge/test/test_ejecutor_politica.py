"""Pruebas de los frenos de la política. No necesitan ROS.

Las de ModeloOnnx generan un .onnx de verdad con scripts/modelo_falso.py y se
saltan si faltan los paquetes `onnx` u `onnxruntime`.

Correr desde src/ugv_bridge:  python3 -m pytest test/test_ejecutor_politica.py
"""
import json
import os
import sys

import numpy as np
import pytest

from ugv_bridge import contrato_politica as cp
from ugv_bridge.ejecutor_politica import EjecutorPolitica, ModeloOnnx

DT = 0.02


class ModeloFijo:
    """Devuelve siempre la misma acción, o lanza si se le pide."""

    def __init__(self, accion):
        self.accion = accion
        self.llamadas = 0

    def inferir(self, obs):
        self.llamadas += 1
        if isinstance(self.accion, Exception):
            raise self.accion
        return np.asarray(self.accion, dtype=np.float64)


def _ejecutor(accion=(0, 0, 0, 0, 0.5, 0.5), **kw):
    ej = EjecutorPolitica(cp.contrato_provisional(), ModeloFijo(accion), DT, **kw)
    ej.estado.pos_flipper = [0.1, 0.2, 0.3, 0.4]
    ej.estado_actualizado(t=0.0)
    return ej


def test_sin_estado_no_publica_nada():
    ej = EjecutorPolitica(cp.contrato_provisional(), ModeloFijo([0] * 6), DT)
    s = ej.paso(0.0)
    assert s.observacion is None and s.flippers is None and s.orugas is None


def test_arranca_desactivado_pero_observa():
    ej = _ejecutor()
    s = ej.paso(0.0)
    assert s.observacion is not None and len(s.observacion) == 40
    assert s.flippers is None and s.orugas is None
    assert ej.modelo.llamadas == 0


def test_consigna_inicial_es_la_pose_medida():
    ej = _ejecutor()
    assert ej.estado.consigna_flipper == [0.1, 0.2, 0.3, 0.4]


def test_sin_modelo_no_se_puede_activar():
    ej = EjecutorPolitica(cp.contrato_provisional(), None, DT)
    assert ej.activar(True) is False
    assert not ej.comandando


def test_activo_comanda_orugas_y_flippers():
    ej = _ejecutor()
    ej.activar(True)
    s = ej.paso(0.01)
    assert s.orugas == pytest.approx([3.0] * 4)
    assert s.flippers == pytest.approx([0.1, 0.2, 0.3, 0.4])
    assert ej.estado.accion_previa == pytest.approx([0, 0, 0, 0, 0.5, 0.5])


def test_desactivar_manda_parada_una_vez():
    ej = _ejecutor()
    ej.activar(True)
    ej.paso(0.01)
    ej.activar(False)
    s = ej.paso(0.02)
    assert s.orugas == [0.0] * 4
    assert s.flippers == [0.1, 0.2, 0.3, 0.4]     # quietos donde están
    s = ej.paso(0.03)
    assert s.orugas is None and s.flippers is None


def test_watchdog_para_y_retoma():
    ej = _ejecutor(timeout_estado=0.5)
    ej.activar(True)
    assert ej.paso(0.1).orugas == pytest.approx([3.0] * 4)
    # El estado deja de llegar: una parada y luego silencio.
    s = ej.paso(1.0)
    assert s.orugas == [0.0] * 4
    assert ej.paso(1.1).orugas is None
    assert ej.modelo.llamadas == 1
    # Vuelve el estado: retoma.
    ej.estado_actualizado(t=1.2)
    assert ej.paso(1.2).orugas == pytest.approx([3.0] * 4)


@pytest.mark.parametrize('accion', [
    RuntimeError('onnx reventó'),
    [0, 0, 0, 0, float('nan'), 0],
    [0, 0, 0],
])
def test_red_que_falla_para_y_desactiva(accion):
    ej = _ejecutor(accion=accion)
    ej.activar(True)
    s = ej.paso(0.01)
    assert s.orugas == [0.0] * 4
    assert not ej.activa
    assert ej.error
    assert ej.paso(0.02).orugas is None      # no se reactiva solo


def test_tope_duro_de_orugas_gana_al_contrato():
    ej = _ejecutor(accion=[0, 0, 0, 0, 1, -1], vel_max_oruga=2.0)
    ej.activar(True)
    assert ej.paso(0.01).orugas == pytest.approx([2.0, -2.0, 2.0, -2.0])


def test_tope_duro_de_flippers_y_limites():
    datos = cp.contrato_provisional().a_dict()
    datos['accion'][0] = {'destino': 'flipper_fl', 'modo': 'posicion', 'escala': 3.0}
    ej = EjecutorPolitica(cp.Contrato(datos), ModeloFijo([1, 1, -1, 0, 0, 0]), DT,
                          vel_max_flipper=1.0, limites_flipper=(-0.5, 0.21))
    ej.estado.pos_flipper = [0.1, 0.2, 0.3, 0.4]
    ej.estado_actualizado(0.0)
    ej.activar(True)
    s = ej.paso(0.01)
    # fl pide 3 rad absolutos: avanza solo 1.0 rad/s * DT desde la pose medida
    assert s.flippers[0] == pytest.approx(0.1 + 1.0 * DT)
    # fr pide +1.2 rad/s, el tope duro lo deja en 1.0 y el límite en 0.21
    assert s.flippers[1] == pytest.approx(0.21)
    # rl baja a 1.0 rad/s; rr no se mueve, pero el límite superior lo recorta
    assert s.flippers[2] == pytest.approx(0.21)
    assert s.flippers[3] == pytest.approx(0.21)


def test_consigna_externa_solo_si_no_comanda():
    ej = _ejecutor()
    ej.consigna_externa(flippers=[1, 1, 1, 1], orugas=[2, 2, 2, 2])
    assert ej.estado.consigna_flipper == [1, 1, 1, 1]
    assert ej.estado.consigna_oruga == [2, 2, 2, 2]
    ej.activar(True)
    ej.consigna_externa(flippers=[9, 9, 9, 9], orugas=[9, 9, 9, 9])
    assert ej.estado.consigna_flipper == [0.1, 0.2, 0.3, 0.4]


def test_observacion_incompleta_frena():
    datos = cp.contrato_provisional().a_dict()
    datos['observacion'].append({'senal': 'extra/contacto_fl', 'op': 'crudo'})
    ej = EjecutorPolitica(cp.Contrato(datos), ModeloFijo([0, 0, 0, 0, 1, 1]), DT)
    ej.estado_actualizado(0.0)
    ej.activar(True)
    s = ej.paso(0.01)
    assert s.observacion is None
    assert s.orugas == [0.0] * 4
    assert 'contacto_fl' in ej.error


# ---------------------------------------------------------------------- #
# ModeloOnnx con un .onnx real
# ---------------------------------------------------------------------- #
@pytest.fixture
def falso():
    pytest.importorskip('onnx')
    pytest.importorskip('onnxruntime')
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
    import modelo_falso
    return modelo_falso


def test_onnx_ida_y_vuelta(falso, tmp_path):
    c = cp.contrato_provisional()
    ruta = str(tmp_path / 'm.onnx')
    w, b = falso.generar(c, ruta, ganancia=0.2, semilla=3)
    falso.escribir_contrato(c, str(tmp_path / 'm.json'), 'prueba')

    m = ModeloOnnx(ruta)
    assert m.contrato.huella() == c.huella()
    obs = np.linspace(-1, 1, c.obs_dim).astype(np.float32)
    esperado = np.tanh(obs @ w + b)
    assert m.inferir(obs) == pytest.approx(esperado, abs=1e-5)


def test_onnx_sesgo_da_accion_constante(falso, tmp_path):
    c = cp.contrato_provisional()
    ruta = str(tmp_path / 'm.onnx')
    falso.generar(c, ruta, ganancia=0.0, sesgo=[0, 0, 0, 0, 0.3, -0.3])
    falso.escribir_contrato(c, str(tmp_path / 'm.json'), 'prueba')
    m = ModeloOnnx(ruta)
    accion = m.inferir(np.ones(c.obs_dim, dtype=np.float32))
    assert accion == pytest.approx([0, 0, 0, 0, 0.3, -0.3], abs=1e-4)


def test_onnx_sin_contrato_no_carga(falso, tmp_path):
    ruta = str(tmp_path / 'm.onnx')
    falso.generar(cp.contrato_provisional(), ruta)
    with pytest.raises(RuntimeError, match='Falta el contrato'):
        ModeloOnnx(ruta)


def test_onnx_que_no_cuadra_con_el_contrato_no_carga(falso, tmp_path):
    ruta = str(tmp_path / 'm.onnx')
    falso.generar(cp.contrato_provisional(), ruta)
    otro = cp.contrato_provisional().a_dict()
    otro['observacion'] = otro['observacion'][:39]
    (tmp_path / 'm.json').write_text(json.dumps({'contrato': otro}), encoding='utf-8')
    with pytest.raises(RuntimeError, match='espera 40'):
        ModeloOnnx(ruta)


def test_onnx_contrato_invalido_no_carga(falso, tmp_path):
    ruta = str(tmp_path / 'm.onnx')
    falso.generar(cp.contrato_provisional(), ruta)
    (tmp_path / 'm.json').write_text('{"contrato": {"formato": 99}}', encoding='utf-8')
    with pytest.raises(RuntimeError, match='Contrato inválido'):
        ModeloOnnx(ruta)


# ---------------------------------------------------------------------- #
# Reloj de la política y guion de giro
# ---------------------------------------------------------------------- #
def _contrato_reloj():
    return cp.Contrato({
        'formato': cp.FORMATO_CONTRATO,
        'observacion': [{'senal': 'reloj/t_activa', 'op': 'crudo'}],
        'accion': [{'destino': 'orugas_izq', 'modo': 'velocidad', 'escala': 1.0}],
    })


def test_reloj_cuenta_desde_la_activacion():
    ej = EjecutorPolitica(_contrato_reloj(), ModeloFijo([0.0]), DT)
    ej.estado_actualizado(10.0)
    assert ej.paso(10.0).observacion[0] == 0.0          # inactiva: reloj en 0
    ej.activar(True)
    assert ej.paso(10.5).observacion[0] == 0.0          # primer paso activo
    ej.estado_actualizado(12.0)
    assert ej.paso(12.0).observacion[0] == pytest.approx(1.5)
    ej.activar(False)
    assert ej.paso(12.02).observacion[0] == 0.0
    ej.activar(True)
    ej.estado_actualizado(20.0)
    assert ej.paso(20.0).observacion[0] == 0.0          # al reactivar, desde cero


def test_guion_giro_90_grados_cada_5_s(falso, tmp_path):
    radio, ancho = falso.leer_geometria()
    ruta = str(tmp_path / 'giro.onnx')
    falso.generar_giro(ruta, 90, periodo=5.0, duracion=1.5, radio=radio, ancho=ancho)
    falso.escribir_contrato(falso.contrato_giro(5.0), str(tmp_path / 'giro.json'), 'prueba')

    m = ModeloOnnx(ruta)
    ej = EjecutorPolitica(m.contrato, m, DT)
    ej.estado_actualizado(0.0)
    ej.activar(True)

    yaw, yaw_por_periodo, t = 0.0, [], 0.0
    for k in range(int(10.0 / DT)):
        t = k * DT
        ej.estado_actualizado(t)
        s = ej.paso(t)
        assert s.flippers == pytest.approx([0.0] * 4)   # los flippers no se tocan
        izq = (s.orugas[0] + s.orugas[2]) / 2           # fl, rl
        der = (s.orugas[1] + s.orugas[3]) / 2           # fr, rr
        assert izq == pytest.approx(-der)               # en el sitio, sin avanzar
        yaw += radio * (der - izq) / ancho * DT         # = track_odometry_node
        if (k + 1) % int(5.0 / DT) == 0:
            yaw_por_periodo.append(yaw)
            yaw = 0.0
    for giro in yaw_por_periodo:
        assert np.degrees(giro) == pytest.approx(90.0, rel=0.03)
    assert len(yaw_por_periodo) == 2


def test_guion_giro_imposible_se_rechaza(falso, tmp_path):
    with pytest.raises(ValueError, match='alarga --duracion'):
        falso.generar_giro(str(tmp_path / 'g.onnx'), 720, 5.0, 0.5, 0.05, 0.30)
    with pytest.raises(ValueError, match='menor que --periodo'):
        falso.generar_giro(str(tmp_path / 'g.onnx'), 90, 5.0, 6.0, 0.05, 0.30)
