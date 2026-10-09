#!/usr/bin/env python3
r"""modelo_falso.py — Genera un .onnx de mentira para probar la cadena completa.

Mientras no llegue el modelo entrenado, sirve para probar todo lo que hay entre
la red y los motores: carga y verificación del contrato, observación, frenos,
/cmd_flippers, /cmd_tracks, driver y emulador. Cuando llegue el modelo real,
solo cambia el archivo.

La red es una sola capa: accion = tanh(W · obs + sesgo). Con `--ganancia 0` y
un `--sesgo` se obtiene una acción CONSTANTE, que es lo más útil para probar:

    # Avanzar despacio, flippers quietos (contrato provisional: 4 flippers + 2 lados)
    python3 modelo_falso.py -s modelos/falso --ganancia 0 --sesgo 0 0 0 0 0.3 0.3

    # Pesos aleatorios pequeños: se mueve "raro" pero acotado
    python3 modelo_falso.py -s modelos/falso

    # Con el contrato que entregue el profesor
    python3 modelo_falso.py -s modelos/falso --contrato su_contrato.json

GUION DE GIRO (`--giro GRADOS`): gira GRADOS en el sitio cada `--periodo`
segundos, tardando `--duracion` en cada giro, y se queda quieto el resto. Usa
su PROPIO contrato mínimo: 1 entrada (reloj/t_activa) y 2 salidas (orugas_izq,
orugas_der); los flippers no se tocan. Es a lazo abierto: la velocidad sale de
radio_oruga y ancho_orugas de config/geometria_robot.yaml, así que en
simulación la odometría marca el ángulo pedido y en el robot real, por el
patinaje, algo menos.

    # 90 grados a la izquierda cada 5 s (negativo = a la derecha)
    python3 modelo_falso.py -s modelos/giro --giro 90 --periodo 5 --duracion 1.5

Produce `<salida>.onnx` y `<salida>.json` ({"contrato": ...}), la misma pareja
que exportar_onnx.py. Necesita los paquetes `onnx` y `numpy` (y `yaml` para
--giro), no ROS. Dentro del contenedor el modelo tiene que estar en el repo
(modelos/), no en /tmp del host:

    ros2 launch ugv_bridge can_sim.launch.py use_foxglove:=true \
        use_politica:=true modelo_politica:=/home/ubuntu/TESIS/modelos/falso.onnx
    # y activar desde Foxglove: /politica/activa -> {"data": true}
"""
import argparse
import json
import os
import sys

import numpy as np

# El contrato se importa del paquete si está instalado y, si no (uso suelto
# sin workspace ROS), del directorio de al lado.
try:
    from ugv_bridge import contrato_politica
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from ugv_bridge import contrato_politica


def generar(contrato, ruta_onnx, ganancia=0.1, sesgo=None, semilla=0):
    """Escribe el .onnx. Devuelve (W, b) para poder verificar la salida."""
    import onnx
    from onnx import helper, numpy_helper, TensorProto

    obs_dim, acc_dim = contrato.obs_dim, contrato.acc_dim
    rng = np.random.default_rng(semilla)
    w = (rng.standard_normal((obs_dim, acc_dim)) * ganancia).astype(np.float32)
    if sesgo is None:
        b = np.zeros(acc_dim, dtype=np.float32)
    else:
        if len(sesgo) != acc_dim:
            raise ValueError(f'--sesgo tiene {len(sesgo)} valores y el contrato '
                             f'pide {acc_dim} acciones')
        # tanh(atanh(x)) = x: el sesgo se da en unidades de acción, [-1, 1].
        b = np.arctanh(np.clip(np.asarray(sesgo, dtype=np.float64), -0.999, 0.999))
        b = b.astype(np.float32)

    grafo = helper.make_graph(
        nodes=[
            helper.make_node('Gemm', ['observacion', 'W', 'b'], ['z']),
            helper.make_node('Tanh', ['z'], ['accion']),
        ],
        name='politica_falsa',
        inputs=[helper.make_tensor_value_info(
            'observacion', TensorProto.FLOAT, ['lote', obs_dim])],
        outputs=[helper.make_tensor_value_info(
            'accion', TensorProto.FLOAT, ['lote', acc_dim])],
        initializer=[numpy_helper.from_array(w, 'W'), numpy_helper.from_array(b, 'b')],
    )
    # opset 17 e IR 8: lo que carga un onnxruntime de hace un par de años, que
    # es lo que suele haber en la Raspberry.
    modelo = helper.make_model(
        grafo, opset_imports=[helper.make_opsetid('', 17)], ir_version=8,
        producer_name='ugv_bridge/modelo_falso')
    onnx.checker.check_model(modelo)
    onnx.save(modelo, ruta_onnx)
    return w, b


RUTA_GEOMETRIA = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', 'config', 'geometria_robot.yaml')

# Escala de las orugas en el contrato del giro: acción 1.0 = 6 rad/s, el mismo
# tope duro por defecto de politica_flippers (vel_max_oruga).
ESCALA_ORUGA_GIRO = 6.0


def leer_geometria(ruta=RUTA_GEOMETRIA):
    """Devuelve (radio_oruga, ancho_orugas) del YAML de geometría."""
    import yaml

    def buscar(nodo, clave):
        if isinstance(nodo, dict):
            if clave in nodo:
                return float(nodo[clave])
            for v in nodo.values():
                hallado = buscar(v, clave)
                if hallado is not None:
                    return hallado
        return None

    with open(ruta, encoding='utf-8') as f:
        datos = yaml.safe_load(f)
    r, ancho = buscar(datos, 'radio_oruga'), buscar(datos, 'ancho_orugas')
    if r is None or ancho is None:
        raise ValueError(f'{ruta} no tiene radio_oruga y ancho_orugas')
    return r, ancho


def contrato_giro(periodo):
    """Contrato mínimo del guion: reloj -> velocidad de cada lado de orugas."""
    return contrato_politica.Contrato({
        'formato': contrato_politica.FORMATO_CONTRATO,
        'nombre': 'guion-giro',
        'version': 1,
        'frecuencia_hz': 50,
        'observacion': [{'senal': 'reloj/t_activa', 'op': 'crudo'}],
        'accion': [
            {'destino': 'orugas_izq', 'modo': 'velocidad', 'escala': ESCALA_ORUGA_GIRO},
            {'destino': 'orugas_der', 'modo': 'velocidad', 'escala': ESCALA_ORUGA_GIRO},
        ],
        'guion': {'periodo_s': periodo},
    })


def velocidad_giro(grados, duracion, radio, ancho):
    """Velocidad de rueda (rad/s) para girar `grados` en `duracion` s, en el sitio.

    Skid-steer: w = (v_der - v_izq) / ancho, con v = radio * velocidad de rueda.
    Con v_der = -v_izq = radio * u queda w = 2 * radio * u / ancho.
    """
    w = np.radians(grados) / duracion
    return w * ancho / (2.0 * radio)


def generar_giro(ruta_onnx, grados, periodo, duracion, radio, ancho):
    """Escribe el .onnx del guion. Devuelve la acción normalizada mientras gira."""
    import onnx
    from onnx import helper, numpy_helper, TensorProto

    if not 0 < duracion < periodo:
        raise ValueError('--duracion tiene que ser positiva y menor que --periodo')
    u = velocidad_giro(grados, duracion, radio, ancho)
    a = u / ESCALA_ORUGA_GIRO
    if abs(a) > 1.0:
        raise ValueError(
            f'girar {grados} grados en {duracion} s pide {abs(u):.2f} rad/s y el '
            f'tope es {ESCALA_ORUGA_GIRO}: alarga --duracion')

    def const(nombre, valor):
        return numpy_helper.from_array(np.asarray(valor, dtype=np.float32), nombre)

    # fase = t mod periodo; girando = fase < duracion; accion = [-a, a] o [0, 0]
    grafo = helper.make_graph(
        nodes=[
            helper.make_node('Mod', ['observacion', 'periodo'], ['fase'], fmod=1),
            helper.make_node('Less', ['fase', 'duracion'], ['girando']),
            helper.make_node('Where', ['girando', 'giro', 'quieto'], ['accion']),
        ],
        name='guion_giro',
        inputs=[helper.make_tensor_value_info('observacion', TensorProto.FLOAT, ['lote', 1])],
        outputs=[helper.make_tensor_value_info('accion', TensorProto.FLOAT, ['lote', 2])],
        initializer=[
            const('periodo', [periodo]),
            const('duracion', [duracion]),
            const('giro', [[-a, a]]),
            const('quieto', [[0.0, 0.0]]),
        ],
    )
    modelo = helper.make_model(
        grafo, opset_imports=[helper.make_opsetid('', 17)], ir_version=8,
        producer_name='ugv_bridge/modelo_falso --giro')
    onnx.checker.check_model(modelo)
    onnx.save(modelo, ruta_onnx)
    return a


def escribir_contrato(contrato, ruta_json, origen):
    with open(ruta_json, 'w', encoding='utf-8') as f:
        json.dump({'contrato': contrato.a_dict(), 'modelo_origen': origen},
                  f, indent=2, ensure_ascii=False)


def main():
    p = argparse.ArgumentParser(description='Genera un modelo ONNX falso con su contrato')
    p.add_argument('-s', '--salida', default='politica_falsa',
                   help='prefijo de salida (default: politica_falsa)')
    p.add_argument('--contrato', help='contrato JSON (default: el provisional)')
    p.add_argument('--ganancia', type=float, default=0.1,
                   help='desviación de los pesos aleatorios; 0 = acción constante')
    p.add_argument('--sesgo', type=float, nargs='+',
                   help='acción con observación en cero, un valor en [-1, 1] por acción')
    p.add_argument('--semilla', type=int, default=0)
    p.add_argument('--giro', type=float, metavar='GRADOS',
                   help='guion: girar GRADOS en el sitio cada --periodo s (+ = izquierda)')
    p.add_argument('--periodo', type=float, default=5.0, help='s entre giros (default 5)')
    p.add_argument('--duracion', type=float, default=1.5,
                   help='s que dura cada giro (default 1.5)')
    args = p.parse_args()

    ruta_onnx = f'{args.salida}.onnx'
    ruta_json = f'{args.salida}.json'
    if args.giro is not None:
        contrato = contrato_giro(args.periodo)
        radio, ancho = leer_geometria()
        try:
            a = generar_giro(ruta_onnx, args.giro, args.periodo, args.duracion,
                             radio, ancho)
        except ValueError as e:
            sys.exit(str(e))
        escribir_contrato(contrato, ruta_json, 'modelo_falso.py --giro')
        print(f'  -> {ruta_onnx}  (giro de {args.giro:g} grados cada {args.periodo:g} s, '
              f'{args.duracion:g} s girando a {a * ESCALA_ORUGA_GIRO:+.2f} rad/s '
              f'en el lado derecho)')
        print(f'  -> {ruta_json}  (contrato {contrato.nombre}, huella {contrato.huella()})')
        return

    if args.contrato:
        contrato = contrato_politica.Contrato.desde_json(args.contrato)
    else:
        contrato = contrato_politica.contrato_provisional()

    try:
        generar(contrato, ruta_onnx, args.ganancia, args.sesgo, args.semilla)
    except ValueError as e:
        sys.exit(str(e))
    escribir_contrato(contrato, ruta_json, 'modelo_falso.py')

    print(f'  -> {ruta_onnx}  ({contrato.obs_dim} entradas -> {contrato.acc_dim} salidas)')
    print(f'  -> {ruta_json}  (contrato {contrato.nombre}, huella {contrato.huella()})')


if __name__ == '__main__':
    main()
