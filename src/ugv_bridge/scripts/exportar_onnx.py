#!/usr/bin/env python3
"""exportar_onnx.py — PC: convierte el modelo entrenado en algo que la Pi pueda correr.

ESTE SCRIPT NO SE EJECUTA EN LA RASPBERRY. Corre en el PC de entrenamiento, con
torch y stable-baselines3 instalados. Produce dos archivos que SÍ van a la Pi:

    politica.onnx    la red, sin PyTorch alrededor
    politica.json    el contrato con el que se entrenó

Y nada más. La Pi no necesita gymnasium, ni stable-baselines3, ni torch: solo
`onnxruntime`. Esa es toda la gracia de exportar.

ES UN EJEMPLO PARA STABLE-BASELINES3 (PPO). La política la entrena otra
persona y puede usar otra librería; lo que importa es el RESULTADO: un .onnx
con una entrada (observación) y una salida (acción), más el .json con el
contrato. Para PyTorch puro basta `torch.onnx.export` y escribir el .json con
`contrato.a_dict()`; para probar sin modelo, scripts/modelo_falso.py.

POR QUÉ EL .json IMPORTA TANTO COMO EL .onnx
--------------------------------------------
Un .onnx solo sabe que recibe N números y devuelve M. No sabe QUÉ significa
cada uno. Si el robot arma la observación en otro orden o con otra escala que
el simulador, la red seguirá funcionando sin dar un solo error: recibirá el par
de un flipper en la casilla donde esperaba una velocidad y devolverá comandos
plausibles y equivocados.

Por eso el .json lleva el CONTRATO COMPLETO (ver contrato_politica.py) y el
robot arma la observación a partir de él: no hay una segunda copia que se
pueda desincronizar. El nodo se niega a cargar un modelo sin .json, o cuya red
no mida lo que dice el contrato.

USO
---
    python3 exportar_onnx.py modelo_flippers.zip -s politica
    python3 exportar_onnx.py modelo_flippers.zip -s politica --verificar
    python3 exportar_onnx.py modelo_flippers.zip -s politica --contrato c.json

Después, a la Pi:
    scp politica.onnx politica.json ros2@robotdeteccion.local:~/

NOTA SOBRE LA POLÍTICA EXPORTADA
--------------------------------
Se exporta la política DETERMINISTA (la media de la distribución), no una
muestra. En entrenamiento conviene explorar; en el robot no: un muestreo
aleatorio sobre ocho motores de 48 V es ruido que se convierte en movimiento.
"""
import argparse
import json
import os
import sys

# El contrato es la fuente de verdad. Se importa del paquete si está instalado
# y, si no (uso suelto en el PC sin workspace ROS), del archivo de al lado.
try:
    from ugv_bridge import contrato_politica as contrato
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from ugv_bridge import contrato_politica as contrato


class _PoliticaDeterminista:
    """Envoltorio torch que expone solo observación -> acción media."""

    def __new__(cls, politica):
        import torch

        class _Modulo(torch.nn.Module):
            def __init__(self, pol):
                super().__init__()
                self.pol = pol

            def forward(self, obs):
                # SB3 >= 2.0: la política expone _predict(), que con
                # deterministic=True devuelve la media de la distribución.
                return self.pol._predict(obs, deterministic=True)

        return _Modulo(politica)


def exportar(ruta_modelo, salida, c):
    try:
        import torch
        from stable_baselines3 import PPO
    except ImportError as e:
        sys.exit(f'Faltan torch y/o stable-baselines3 en este PC: {e}')

    print(f'Cargando {ruta_modelo}...')
    modelo = PPO.load(ruta_modelo, device='cpu')

    envoltorio = _PoliticaDeterminista(modelo.policy)
    envoltorio.eval()

    ruta_onnx = f'{salida}.onnx'
    ejemplo = torch.zeros(1, c.obs_dim, dtype=torch.float32)
    torch.onnx.export(
        envoltorio, ejemplo, ruta_onnx,
        input_names=['observacion'], output_names=['accion'],
        # Lote dinámico: el nodo infiere de uno en uno, pero así el mismo
        # archivo sirve para evaluar lotes en el PC.
        dynamic_axes={'observacion': {0: 'lote'}, 'accion': {0: 'lote'}},
        opset_version=17,
    )
    print(f'  -> {ruta_onnx}')

    ruta_json = f'{salida}.json'
    meta = {'contrato': c.a_dict(), 'modelo_origen': os.path.basename(ruta_modelo)}
    with open(ruta_json, 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    print(f'  -> {ruta_json}  (contrato {c.nombre}, huella {c.huella()})')
    return ruta_onnx


def verificar(ruta_onnx, c):
    """Carga el .onnx como lo hará el robot y comprueba que entra y sale lo debido."""
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as e:
        sys.exit(f'Para --verificar hace falta onnxruntime: {e}')

    sesion = ort.InferenceSession(ruta_onnx, providers=['CPUExecutionProvider'])
    entrada = sesion.get_inputs()[0]
    dim = entrada.shape[-1]
    if isinstance(dim, int) and dim != c.obs_dim:
        sys.exit(f'FALLO: la red espera {dim} y el contrato da {c.obs_dim}')

    obs = np.zeros((1, c.obs_dim), dtype=np.float32)
    salida = sesion.run(None, {entrada.name: obs})[0].ravel()
    if salida.shape[0] != c.acc_dim:
        sys.exit(f'FALLO: la red devuelve {salida.shape[0]} valores y el contrato '
                 f'describe {c.acc_dim}')

    print(f'Verificado: entrada {c.obs_dim} -> salida {c.acc_dim}.')
    print(f'  acción con observación en cero: {[round(float(v), 4) for v in salida]}')


def main():
    p = argparse.ArgumentParser(
        description='Exporta a ONNX el modelo entrenado, con su contrato')
    p.add_argument('modelo', help='archivo .zip de stable-baselines3')
    p.add_argument('-s', '--salida', default='politica',
                   help='prefijo de salida (default: politica)')
    p.add_argument('--verificar', action='store_true',
                   help='cargar el .onnx exportado y comprobar formas')
    p.add_argument('--contrato',
                   help='contrato JSON con el que se entrenó (default: el provisional)')
    args = p.parse_args()

    if args.contrato:
        c = contrato.Contrato.desde_json(args.contrato)
    else:
        c = contrato.contrato_provisional()
    ruta_onnx = exportar(args.modelo, args.salida, c)
    if args.verificar:
        verificar(ruta_onnx, c)


if __name__ == '__main__':
    main()
