#!/usr/bin/env python3
"""ejecutor_politica.py — Carga el modelo y lo ejecuta con frenos. Sin ROS.

Separa lo que CAMBIA con cada modelo (el contrato, en `contrato_politica.py`)
de lo que NO cambia: cómo se carga un modelo, cuándo se le deja comandar y qué
se hace cuando algo falla. `politica_flippers.py` es solo el pegamento ROS
alrededor de `EjecutorPolitica`, y por eso esta lógica se puede probar sin ROS
(test/test_ejecutor_politica.py).

FRENOS (todos activos a la vez)
-------------------------------
  1. ARRANCA DESACTIVADO. No sale un comando hasta que se activa.
  2. CONTRATO VERIFICADO AL CARGAR. El modelo trae su contrato en el .json; si
     la red no mide lo que dice el contrato, no se carga.
  3. WATCHDOG DE ESTADO. Si el estado del robot deja de llegar, no se comanda
     sobre una foto vieja.
  4. TOPES DUROS DEL ROBOT, independientes del contrato: velocidad máxima de
     orugas y de flippers, y ángulos opcionales. Un contrato mal escrito puede
     pedir lo que quiera; lo que sale al bus no pasa de aquí.
  5. PARADA SEGURA. Al desactivarse, al saltar el watchdog o si la red
     devuelve basura, se manda UNA VEZ: orugas a 0 y flippers quietos donde
     están. Es imprescindible porque flipper_node retiene el último comando:
     sin esto, las orugas seguirían girando a la última velocidad pedida.

Si la red falla (excepción, NaN, tamaño equivocado), además de parar se
desactiva: volver a activar es una decisión del operador, no automática.
"""
import json
import os

import numpy as np

from ugv_bridge import contrato_politica


class ModeloOnnx:
    """Un .onnx con su contrato, verificados uno contra el otro."""

    def __init__(self, ruta, ruta_contrato=None):
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise RuntimeError(
                'Falta onnxruntime en el Python que corre ROS (en el PC, la Pi o el '
                'contenedor). En Ubuntu 24.04: pip3 install --user '
                '--break-system-packages onnxruntime "numpy<2"') from e

        if not os.path.isfile(ruta):
            raise RuntimeError(f'No existe el modelo: {ruta}')

        # El contrato es OBLIGATORIO: sin él, nadie sabe qué significa cada
        # número que entra y sale de la red.
        ruta_contrato = ruta_contrato or os.path.splitext(ruta)[0] + '.json'
        if not os.path.isfile(ruta_contrato):
            raise RuntimeError(
                f'Falta el contrato del modelo ({ruta_contrato}). Sin él no se '
                f'sabe qué entra ni qué sale de la red. Ver contrato_politica.py.')
        try:
            self.contrato = contrato_politica.Contrato.desde_json(ruta_contrato)
        except (contrato_politica.ContratoInvalido, json.JSONDecodeError) as e:
            raise RuntimeError(f'Contrato inválido en {ruta_contrato}: {e}') from e

        # Un hilo: la Pi tiene que repartir CPU con el bucle de control a 100 Hz.
        opciones = ort.SessionOptions()
        opciones.intra_op_num_threads = 1
        opciones.inter_op_num_threads = 1
        self.sesion = ort.InferenceSession(
            ruta, sess_options=opciones, providers=['CPUExecutionProvider'])

        entrada = self.sesion.get_inputs()[0]
        self.nombre_entrada = entrada.name
        dim = entrada.shape[-1]
        if isinstance(dim, int) and dim != self.contrato.obs_dim:
            raise RuntimeError(
                f'La red espera {dim} valores de entrada y el contrato produce '
                f'{self.contrato.obs_dim}. No se carga.')
        salida = self.sesion.get_outputs()[0]
        self.nombre_salida = salida.name
        dim = salida.shape[-1]
        if isinstance(dim, int) and dim != self.contrato.acc_dim:
            raise RuntimeError(
                f'La red devuelve {dim} valores y el contrato describe '
                f'{self.contrato.acc_dim} acciones. No se carga.')
        self.ruta = ruta

    def inferir(self, obs):
        salida = self.sesion.run(
            [self.nombre_salida],
            {self.nombre_entrada: obs.reshape(1, -1).astype(np.float32)})[0]
        return np.asarray(salida, dtype=np.float64).ravel()


class Salida:
    """Lo que el nodo tiene que publicar tras un paso. None = no publicar."""

    def __init__(self, observacion=None, flippers=None, orugas=None):
        self.observacion = observacion
        self.flippers = flippers
        self.orugas = orugas


class EjecutorPolitica:
    """La política con sus frenos. El nodo le da el estado y publica lo que devuelve.

    Parameters
    ----------
    contrato : contrato_politica.Contrato que define la observación. Si hay
        modelo, tiene que ser el del modelo.
    modelo : objeto con `inferir(obs) -> acción`, o None para solo observar.
    dt : paso de control en segundos.
    timeout_estado : segundos sin estado tras los que el watchdog frena.
    vel_max_oruga, vel_max_flipper : topes duros en rad/s.
    limites_flipper : (min_rad, max_rad) o None.
    """

    def __init__(self, contrato, modelo, dt, timeout_estado=0.5,
                 vel_max_oruga=6.0, vel_max_flipper=1.5, limites_flipper=None):
        self.contrato = contrato
        self.modelo = modelo
        self.dt = float(dt)
        self.timeout = float(timeout_estado)
        self.vel_max_oruga = float(vel_max_oruga)
        self.vel_max_flipper = float(vel_max_flipper)
        self.limites = limites_flipper

        self.estado = contrato_politica.EstadoRobot()
        self.estado.accion_previa = [0.0] * contrato.acc_dim
        self.activa = False
        self.t_estado = None
        self.consigna_sembrada = False
        self.parada_pendiente = False
        self.frenando = False
        self.t_activacion = None
        self.error = None
        self.ultimo_error_obs = None
        self.ciclos_publicados = 0

    # ------------------------------------------------------------------ #
    # Entradas
    # ------------------------------------------------------------------ #
    def estado_actualizado(self, t):
        """El nodo llama esto tras volcar /joint_states en `self.estado`."""
        # La primera consigna es la pose ACTUAL, no cero. Si se sembrara en cero,
        # el primer ciclo activo llevaría los flippers al cero mecánico de golpe.
        if not self.consigna_sembrada:
            self.estado.consigna_flipper = list(self.estado.pos_flipper)
            self.consigna_sembrada = True
        self.t_estado = float(t)

    def consigna_externa(self, flippers=None, orugas=None):
        """Comandos que publicó otro (Teleop, Foxglove) o el eco de los propios.

        Mientras la política comanda, se ignoran: lo que llega es su propio eco.
        Si no comanda, la consigna es la de quien sí lo hace; sin esto el error
        de seguimiento se mediría contra una foto vieja y la señal táctil de
        /politica/observacion no serviría justo cuando se conduce a mano.
        """
        if self.comandando:
            return
        if flippers is not None and len(flippers) >= 4:
            self.estado.consigna_flipper = [float(v) for v in flippers[:4]]
            self.consigna_sembrada = True
        if orugas is not None and len(orugas) >= 4:
            self.estado.consigna_oruga = [float(v) for v in orugas[:4]]

    def activar(self, activa):
        """Devuelve True si cambió el estado."""
        activa = bool(activa) and self.modelo is not None
        if activa == self.activa:
            return False
        self.activa = activa
        if activa:
            # Al activarse se parte de la pose real y de una acción previa nula:
            # si no, el primer comando continuaría lo que quedó de la vez anterior.
            self.estado.consigna_flipper = list(self.estado.pos_flipper)
            self.estado.accion_previa = [0.0] * self.contrato.acc_dim
            self.error = None
            self.frenando = False
        else:
            self.parada_pendiente = True
        # El reloj de la política (reloj/t_activa) arranca en el primer paso activo.
        self.t_activacion = None
        self.estado.t_activa = 0.0
        return True

    @property
    def comandando(self):
        return self.activa and self.modelo is not None

    def estado_fresco(self, t):
        return self.t_estado is not None and (float(t) - self.t_estado) <= self.timeout

    # ------------------------------------------------------------------ #
    # Ciclo
    # ------------------------------------------------------------------ #
    def paso(self, t):
        """Un ciclo de control. Devuelve qué publicar."""
        salida = Salida()
        if self.t_estado is None:
            return salida

        if self.comandando:
            if self.t_activacion is None:
                self.t_activacion = float(t)
            self.estado.t_activa = float(t) - self.t_activacion

        obs = None
        try:
            obs = self.contrato.observacion(self.estado)
            salida.observacion = obs
            self.ultimo_error_obs = None
        except (KeyError, ValueError) as e:
            self.ultimo_error_obs = str(e)

        if self.parada_pendiente:
            self.parada_pendiente = False
            self._parar(salida)
            return salida
        if not self.comandando:
            return salida

        if not self.estado_fresco(t):
            if not self.frenando:
                self.frenando = True
                self._parar(salida)
            return salida
        if obs is None:
            self._fallar(salida, f'observación incompleta: {self.ultimo_error_obs}')
            return salida
        self.frenando = False

        try:
            accion = np.asarray(self.modelo.inferir(obs), dtype=np.float64).ravel()
            flippers, orugas = self.contrato.aplicar_accion(accion, self.estado, self.dt)
        except Exception as e:  # noqa: B902 - cualquier fallo de la red frena
            self._fallar(salida, f'la red falló: {e}')
            return salida

        salida.flippers = self._acotar_flippers(flippers)
        salida.orugas = self._acotar_orugas(orugas)
        self.estado.consigna_flipper = list(salida.flippers)
        self.estado.consigna_oruga = list(salida.orugas)
        self.estado.accion_previa = [float(v) for v in np.clip(accion, -1.0, 1.0)]
        self.ciclos_publicados += 1
        return salida

    # ------------------------------------------------------------------ #
    def _acotar_flippers(self, objetivo):
        """Tope duro de velocidad (paso por ciclo respecto a la pose real) y de ángulo."""
        paso_max = self.vel_max_flipper * self.dt
        acotado = []
        for i, v in enumerate(objetivo):
            pos = float(self.estado.pos_flipper[i])
            consigna = float(self.estado.consigna_flipper[i])
            # Un flipper que el contrato no mueve conserva su consigna intacta.
            if v != consigna:
                v = min(max(v, pos - paso_max), pos + paso_max)
            if self.limites is not None:
                v = min(max(v, self.limites[0]), self.limites[1])
            acotado.append(float(v))
        return acotado

    def _acotar_orugas(self, orugas):
        m = self.vel_max_oruga
        return [float(min(max(v, -m), m)) for v in orugas]

    def _parar(self, salida):
        """Orugas a 0 y flippers quietos donde están medidos."""
        salida.orugas = [0.0] * len(contrato_politica.ORUGAS)
        salida.flippers = [float(v) for v in self.estado.pos_flipper]
        self.estado.consigna_oruga = list(salida.orugas)
        self.estado.consigna_flipper = list(salida.flippers)

    def _fallar(self, salida, motivo):
        self.error = motivo
        self.activa = False
        self.t_activacion = None
        self.estado.t_activa = 0.0
        self._parar(salida)


def huella_texto(c):
    """'nombre v2 (a1b2c3...)' para logs y diagnóstico."""
    v = '' if c.version is None else f' v{c.version}'
    return f'{c.nombre}{v} ({c.huella()})'
