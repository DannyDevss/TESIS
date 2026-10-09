#!/usr/bin/env python3
"""contrato_politica.py — Contrato entre el entrenamiento y la inferencia en el robot.

El contrato dice qué números entran a la red, en qué orden y con qué escala, y
qué significa cada número que sale. Si el entrenamiento y el robot no coinciden
bit a bit, el modelo produce basura sin dar ningún error: la red recibe valores
en posiciones que no esperaba y devuelve comandos plausibles pero equivocados.
Es el fallo sim-to-real más caro y el más difícil de diagnosticar, porque nada
se cae.

NO IMPORTA ROS. Son funciones puras sobre diccionarios y arrays, a propósito:
el mismo código sirve en el PC de entrenamiento, en las pruebas y en el nodo
ROS del robot.

---------------------------------------------------------------------------
EL CONTRATO ES UN DATO, NO CÓDIGO
---------------------------------------------------------------------------
La política la entrena otra persona, con su propio simulador y su propio
vector de observación, que todavía no conocemos. Por eso el contrato no está
fijo aquí: es un diccionario (un JSON en disco) que viaja JUNTO al modelo:

    politica.onnx    la red
    politica.json    {"contrato": {...}}   <- este formato

Adaptarse al modelo que entreguen es escribir ese JSON, no tocar el nodo. El
nodo, sus frenos de seguridad y el driver quedan iguales.

    {
      "formato": 1,
      "nombre": "provisional-ugv",
      "version": 2,
      "frecuencia_hz": 50,
      "observacion": [
        {"senal": "flipper_fl/pos", "op": "sin"},
        {"senal": "flipper_fl/vel", "op": "escala", "escala": 3.0},
        {"senal": "imu/acel_z", "op": "escala", "escala": 9.81, "centro": 9.81},
        {"senal": "accion_previa/0", "op": "crudo"},
        ...
      ],
      "accion": [
        {"destino": "flipper_fl", "modo": "velocidad", "escala": 1.2},
        {"destino": "orugas_izq", "modo": "velocidad", "escala": 6.0},
        ...
      ]
    }

OBSERVACIÓN: cada término toma una SEÑAL del catálogo y le aplica una `op`:

    crudo    el valor tal cual
    escala   (valor - centro) / escala, saturado a [-1, 1] salvo
             "saturar": false. `centro` vale 0 si se omite.
    sin/cos  seno o coseno del valor (para ángulos que dan la vuelta)

CATÁLOGO DE SEÑALES (unidades del eje de salida, SI, como /joint_states):

    flipper_<n>/pos /vel /esfuerzo /consigna /error      n = fl, fr, rl, rr
    track_<n>/vel /esfuerzo /consigna /desliz
    imu/roll  imu/pitch  imu/giro_x|y|z  imu/acel_x|y|z
    accion_previa/<i>    la acción i del ciclo anterior, ya saturada
    reloj/t_activa       segundos desde que se activó la política (0 si no
                         lo está). Para guiones de prueba (scripts/
                         modelo_falso.py --giro); una política de terreno
                         no debería depender de la hora.
    extra/<lo-que-sea>   sensores que todavía no existen (ver más abajo)

    error  = consigna - pos    señal táctil: el flipper no llega porque toca algo
    desliz = consigna - vel    la oruga patina: terreno malo

ACCIÓN: la red devuelve un número por término, en [-1, 1] (se satura igual):

    destino           modo        significado del valor a
    flipper_<n>       velocidad   a * escala rad/s, integrado sobre la posición
                                  MEDIDA (ver integrar_accion)
    flipper_<n>       posicion    centro + a * escala rad, ángulo absoluto
    track_<n>         velocidad   a * escala rad/s en esa oruga
    orugas_izq|der    velocidad   a * escala rad/s en las dos orugas del lado
                                  (izq = fl, rl; der = fr, rr)

Un flipper que el contrato no comanda conserva su consigna. Una oruga que no
comanda queda en 0.

SENSORES FUTUROS (`extra/`)
---------------------------
Se está pensando en sensores en los flippers que digan DÓNDE tocaron, como un
bastón de ciego. Cuando existan, el nodo los pondrá en `EstadoRobot.extra`
con un nombre (por ejemplo 'contacto_fl/posicion') y el contrato los pedirá
como 'extra/contacto_fl/posicion'. No hace falta cambiar este archivo.

PERCEPCIÓN TÁCTIL SIN SENSOR TÁCTIL
-----------------------------------
Mientras tanto, los motores YA son un sensor de contacto. Cuando un flipper se
apoya, crece el error de seguimiento (el lazo de posición no llega) y sube el
par (`effort`, derivado de Iq). Es la propiocepción con la que los cuadrúpedos
detectan el apoyo de las patas. El contrato provisional de abajo las incluye.

  ADVERTENCIA SOBRE EL PAR: el GIM6010-8 trae los mensajes de Iq APAGADOS de
  fábrica. Sin activarlos por USB (`odrv0.axis0.config.can.iq_rate_ms = 10`),
  `effort` llega en cero. Y KT_NM_POR_A = 1.0 en protocolo_can.py es un valor
  de relleno: mientras no se ponga el Kt real, `effort` está en una unidad
  arbitraria. A la red le da igual, PERO tiene que ser la misma unidad en el
  simulador y en el robot.

VERSIONES
---------
`formato` es la versión de ESTE esquema (lo que entiende el código). `version`
y `nombre` son del contrato concreto y solo se registran. La garantía de que
el contrato con que se entrenó y el que usa el robot son el mismo no viene de
un número, sino de que el contrato viaja dentro del .json del modelo, y de su
`huella()` (un hash), que se muestra en /diagnostics.
"""
import copy
import hashlib
import json
import math

import numpy as np

# Versión del esquema del contrato. Subirla solo si cambia lo que el código
# sabe interpretar (nuevas `op`, nuevos `modo`), no al cambiar un contrato.
FORMATO_CONTRATO = 1

# Orden canónico. Es el mismo de /cmd_flippers, /cmd_tracks y de los IDs de
# motor (1-4 orugas, 5-8 flippers) en todo el proyecto. No reordenar.
FLIPPERS = ('fl', 'fr', 'rl', 'rr')
ORUGAS = ('fl', 'fr', 'rl', 'rr')
JUNTAS_FLIPPER = tuple(f'flipper_{n}' for n in FLIPPERS)
JUNTAS_ORUGA = tuple(f'track_{n}' for n in ORUGAS)

# Orugas de cada lado, para los destinos `orugas_izq` / `orugas_der`. En todo
# el proyecto velocidad positiva = avanzar, en ambos lados.
LADOS = {
    'orugas_izq': ('fl', 'rl'),
    'orugas_der': ('fr', 'rr'),
}

OPS = ('crudo', 'escala', 'sin', 'cos')
MODOS_FLIPPER = ('velocidad', 'posicion')

_CAMPOS_FLIPPER = ('pos', 'vel', 'esfuerzo', 'consigna', 'error')
_CAMPOS_ORUGA = ('vel', 'esfuerzo', 'consigna', 'desliz')
_SENALES_IMU = ('roll', 'pitch', 'giro_x', 'giro_y', 'giro_z',
                'acel_x', 'acel_y', 'acel_z')

# ---------------------------------------------------------------------- #
# Escalas del contrato PROVISIONAL.
#
# No son límites duros: son el valor que mapea a 1.0. Se saturan con clip, así
# que pasarse no rompe nada, solo aplana.
#
# TODO(hardware): ajustar con medidas reales. Estos valores son estimaciones
# razonables para el GIM6010-8 con la reducción 8:1; en cuanto haya un flipper
# empujando contra el suelo con telemetría de Iq activada, mirar los rangos
# reales en Foxglove (/politica/observacion) y corregir.
# ---------------------------------------------------------------------- #
VEL_FLIPPER_MAX = 3.0        # rad/s en el eje de salida
ERROR_MAX = 0.35             # rad de error de seguimiento (~20 grados)
TORQUE_FLIPPER_MAX = 20.0    # Nm (unidad de `effort`; depende de KT_NM_POR_A)
VEL_ORUGA_MAX = 12.0         # rad/s
TORQUE_ORUGA_MAX = 20.0      # Nm
GIRO_MAX = 3.0               # rad/s
ACEL_MAX = 20.0              # m/s^2

# Velocidades que pide el contrato provisional con la acción saturada a 1.0.
# Con un paso de control a 50 Hz, el peor comando mueve un flipper
# 1.2/50 = 0.024 rad (1.4 grados) por ciclo.
VEL_ACCION_FLIPPER = 1.2     # rad/s
VEL_ACCION_ORUGA = 6.0       # rad/s


class ContratoInvalido(ValueError):
    """El contrato no se puede usar: mejor no arrancar que comandar a ciegas."""


class EstadoRobot:
    """Lo que el robot sabe de sí mismo en un instante, en unidades SI.

    Todas las listas van en el orden canónico fl, fr, rl, rr. `giro` y `acel`
    son vectores de 3 componentes (x, y, z). `extra` guarda las señales de
    sensores nuevos, por nombre, sin el prefijo 'extra/'.
    """

    def __init__(self):
        self.pos_flipper = [0.0] * 4
        self.vel_flipper = [0.0] * 4
        self.esf_flipper = [0.0] * 4
        self.consigna_flipper = [0.0] * 4
        self.vel_oruga = [0.0] * 4
        self.esf_oruga = [0.0] * 4
        self.consigna_oruga = [0.0] * 4
        self.roll = 0.0
        self.pitch = 0.0
        self.giro = [0.0, 0.0, 0.0]
        self.acel = [0.0, 0.0, 0.0]
        self.accion_previa = []
        self.t_activa = 0.0
        self.extra = {}


def senales(estado):
    """Convierte un EstadoRobot en {nombre de señal: valor}. Es el catálogo."""
    s = {}
    for i, n in enumerate(FLIPPERS):
        pos = float(estado.pos_flipper[i])
        consigna = float(estado.consigna_flipper[i])
        s[f'flipper_{n}/pos'] = pos
        s[f'flipper_{n}/vel'] = float(estado.vel_flipper[i])
        s[f'flipper_{n}/esfuerzo'] = float(estado.esf_flipper[i])
        s[f'flipper_{n}/consigna'] = consigna
        s[f'flipper_{n}/error'] = consigna - pos
    for i, n in enumerate(ORUGAS):
        vel = float(estado.vel_oruga[i])
        consigna = float(estado.consigna_oruga[i])
        s[f'track_{n}/vel'] = vel
        s[f'track_{n}/esfuerzo'] = float(estado.esf_oruga[i])
        s[f'track_{n}/consigna'] = consigna
        s[f'track_{n}/desliz'] = consigna - vel
    s['imu/roll'] = float(estado.roll)
    s['imu/pitch'] = float(estado.pitch)
    for k, eje in enumerate('xyz'):
        s[f'imu/giro_{eje}'] = float(estado.giro[k])
        s[f'imu/acel_{eje}'] = float(estado.acel[k])
    for i, v in enumerate(estado.accion_previa):
        s[f'accion_previa/{i}'] = float(v)
    s['reloj/t_activa'] = float(estado.t_activa)
    for nombre, v in estado.extra.items():
        s[f'extra/{nombre}'] = float(v)
    return s


def _senal_conocida(nombre, acc_dim):
    """Indica si `nombre` existe en el catálogo (las `extra/` se aceptan todas)."""
    if nombre.startswith('extra/') and len(nombre) > len('extra/'):
        return True
    if nombre == 'reloj/t_activa':
        return True
    if nombre.startswith('accion_previa/'):
        indice = nombre[len('accion_previa/'):]
        return indice.isdigit() and int(indice) < acc_dim
    grupo, _, campo = nombre.partition('/')
    if grupo in JUNTAS_FLIPPER:
        return campo in _CAMPOS_FLIPPER
    if grupo in JUNTAS_ORUGA:
        return campo in _CAMPOS_ORUGA
    return grupo == 'imu' and campo in _SENALES_IMU


def _numero(termino, clave, por_defecto=None):
    v = termino.get(clave, por_defecto)
    if v is None:
        raise ContratoInvalido(f'al término {termino} le falta "{clave}"')
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ContratoInvalido(f'"{clave}" no es un número en {termino}')
    if not math.isfinite(v):
        raise ContratoInvalido(f'"{clave}" no es finito en {termino}')
    return v


class Contrato:
    """Un contrato validado. Construirlo ya comprueba que se puede usar entero."""

    def __init__(self, datos):
        if not isinstance(datos, dict):
            raise ContratoInvalido('el contrato tiene que ser un diccionario')
        formato = datos.get('formato')
        if formato != FORMATO_CONTRATO:
            raise ContratoInvalido(
                f'contrato en formato {formato!r} y este código entiende el '
                f'formato {FORMATO_CONTRATO}')

        self.datos = copy.deepcopy(datos)
        self.nombre = str(datos.get('nombre', 'sin-nombre'))
        self.version = datos.get('version')
        self.frecuencia_hz = datos.get('frecuencia_hz')
        if self.frecuencia_hz is not None:
            self.frecuencia_hz = _numero(datos, 'frecuencia_hz')
            if self.frecuencia_hz <= 0:
                raise ContratoInvalido('frecuencia_hz tiene que ser positiva')

        accion = datos.get('accion')
        if not isinstance(accion, list) or not accion:
            raise ContratoInvalido('"accion" tiene que ser una lista no vacía')
        self._acciones = [self._validar_accion(t) for t in accion]
        self.acc_dim = len(self._acciones)

        observacion = datos.get('observacion')
        if not isinstance(observacion, list) or not observacion:
            raise ContratoInvalido('"observacion" tiene que ser una lista no vacía')
        self._terminos = [self._validar_termino(t) for t in observacion]
        self.obs_dim = len(self._terminos)

        self.senales_extra = sorted(
            {t[0][len('extra/'):] for t in self._terminos
             if t[0].startswith('extra/')})

    # ------------------------------------------------------------------ #
    def _validar_termino(self, t):
        if not isinstance(t, dict):
            raise ContratoInvalido(f'término de observación inválido: {t!r}')
        senal = t.get('senal')
        if not isinstance(senal, str) or not _senal_conocida(senal, self.acc_dim):
            raise ContratoInvalido(f'señal desconocida en la observación: {senal!r}')
        op = t.get('op')
        if op not in OPS:
            raise ContratoInvalido(f'op {op!r} desconocida en {t}; valen {OPS}')
        if op == 'escala':
            escala = _numero(t, 'escala')
            if escala == 0:
                raise ContratoInvalido(f'escala 0 en {t}')
            return (senal, op, escala, _numero(t, 'centro', 0.0),
                    bool(t.get('saturar', True)))
        return (senal, op, None, None, None)

    def _validar_accion(self, t):
        if not isinstance(t, dict):
            raise ContratoInvalido(f'término de acción inválido: {t!r}')
        destino = t.get('destino')
        modo = t.get('modo')
        escala = _numero(t, 'escala')
        if escala <= 0:
            raise ContratoInvalido(f'escala no positiva en {t}')
        if destino in JUNTAS_FLIPPER:
            if modo not in MODOS_FLIPPER:
                raise ContratoInvalido(
                    f'modo {modo!r} para {destino}; valen {MODOS_FLIPPER}')
            indices = (JUNTAS_FLIPPER.index(destino),)
            tipo = 'flipper'
        elif destino in JUNTAS_ORUGA or destino in LADOS:
            if modo != 'velocidad':
                raise ContratoInvalido(f'las orugas solo aceptan modo velocidad: {t}')
            nombres = LADOS.get(destino, (destino[len('track_'):],))
            indices = tuple(ORUGAS.index(n) for n in nombres)
            tipo = 'oruga'
        else:
            raise ContratoInvalido(f'destino de acción desconocido: {destino!r}')
        return (tipo, indices, modo, escala, _numero(t, 'centro', 0.0))

    # ------------------------------------------------------------------ #
    def observacion(self, estado):
        """Arma la observación: np.ndarray float32 de longitud obs_dim.

        Una señal pedida que no está (un sensor `extra/` que no llegó) es un
        error, no un cero: un sensor caído no puede parecer un sensor que mide
        cero.
        """
        s = senales(estado)
        obs = np.empty(self.obs_dim, dtype=np.float32)
        for k, (senal, op, escala, centro, saturar) in enumerate(self._terminos):
            if senal not in s:
                raise KeyError(f'falta la señal {senal!r}')
            v = s[senal]
            if not math.isfinite(v):
                raise ValueError(f'la señal {senal!r} vale {v}')
            if op == 'sin':
                v = math.sin(v)
            elif op == 'cos':
                v = math.cos(v)
            elif op == 'escala':
                v = (v - centro) / escala
                if saturar:
                    v = max(-1.0, min(1.0, v))
            obs[k] = v
        return obs

    def aplicar_accion(self, accion, estado, dt):
        """Salida de la red -> (consignas de /cmd_flippers, de /cmd_tracks).

        La acción se satura a [-1, 1] antes de nada. Devuelve dos listas de 4
        floats en el orden canónico: posiciones absolutas de los flippers (rad)
        y velocidades de las orugas (rad/s).
        """
        a = np.asarray(accion, dtype=np.float64).ravel()
        if a.shape[0] != self.acc_dim:
            raise ValueError(
                f'acción de {a.shape[0]} valores, el contrato dice {self.acc_dim}')
        if not np.all(np.isfinite(a)):
            raise ValueError(f'acción no finita: {a.tolist()}')
        a = np.clip(a, -1.0, 1.0)

        flippers = [float(v) for v in estado.consigna_flipper]
        orugas = [0.0] * len(ORUGAS)
        for valor, (tipo, indices, modo, escala, centro) in zip(a, self._acciones):
            for i in indices:
                if tipo == 'oruga':
                    orugas[i] = float(valor * escala)
                elif modo == 'velocidad':
                    flippers[i] = integrar_accion(
                        valor * escala, estado.pos_flipper[i], dt)
                else:
                    flippers[i] = float(centro + valor * escala)
        return flippers, orugas

    # ------------------------------------------------------------------ #
    def descripcion_observacion(self):
        """Nombre de cada componente, en orden. Para depurar y etiquetar gráficas."""
        return [s if op in ('crudo', 'escala') else f'{op}({s})'
                for s, op, *_ in self._terminos]

    def a_dict(self):
        return copy.deepcopy(self.datos)

    def huella(self):
        """Hash corto del contrato: dos contratos con la misma huella son iguales."""
        canon = json.dumps(self.datos, sort_keys=True, ensure_ascii=False,
                           separators=(',', ':'))
        return hashlib.sha256(canon.encode('utf-8')).hexdigest()[:12]

    @classmethod
    def desde_json(cls, ruta):
        """Lee un contrato suelto o el `.json` de un modelo ({"contrato": {...}})."""
        with open(ruta, encoding='utf-8') as f:
            datos = json.load(f)
        if isinstance(datos, dict) and 'contrato' in datos:
            datos = datos['contrato']
        return cls(datos)


def integrar_accion(velocidad, pos_actual, dt):
    """Velocidad de un flipper (rad/s) -> su nueva posición absoluta (rad).

    Se integra sobre la posición REAL medida, no sobre la consigna anterior, a
    propósito: si un flipper está atascado contra una piedra, la consigna no se
    le escapa hacia adelante acumulando un error enorme que luego se descarga de
    golpe al liberarse.
    """
    return float(pos_actual) + float(velocidad) * float(dt)


def contrato_provisional():
    """Nuestra propuesta, a la espera del contrato real: 40 observaciones, 6 acciones.

    Observación: por flipper sin/cos del ángulo, velocidad, error y esfuerzo
    (20); por oruga velocidad, deslizamiento y esfuerzo (12); chasis sin/cos de
    roll y pitch, giro y aceleración vertical (8). Sin pose ni rumbo: la
    política debe depender de lo que el robot siente bajo él, no de dónde está.

    Acción: velocidad de cada flipper (4) y de cada lado de orugas (2).
    """
    obs = []
    for n in JUNTAS_FLIPPER:
        obs += [
            {'senal': f'{n}/pos', 'op': 'sin'},
            {'senal': f'{n}/pos', 'op': 'cos'},
            {'senal': f'{n}/vel', 'op': 'escala', 'escala': VEL_FLIPPER_MAX},
            {'senal': f'{n}/error', 'op': 'escala', 'escala': ERROR_MAX},
            {'senal': f'{n}/esfuerzo', 'op': 'escala', 'escala': TORQUE_FLIPPER_MAX},
        ]
    for n in JUNTAS_ORUGA:
        obs += [
            {'senal': f'{n}/vel', 'op': 'escala', 'escala': VEL_ORUGA_MAX},
            {'senal': f'{n}/desliz', 'op': 'escala', 'escala': VEL_ORUGA_MAX},
            {'senal': f'{n}/esfuerzo', 'op': 'escala', 'escala': TORQUE_ORUGA_MAX},
        ]
    obs += [
        {'senal': 'imu/roll', 'op': 'sin'},
        {'senal': 'imu/roll', 'op': 'cos'},
        {'senal': 'imu/pitch', 'op': 'sin'},
        {'senal': 'imu/pitch', 'op': 'cos'},
        {'senal': 'imu/giro_x', 'op': 'escala', 'escala': GIRO_MAX},
        {'senal': 'imu/giro_y', 'op': 'escala', 'escala': GIRO_MAX},
        {'senal': 'imu/giro_z', 'op': 'escala', 'escala': GIRO_MAX},
        {'senal': 'imu/acel_z', 'op': 'escala', 'escala': ACEL_MAX},
    ]
    accion = [{'destino': n, 'modo': 'velocidad', 'escala': VEL_ACCION_FLIPPER}
              for n in JUNTAS_FLIPPER]
    accion += [{'destino': lado, 'modo': 'velocidad', 'escala': VEL_ACCION_ORUGA}
               for lado in LADOS]
    return Contrato({
        'formato': FORMATO_CONTRATO,
        'nombre': 'provisional-ugv',
        'version': 2,
        'frecuencia_hz': 50,
        'observacion': obs,
        'accion': accion,
    })
