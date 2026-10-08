#!/usr/bin/env python3
"""contrato_politica.py — Contrato entre el entrenamiento (PC) y la inferencia (Pi).

ESTE ARCHIVO ES LA ÚNICA FUENTE DE VERDAD del vector de observación y del vector
de acción. Lo lee el entrenamiento en el PC y lo lee el nodo de inferencia en la
Raspberry. Si los dos no coinciden bit a bit en el orden y la escala de cada
componente, el modelo entrenado produce basura en el robot sin dar ningún error:
la red recibe números en posiciones que no esperaba y devuelve ángulos
plausibles pero equivocados. Es el fallo sim-to-real más caro y el más difícil
de diagnosticar, porque nada se cae.

Para que no pueda pasar en silencio, `VERSION_CONTRATO` se graba en el JSON que
acompaña al modelo exportado y el nodo de inferencia se niega a cargar un modelo
cuya versión no sea la suya.

NO IMPORTA ROS NI GYMNASIUM. Son funciones puras sobre arrays, a propósito: el
mismo código corre dentro del simulador en el PC y dentro del nodo ROS en la Pi.

---------------------------------------------------------------------------
LA IDEA: PERCEPCIÓN TÁCTIL SIN SENSOR TÁCTIL
---------------------------------------------------------------------------
El objetivo es que el robot note las imperfecciones del suelo y acomode los
flippers solo. La tentación es buscar un sensor de contacto que montar en la
punta de cada flipper. No hace falta: los motores YA son el sensor.

Cuando un flipper se apoya contra el terreno pasan dos cosas medibles, las dos
disponibles hoy en /joint_states:

  1. EL ERROR DE SEGUIMIENTO CRECE. El lazo de posición del ODrive persigue la
     consigna; si algo lo frena, el eje se queda atrás. `consigna - real` es
     proporcional a cuánto le cuesta llegar, o sea al contacto.
  2. EL PAR SUBE. El driver tiene que meter más corriente para sostener la
     posición. `effort` de /joint_states (torque_nm, derivado de Iq) es una
     medida directa de la fuerza de contacto.

Esto es exactamente lo que hacen los cuadrúpedos (ANYmal, MIT Cheetah) para
detectar el apoyo de las patas: propiocepción, no tacto. La misma señal sirve
para distinguir "pisé firme", "estoy patinando" y "hay un escalón aquí".

  ADVERTENCIA SOBRE EL PAR: el GIM6010-8 trae los mensajes de Iq APAGADOS de
  fábrica. Sin activarlos por USB (`odrv0.axis0.config.can.iq_rate_ms = 10`),
  el campo `effort` llega en cero y la mitad de la señal táctil no existe. El
  error de seguimiento sí funciona sin tocar nada, así que el contrato sirve
  igual, pero con la mano atada. Activarlo en los 8 motores es prerrequisito.

  Y ojo con KT_NM_POR_A = 1.0 en protocolo_can.py: es un marcador de posición.
  Mientras no se ponga el Kt real del datasheet, `effort` está en una unidad
  arbitraria. Para el RL da igual (la red aprende la escala que le den), PERO
  tiene que ser LA MISMA en entrenamiento y en el robot. Por eso la constante
  de normalización vive aquí y no en cada nodo.

---------------------------------------------------------------------------
OBSERVACIÓN (40 valores, todos normalizados a ~[-1, 1])
---------------------------------------------------------------------------
Por cada flipper (4: fl, fr, rl, rr), 5 valores = 20

    sin(angulo), cos(angulo)   las juntas son `continuous` (360 grados), así
                               que el ángulo crudo tiene un salto en +-pi que
                               la red tendría que aprender a ignorar. En seno y
                               coseno el espacio es continuo y sin costuras.
    velocidad / VEL_FLIPPER_MAX
    error_seguimiento / ERROR_MAX      <- señal táctil 1
    esfuerzo / TORQUE_FLIPPER_MAX      <- señal táctil 2

Por cada oruga (4: fl, fr, rl, rr), 3 valores = 12

    velocidad / VEL_ORUGA_MAX
    deslizamiento (comandada - real) / VEL_ORUGA_MAX   <- patina = terreno malo
    esfuerzo / TORQUE_ORUGA_MAX                        <- carga de tracción

Chasis, 8 valores

    sin(roll), cos(roll), sin(pitch), cos(pitch)   actitud del EKF o la IMU
    giro_x, giro_y, giro_z / GIRO_MAX              velocidad angular
    acel_z / ACEL_MAX                              impactos verticales

Deliberadamente NO entran: la posición absoluta en el mundo ni el rumbo. La
política de flippers debe depender solo de lo que el robot siente bajo él, no
de dónde está. Si entrara la pose, el modelo se aprendería el escenario de
entrenamiento de memoria y no generalizaría a un terreno nuevo.

---------------------------------------------------------------------------
ACCIÓN (4 valores en [-1, 1])
---------------------------------------------------------------------------
Una VELOCIDAD normalizada por flipper, no un ángulo absoluto. Tres razones:

  - Un ángulo absoluto permite que un error de un paso mande el flipper de un
    extremo a otro en un ciclo. Una velocidad acotada no puede: el peor error
    posible mueve el flipper `VEL_ACCION_MAX / frecuencia` radianes.
  - La red no tiene que saber dónde está el cero mecánico de cada flipper.
  - Encaja con cómo se siente el terreno: "sube un poco este flipper" es la
    corrección natural ante un bache, no "ponte en 37 grados".

`integrar_accion()` convierte esas velocidades en las 4 posiciones absolutas
que espera /cmd_flippers, acotando el paso. Es la última barrera de seguridad
antes del hardware y vive aquí para que el simulador aplique exactamente la
misma, o el robot se comportará distinto a como se entrenó.
"""
import math

import numpy as np

# Subir esto SIEMPRE que cambie el orden, el tamaño o la escala de los vectores.
# Un modelo exportado con otra versión será rechazado al cargarlo.
VERSION_CONTRATO = 1

# Orden canónico. Es el mismo de /cmd_flippers, /cmd_tracks y de los IDs de
# motor (1-4 orugas, 5-8 flippers) en todo el proyecto. No reordenar.
FLIPPERS = ('fl', 'fr', 'rl', 'rr')
ORUGAS = ('fl', 'fr', 'rl', 'rr')
JUNTAS_FLIPPER = tuple(f'flipper_{n}' for n in FLIPPERS)
JUNTAS_ORUGA = tuple(f'track_{n}' for n in ORUGAS)

# ---------------------------------------------------------------------- #
# Escalas de normalización.
#
# No son límites duros: son el valor que mapea a 1.0. Se saturan con clip, así
# que pasarse no rompe nada, solo aplana. Lo importante es que el PC y la Pi
# usen EXACTAMENTE los mismos números, que es el motivo de que vivan aquí.
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

# Velocidad máxima que puede pedir la política, con la acción saturada a 1.0.
# Es el tope de seguridad del lazo: con un paso de control a 50 Hz, el peor
# comando posible mueve el flipper 1.2/50 = 0.024 rad (1.4 grados) por ciclo.
VEL_ACCION_MAX = 1.2         # rad/s

# Dimensiones derivadas. Si no cuadran con la entrada del modelo ONNX, el nodo
# de inferencia se niega a arrancar.
OBS_POR_FLIPPER = 5
OBS_POR_ORUGA = 3
OBS_CHASIS = 8
OBS_DIM = (len(FLIPPERS) * OBS_POR_FLIPPER
           + len(ORUGAS) * OBS_POR_ORUGA
           + OBS_CHASIS)          # 20 + 12 + 8 = 40
ACC_DIM = len(FLIPPERS)           # 4


def _n(valor, escala):
    """Normaliza y satura a [-1, 1]. None y NaN cuentan como 0."""
    if valor is None:
        return 0.0
    v = float(valor)
    if not math.isfinite(v):
        return 0.0
    return max(-1.0, min(1.0, v / escala))


def construir_observacion(pos_flipper, vel_flipper, esfuerzo_flipper,
                          consigna_flipper, vel_oruga, esfuerzo_oruga,
                          consigna_oruga, roll, pitch, giro, acel_z):
    """Arma el vector de observación. Función pura: mismas entradas, misma salida.

    Todas las secuencias van en el orden canónico fl, fr, rl, rr y en unidades
    del EJE DE SALIDA (radianes, rad/s, Nm), que es lo que publica
    /joint_states. `giro` son las tres componentes de velocidad angular.

    Devuelve un np.ndarray float32 de longitud OBS_DIM.
    """
    obs = []

    for i in range(len(FLIPPERS)):
        ang = float(pos_flipper[i])
        error = float(consigna_flipper[i]) - ang
        obs += [
            math.sin(ang),
            math.cos(ang),
            _n(vel_flipper[i], VEL_FLIPPER_MAX),
            _n(error, ERROR_MAX),
            _n(esfuerzo_flipper[i], TORQUE_FLIPPER_MAX),
        ]

    for i in range(len(ORUGAS)):
        real = float(vel_oruga[i])
        obs += [
            _n(real, VEL_ORUGA_MAX),
            _n(float(consigna_oruga[i]) - real, VEL_ORUGA_MAX),
            _n(esfuerzo_oruga[i], TORQUE_ORUGA_MAX),
        ]

    obs += [
        math.sin(roll), math.cos(roll),
        math.sin(pitch), math.cos(pitch),
        _n(giro[0], GIRO_MAX), _n(giro[1], GIRO_MAX), _n(giro[2], GIRO_MAX),
        _n(acel_z, ACEL_MAX),
    ]

    vec = np.asarray(obs, dtype=np.float32)
    if vec.shape[0] != OBS_DIM:
        raise ValueError(
            f'observación de {vec.shape[0]} valores, el contrato dice {OBS_DIM}. '
            f'Alguien cambió el layout sin subir VERSION_CONTRATO.')
    return vec


def integrar_accion(accion, pos_actual, dt, limites=None):
    """Acción de la red -> las 4 posiciones absolutas de /cmd_flippers.

    Última barrera antes del hardware, y por eso vive en el contrato: el
    simulador tiene que aplicar exactamente esta misma, o el robot se comportará
    distinto a como se entrenó.

    Parameters
    ----------
    accion : secuencia de 4 velocidades normalizadas, se saturan a [-1, 1].
    pos_actual : 4 ángulos REALES medidos (rad), no la consigna anterior.
        Integrar sobre la posición medida y no sobre la consigna es
        deliberado: si un flipper está atascado contra una piedra, la consigna
        no se le escapa hacia adelante acumulando un error enorme que luego se
        descarga de golpe al liberarse.
    dt : paso de tiempo en segundos.
    limites : (min_rad, max_rad) o None. Las juntas son `continuous`, así que
        por defecto no hay tope; se puede poner uno si el arnés lo exige.

    Returns
    -------
    list[float] de 4 posiciones absolutas en rad.
    """
    a = np.clip(np.asarray(accion, dtype=np.float64).ravel(), -1.0, 1.0)
    if a.shape[0] != ACC_DIM:
        raise ValueError(f'acción de {a.shape[0]} valores, el contrato dice {ACC_DIM}')

    paso = a * VEL_ACCION_MAX * float(dt)
    objetivo = np.asarray(pos_actual, dtype=np.float64).ravel() + paso

    if limites is not None:
        objetivo = np.clip(objetivo, float(limites[0]), float(limites[1]))
    return [float(v) for v in objetivo]


def descripcion_observacion():
    """Nombre de cada componente, en orden. Para depurar y etiquetar las gráficas."""
    nombres = []
    for n in FLIPPERS:
        nombres += [f'flipper_{n}/sin', f'flipper_{n}/cos', f'flipper_{n}/vel',
                    f'flipper_{n}/error', f'flipper_{n}/esfuerzo']
    for n in ORUGAS:
        nombres += [f'track_{n}/vel', f'track_{n}/desliz', f'track_{n}/esfuerzo']
    nombres += ['chasis/sin_roll', 'chasis/cos_roll',
                'chasis/sin_pitch', 'chasis/cos_pitch',
                'chasis/giro_x', 'chasis/giro_y', 'chasis/giro_z',
                'chasis/acel_z']
    return nombres


def resumen():
    """Contrato serializable. Se graba junto al modelo y se verifica al cargarlo."""
    return {
        'version_contrato': VERSION_CONTRATO,
        'obs_dim': OBS_DIM,
        'acc_dim': ACC_DIM,
        'flippers': list(FLIPPERS),
        'orugas': list(ORUGAS),
        'observacion': descripcion_observacion(),
        'escalas': {
            'VEL_FLIPPER_MAX': VEL_FLIPPER_MAX,
            'ERROR_MAX': ERROR_MAX,
            'TORQUE_FLIPPER_MAX': TORQUE_FLIPPER_MAX,
            'VEL_ORUGA_MAX': VEL_ORUGA_MAX,
            'TORQUE_ORUGA_MAX': TORQUE_ORUGA_MAX,
            'GIRO_MAX': GIRO_MAX,
            'ACEL_MAX': ACEL_MAX,
            'VEL_ACCION_MAX': VEL_ACCION_MAX,
        },
    }
