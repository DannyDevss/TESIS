#!/usr/bin/env python3
"""registrar_ensayo.py — Excita UN motor en modo PAR y registra su respuesta a CSV.

Es la mitad "de banco" de la identificación de sistemas: mete una señal conocida
(escalón, chirp, PRBS o escalera de fricción) por la entrada de par y guarda lo
que devuelve el encoder, con marcas de tiempo utilizables. La otra mitad, el
ajuste del modelo y el cálculo de ganancias, vive en `identificar.py` / `.m`.

---------------------------------------------------------------------------
POR QUÉ EN MODO PAR Y NO EN VELOCIDAD
---------------------------------------------------------------------------
Lo que se quiere identificar es la PLANTA (inercia, fricción, gravedad), no el
lazo de velocidad que ya trae el driver. Si se excita en modo velocidad, lo que
se mide es el lazo cerrado CON las ganancias actuales, que son justo las que se
quieren calcular: se muerde la cola. En modo par (control_mode = 1) el driver
sólo cierra el lazo de corriente —rapidísimo comparado con la mecánica— y lo
que se ve entre `Input_Torque` y la velocidad del encoder es la planta desnuda.

---------------------------------------------------------------------------
EL PROBLEMA DEL PUNTO DE OPERACIÓN, Y EL REGULADOR LENTO
---------------------------------------------------------------------------
En par puro, una oruga girando en el aire es un integrador con muy poca
fricción: al primer semiciclo lento del chirp SE EMBALA y el ensayo muere contra
el límite de velocidad. (No es teoría: la primera versión de este script abortó
así en la propia simulación, a los 1,6 s.) El flipper tiene el problema
simétrico: la gravedad se lo lleva contra el tope.

La solución es la misma para los dos: un regulador PI LENTO en el host, que
sostiene el punto de operación mientras la excitación cabalga encima.

    ancho de banda del regulador   ~0,4 rad/s   (`--w-reg`)
    banda de identificación         6-125 rad/s (f0..f1 del chirp)

Con más de una década de separación, el regulador no interfiere en la banda que
se está midiendo. Además se le realimenta la velocidad FILTRADA a su propio
ancho de banda, de modo que su ganancia dentro de la banda de identificación cae
otro orden de magnitud. Aun así, el par del regulador ES parte de la entrada de
la planta, y por eso el CSV guarda `par_cmd` (total) además de `par_exc`: el
ajuste usa `par_cmd`, que es lo que el motor recibió de verdad. Identificar con
`par_exc` mientras el regulador mete par por detrás sesgaría J y b.

En la oruga el punto de operación NO es velocidad cero, sino `--vel-operacion`
(3 rev/s por defecto). Es a propósito: alrededor del cero, la fricción de
Coulomb del reductor 8:1 es una discontinuidad, y ningún modelo lineal la
describe. Girando siempre en el mismo sentido, la fricción es localmente una
recta y el ajuste tiene sentido. En el flipper la referencia es 0 y el regulador
converge solo al par que compensa la gravedad.

---------------------------------------------------------------------------
UNIDADES: SE REGISTRA EN CRUDO, TAL COMO HABLA EL DRIVER
---------------------------------------------------------------------------
Este script NO usa las conversiones de `protocolo_can` (rad, eje de salida).
Guarda revoluciones, rev/s y N·m tal cual viajan por el bus, a propósito:

  1. `RELACION_REDUCCION = 8.0` sigue marcada como SIN VERIFICAR en hardware.
     Meter una constante no verificada en los datos de identificación
     contaminaría el resultado sin dejar rastro.
  2. Es que además no hace falta. `vel_gain` del driver opera en SUS unidades:
     par por (rev/s) de error. Si la inercia J' se identifica en esas mismas
     unidades, la ganancia sale directa:

         vel_gain = J' * omega_c        (omega_c en rad/s, J' en par/(rev/s^2))

     sin ningún 2*pi ni ningún 8. La relación con el J del SI, por si se quiere
     poner en la memoria, es J' = 2*pi*J_SI referido al MISMO eje.

Lo mismo con el par: el manual dice que en el control normal por CAN la
posición, la velocidad y el par se refieren al lado del ROTOR (sólo el modo MIT
usa el eje de salida), pero la constante de par del datasheet (0,47 N·m/A frente
a los 8,27/Kv = 0,67 N·m/A que tocarían en el rotor) no cuadra del todo con esa
lectura. No importa para lo que hace este script: mientras el par que se manda y
la ganancia que se calcula estén en la misma unidad —la del driver, sea cual
sea—, el resultado es coherente. Lo que NO se puede hacer es mezclar estos
números con N·m del datasheet sin resolver antes esa ambigüedad.

---------------------------------------------------------------------------
SEGURIDAD (leer antes de energizar)
---------------------------------------------------------------------------
En modo par el motor NO tiene lazo de posición ni de velocidad. El regulador
lento de este script es una red, no un freno: si el proceso muere, lo único que
queda es el límite que se le fijó al driver por CAN.

  * Oruga (IDs 1-4): el robot LEVANTADO, con la oruga girando en el aire, o
    directamente el motor solo en el banco. Nunca apoyado en el suelo.
  * Flipper (IDs 5-8): recorrido MECÁNICAMENTE LIMITADO. Antes del ensayo hay
    que saber dónde están los topes y dejar el flipper a media carrera. El
    script vigila la excursión y aborta, pero el margen lo pone el usuario con
    `--limite-pos`.

Protecciones que sí implementa el script, todas con parada inmediata (par 0 +
IDLE) y anotadas en el CSV:

    - excursión de posición fuera de `--limite-pos`
    - velocidad por encima de `--limite-vel`
    - `axis_error` distinto de 0 en el heartbeat
    - silencio del motor (sin tramas de encoder) más de 200 ms
    - Ctrl+C

Además fija por CAN (0x00F) un límite de velocidad y de corriente en el propio
driver antes de empezar, que es la red que queda si este proceso se muere.

---------------------------------------------------------------------------
USO
---------------------------------------------------------------------------
    source /opt/ros/jazzy/setup.bash && source ~/TESIS/install/setup.bash

    # 0) Ensayar la cadena entera SIN hardware (planta simulada):
    python3 identificacion/registrar_ensayo.py --motor 1 --perfil chirp --simular

    # 1) Fricción de la oruga (escalera de par, estado estacionario):
    python3 identificacion/registrar_ensayo.py --motor 1 --perfil friccion --canal can0

    # 2) Dinámica de la oruga (chirp; la amplitud se calcula sola tras la sonda):
    python3 identificacion/registrar_ensayo.py --motor 1 --perfil chirp --canal can0

    # 3) Validación con OTRA señal (nunca se valida con la de ajuste):
    python3 identificacion/registrar_ensayo.py --motor 1 --perfil prbs --canal can0

    # 4) Flipper: el regulador busca solo el par que compensa la gravedad.
    python3 identificacion/registrar_ensayo.py --motor 5 --perfil chirp \
        --canal can0 --amplitud 0.10 --limite-pos 0.4

En el pi3hat (sin adaptador USB-CAN), añadir `--pi3hat`; el bus se deduce del
mapa por esquina de `driver_movimiento`. Va bien, pero las marcas de tiempo son
las del ciclo del host, no las del kernel: para identificar es peor. Si hay un
adaptador SocketCAN a mano, usarlo.
"""
import argparse
import json
import math
import os
import random
import signal
import struct
import sys
import time
from datetime import datetime

# `protocolo_can` es la única fuente de verdad de IDs y códigos de comando; de
# ahí se toman, y NADA de sus conversiones de unidades (ver cabecera).
_RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_RAIZ, 'src', 'ugv_bridge'))
try:
    from ugv_bridge import protocolo_can as proto
except ImportError as e:  # pragma: no cover
    sys.exit(f'No se pudo importar ugv_bridge.protocolo_can ({e}).\n'
             f'Correr desde la raíz del repo, o hacer:\n'
             f'    source /opt/ros/jazzy/setup.bash && source ~/TESIS/install/setup.bash')

IDS_ORUGAS = [1, 2, 3, 4]
IDS_FLIPPERS = [5, 6, 7, 8]

# Cableado por esquina, igual que MAPA_BUSES_POR_DEFECTO de driver_movimiento.
MAPA_BUSES = {1: 1, 5: 1, 2: 2, 6: 2, 3: 3, 7: 3, 4: 4, 8: 4}

# Valores por defecto por familia. Deliberadamente TÍMIDOS: el primer ensayo se
# hace con esto y se sube la amplitud sólo si la respuesta se pierde en el ruido.
POR_DEFECTO = {
    'oruga': dict(amplitud=None,            # se calcula tras la sonda
                  limite_vel=10.0, limite_pos=1e9, limite_corriente=6.0,
                  vel_operacion=3.0, f0=1.0, f1=20.0, duracion=25.0,
                  par_sonda=0.05, swing=1.5),
    'flipper': dict(amplitud=None,          # se calcula tras las dos sondas
                    limite_vel=6.0, limite_pos=0.40, limite_corriente=6.0,
                    vel_operacion=0.0, f0=0.3, f1=12.0, duracion=25.0,
                    par_sonda=0.05, swing=None),
}

SILENCIO_MAX_S = 0.20     # sin tramas de encoder -> abortar
CALENTAMIENTO_S = 1.0     # par 0 antes de nada, para fijar el cero de posición
FUNDIDO_S = 0.25          # rampa de entrada de la excitación (evita el tirón en t=0)


def familia(id_motor):
    if id_motor in IDS_ORUGAS:
        return 'oruga'
    if id_motor in IDS_FLIPPERS:
        return 'flipper'
    raise ValueError(f'ID de motor fuera de los IDs acordados (1-8): {id_motor}')


# ---------------------------------------------------------------------- #
# Tramas en unidades CRUDAS del driver (ver cabecera: sin reducción, sin rad)
# ---------------------------------------------------------------------- #
CMD_SET_INPUT_TORQUE = 0x00E   # manual 4.1.2: float32 Input_Torque


def trama_par(nm):
    return struct.pack('<f', float(nm))


def trama_limites(vel_rev_s, corriente_a):
    return struct.pack('<ff', float(abs(vel_rev_s)), float(abs(corriente_a)))


def parsear_encoder_crudo(data):
    """0x009 -> (pos_rev, vel_rev_s) SIN convertir. None si la trama es corta."""
    if data is None or len(data) < 8:
        return None
    return struct.unpack('<ff', bytes(data[:8]))


def _sat(v, lim):
    return max(-lim, min(lim, v))


# ---------------------------------------------------------------------- #
# Señales de excitación
# ---------------------------------------------------------------------- #
class Excitacion:
    """u(t) de cada perfil, normalizada a +-1; la amplitud la aplica el bucle.

    Los cuatro perfiles no son intercambiables:

    escalon  -- mirar a ojo si la respuesta tiene la pinta esperada. Vale para
                sanidad, es POBRE para identificar: excita una banda estrecha.
    chirp    -- el caballo de batalla. Barrido logarítmico f0->f1: reparte la
                energía por décadas, así que el ajuste no queda dominado por las
                frecuencias altas como pasa con un barrido lineal.
    prbs     -- señal de VALIDACIÓN. Distinta en espectro al chirp, que es justo
                lo que hace falta: validar con la misma señal del ajuste no
                demuestra nada.
    friccion -- escalera de par constante. No es dinámica: cada peldaño se
                mantiene hasta que la velocidad se estabiliza, y de la pareja
                (par, velocidad final) sale la recta b*w + Tc de la fricción.
    """

    def __init__(self, perfil, args):
        self.perfil = perfil
        self.f0 = args.f0
        self.f1 = args.f1
        self.dur = args.duracion
        self.t_paso = args.t_paso
        self.rng = random.Random(args.semilla)
        self._prbs_valor = 1.0
        self._prbs_prox = 0.0
        self._prbs_hold = 1.0 / (2.0 * max(args.f1, 0.5))
        # Peldaños de la escalera: pasa por los dos signos, para que el Coulomb
        # se vea a ambos lados del cero.
        n = max(int(args.duracion / max(args.t_paso, 0.1)), 4)
        self.peldanos = [(-1.0 + 2.0 * i / (n - 1)) for i in range(n)]

    def duracion_total(self):
        if self.perfil == 'friccion':
            return len(self.peldanos) * self.t_paso
        return self.dur

    def __call__(self, t):
        u = self._crudo(t)
        # Fundido de entrada: sin esto el chirp arranca con un salto de par.
        if t < FUNDIDO_S and self.perfil != 'friccion':
            u *= 0.5 * (1.0 - math.cos(math.pi * t / FUNDIDO_S))
        return u

    def _crudo(self, t):
        if self.perfil == 'escalon':
            return 0.0 if t < 1.0 else 1.0

        if self.perfil == 'chirp':
            # Barrido logarítmico: f(t) = f0*(f1/f0)^(t/T). La fase es su
            # integral; usarla directamente (en vez de sin(2*pi*f(t)*t), que es
            # el error clásico) es lo que hace que las frecuencias que se
            # excitan sean de verdad las que se creen.
            k = self.f1 / self.f0
            T = self.dur
            fase = 2 * math.pi * self.f0 * T / math.log(k) * (k ** (t / T) - 1.0)
            return math.sin(fase)

        if self.perfil == 'prbs':
            if t >= self._prbs_prox:
                self._prbs_prox = t + self._prbs_hold
                self._prbs_valor = 1.0 if self.rng.random() < 0.5 else -1.0
            return self._prbs_valor

        if self.perfil == 'friccion':
            i = min(int(t / self.t_paso), len(self.peldanos) - 1)
            return self.peldanos[i]

        raise ValueError(self.perfil)


class Regulador:
    """PI lento que sostiene el punto de operación (ver cabecera del módulo).

    Se sintoniza con la MISMA fórmula que este proyecto le propone al driver:
    kp = J*w_reg. Eso vuelve el regulador insensible a lo que valga la inercia
    real, que es justo lo que hace falta cuando aún no se ha identificado nada:
    la sonda da un J aproximado y con él el regulador ya queda bien amortiguado.

        kp = J*w_reg          ki = 0.5*J*w_reg^2

    La ecuación característica queda  J*s^2 + (b+kp)*s + ki = 0, o sea
    wn = 0,71*w_reg y zeta >= 0,71 SEA CUAL SEA la fricción b: el regulador no
    puede quedar mal amortiguado ni aunque J se haya estimado con un factor 2 de
    error.

    La velocidad realimentada va filtrada a `w_reg`: dentro de la banda de
    identificación el regulador ve una señal ya atenuada y casi no responde a la
    excitación, que es como debe ser.

    Con `pos_ref` (el flipper) lleva encima un sostén de POSICIÓN todavía más
    lento, que devuelve el eje a su ángulo de partida. La oruga no lo usa: ahí la
    posición no significa nada, son vueltas dadas.
    """

    def __init__(self, J, w_reg, vel_ref, par_max, pos_ref=None, vel_sostener=0.25):
        self.kp = J * w_reg
        self.ki = 0.5 * J * w_reg * w_reg
        self.w_reg = w_reg
        self.vel_ref = vel_ref
        self.par_max = par_max
        # Sostén de POSICIÓN, sólo para el flipper (ver docstring de la clase).
        self.pos_ref = pos_ref
        self.kx = w_reg / 4.0
        self.vel_sostener = vel_sostener
        self.i = 0.0
        self.vel_filtrada = 0.0

    def __call__(self, vel, dt, pos=None):
        if self.pos_ref is not None and pos is not None:
            # Cascada lenta: el error de posición pide velocidad, saturada a algo
            # muy pequeño. Sin esto el flipper se queda aparcado donde lo dejó la
            # sonda —el regulador de velocidad lo sostiene igual de contento en
            # cualquier ángulo— y arranca la excitación con medio recorrido ya
            # gastado. Su ancho de banda es w_reg/4, otra década por debajo de la
            # banda de identificación.
            self.vel_ref = _sat(-self.kx * (pos - self.pos_ref), self.vel_sostener)
        alfa = min(1.0, self.w_reg * dt)
        self.vel_filtrada += alfa * (vel - self.vel_filtrada)
        e = self.vel_ref - self.vel_filtrada
        self.i = _sat(self.i + self.ki * e * dt, self.par_max)
        return _sat(self.kp * e + self.i, self.par_max)


# ---------------------------------------------------------------------- #
# Transportes
# ---------------------------------------------------------------------- #
class TransporteSocketCAN:
    """SocketCAN (can0 con adaptador USB-CAN, vcan0 con el emulador).

    Es el transporte PREFERIDO para identificar: `msg.timestamp` de python-can
    con socketcan es la marca de tiempo que pone el kernel al recibir la trama,
    no la que pone el proceso al leerla. La diferencia es de milisegundos, pero
    milisegundos de jitter en el eje de tiempos es exactamente el ruido que peor
    lleva un ajuste de modelo.
    """

    tiempos_del_kernel = True

    def __init__(self, canal):
        try:
            import can
        except ImportError:
            sys.exit('Falta python-can:  pip install python-can  '
                     '(o apt install python3-can)')
        self._can = can
        self.bus = can.interface.Bus(channel=canal, interface='socketcan')

    def enviar(self, node_id, cmd_id, datos=b''):
        self.bus.send(self._can.Message(
            arbitration_id=proto.id_arbitraje(node_id, cmd_id),
            data=bytes(datos), is_extended_id=False))

    def drenar(self, hasta_t):
        """Todo lo que haya llegado hasta `hasta_t` (monotónico) -> lista."""
        salida = []
        while True:
            restante = hasta_t - time.monotonic()
            if restante <= 0:
                return salida
            msg = self.bus.recv(timeout=restante)
            if msg is None:
                continue
            nodo = proto.nodo_de_id(msg.arbitration_id)
            if nodo is None:
                continue
            salida.append((msg.timestamp, nodo,
                           proto.cmd_de_id(msg.arbitration_id), bytes(msg.data)))

    def cerrar(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


class TransportePi3Hat:
    """Envoltorio del transporte del proyecto, para correr sin adaptador USB-CAN.

    Peor para identificar que SocketCAN: el pi3hat entrega las tramas al final
    del ciclo SPI, así que todas las de un ciclo comparten marca de tiempo y el
    eje de tiempos queda cuantizado al periodo del bucle. Sirve, pero conviene
    bajar `--rate-hz` y no fiarse de la banda alta del ajuste.
    """

    tiempos_del_kernel = False

    def __init__(self, id_motor):
        from ugv_bridge import pi3hat_backend as backend
        self._t = backend.TransportePi3Hat(MAPA_BUSES, ids_presentes=[id_motor])
        self._pendientes = []

    def enviar(self, node_id, cmd_id, datos=b''):
        recibido = self._t.intercambiar([(node_id, cmd_id, bytes(datos))])
        ahora = time.time()
        for nodo, tramas in recibido.items():
            for cmd, data in tramas.items():
                self._pendientes.append((ahora, nodo, cmd, data))

    def drenar(self, hasta_t):
        salida, self._pendientes = self._pendientes, []
        espera = hasta_t - time.monotonic()
        if espera > 0:
            time.sleep(espera)
        return salida

    def cerrar(self):
        try:
            self._t.cerrar()
        except Exception:
            pass


class TransporteSimulado:
    """Planta simulada: permite ensayar TODA la cadena sin tocar el hardware.

    No es un adorno. El orden correcto de trabajo es correr aquí el ensayo, el
    ajuste y el cálculo de ganancias, comprobar que `identificar.py` devuelve los
    parámetros que se le metieron a la simulación, y sólo entonces energizar. Si
    la cadena de análisis está mal, más vale descubrirlo sin un motor girando.

    El modelo reproduce lo que hace difícil el problema real:
      - primer orden (oruga) o segundo orden con gravedad (flipper)
      - fricción de Coulomb, que es la no linealidad que rompe el ajuste lineal
      - retardo de transporte del bus
      - cuantización del encoder y jitter en el periodo de emisión
    """

    tiempos_del_kernel = True

    # Parámetros "de verdad" que el análisis tiene que recuperar.
    VERDAD = {
        'oruga': dict(J=0.0022, b=0.0035, Tc=0.010, k=0.0),
        'flipper': dict(J=0.0060, b=0.0090, Tc=0.014, k=0.045),
    }

    def __init__(self, id_motor, periodo_encoder=0.010, semilla=0):
        self.p = dict(self.VERDAD[familia(id_motor)])
        self.rng = random.Random(semilla)
        self.pos = 0.0
        self.vel = 0.0
        self.u = 0.0
        self.cola_u = []                 # (t_efectivo, par) -> retardo del bus
        self.retardo = 0.0018
        self.periodo = periodo_encoder
        self.t0 = time.monotonic()
        self.t_sim = 0.0
        self.prox_emision = 0.0
        self.armado = False

    def enviar(self, node_id, cmd_id, datos=b''):
        if cmd_id == CMD_SET_INPUT_TORQUE:
            (par,) = struct.unpack('<f', bytes(datos[:4]))
            self.cola_u.append((self.t_sim + self.retardo, par))
        elif cmd_id == proto.CMD_SET_AXIS_STATE:
            (estado,) = struct.unpack('<I', bytes(datos[:4]))
            self.armado = (estado == proto.ESTADO_CLOSED_LOOP)

    def drenar(self, hasta_t):
        salida = []
        objetivo = hasta_t - self.t0
        h = 0.0005
        while self.t_sim < objetivo:
            while self.cola_u and self.cola_u[0][0] <= self.t_sim:
                self.u = self.cola_u.pop(0)[1]
            par = self.u if self.armado else 0.0
            # J*w' = u - b*w - Tc*sgn(w) - k*x   (k=0 en la oruga)
            coulomb = self.p['Tc'] * math.tanh(self.vel / 0.02)
            acc = (par - self.p['b'] * self.vel - coulomb
                   - self.p['k'] * self.pos) / self.p['J']
            self.vel += acc * h
            self.pos += self.vel * h
            self.t_sim += h
            if self.t_sim >= self.prox_emision:
                self.prox_emision = self.t_sim + self.periodo * (
                    1.0 + 0.05 * (self.rng.random() - 0.5))
                pos_q = round(self.pos * 16384) / 16384.0   # cuantización encoder
                vel_q = self.vel + self.rng.gauss(0, 0.004)
                salida.append((self.t0 + self.t_sim, None,
                               proto.CMD_GET_ENCODER_ESTIMATES,
                               struct.pack('<ff', pos_q, vel_q)))
                salida.append((self.t0 + self.t_sim, None, proto.CMD_HEARTBEAT,
                               proto.trama_heartbeat(
                                   estado_eje=proto.ESTADO_CLOSED_LOOP if self.armado
                                   else proto.ESTADO_IDLE)))
        espera = hasta_t - time.monotonic()
        if espera > 0:
            time.sleep(espera)
        return salida

    def cerrar(self):
        pass


# ---------------------------------------------------------------------- #
# Ensayo
# ---------------------------------------------------------------------- #
class Abortado(RuntimeError):
    pass


class Ensayo:
    def __init__(self, args):
        self.args = args
        self.id = args.motor
        self.familia = familia(args.motor)
        self.filas = []            # (t, par_cmd, par_exc, pos, vel, fase)
        self.eventos = []
        self.pos0 = None
        self.vel_actual = 0.0
        self.pos_actual = None
        self.J_sonda = None
        self.amplitud_tanteo = None
        self.regulador = None
        self.ultimo_encoder = None
        self.parar = False
        self.abortado_por_excursion = False

        if args.simular:
            self.tr = TransporteSimulado(args.motor, semilla=args.semilla)
        elif args.pi3hat:
            self.tr = TransportePi3Hat(args.motor)
        else:
            self.tr = TransporteSocketCAN(args.canal)

    # -- envío ---------------------------------------------------------- #
    def _enviar(self, cmd, datos=b''):
        self.tr.enviar(self.id, cmd, datos)

    def evento(self, texto):
        self.eventos.append((round(time.monotonic(), 3), texto))
        print(f'  [{texto}]', flush=True)

    # -- ciclo de vida del eje ------------------------------------------ #
    def armar(self):
        a = self.args
        self._enviar(proto.CMD_CLEAR_ERRORS)
        # La red de seguridad que sobrevive a que este proceso se muera.
        self._enviar(proto.CMD_SET_LIMITS,
                     trama_limites(a.limite_vel * 1.5, a.limite_corriente))
        self._enviar(proto.CMD_SET_CONTROLLER_MODE,
                     struct.pack('<II', proto.CONTROL_TORQUE, proto.ENTRADA_PASSTHROUGH))
        self._enviar(CMD_SET_INPUT_TORQUE, trama_par(0.0))
        self._enviar(proto.CMD_SET_AXIS_STATE,
                     struct.pack('<I', proto.ESTADO_CLOSED_LOOP))
        self.evento('armado: modo par, lazo cerrado')

    def desarmar(self):
        for _ in range(3):
            try:
                self._enviar(CMD_SET_INPUT_TORQUE, trama_par(0.0))
                self._enviar(proto.CMD_SET_AXIS_STATE,
                             struct.pack('<I', proto.ESTADO_IDLE))
            except Exception:
                pass
        self.evento('desarmado: par 0 + IDLE')

    # -- vigilancia ------------------------------------------------------ #
    def _procesar(self, tramas, t_ahora):
        for t_rx, _nodo, cmd, data in tramas:
            if cmd == proto.CMD_GET_ENCODER_ESTIMATES:
                lectura = parsear_encoder_crudo(data)
                if lectura is None:
                    continue
                pos, vel = lectura
                self.ultimo_encoder = t_ahora
                self.vel_actual = vel
                self.pos_actual = pos
                if self.pos0 is None:
                    self.pos0 = pos
                if abs(vel) > self.args.limite_vel:
                    raise Abortado(f'velocidad {vel:.2f} rev/s > límite '
                                   f'{self.args.limite_vel} rev/s')
                if abs(pos - self.pos0) > self.args.limite_pos:
                    self.abortado_por_excursion = True
                    raise Abortado(f'excursión {pos - self.pos0:+.3f} rev > límite '
                                   f'±{self.args.limite_pos} rev')
                yield t_rx, pos, vel
            elif cmd == proto.CMD_HEARTBEAT:
                hb = proto.parsear_heartbeat(data)
                if hb and hb['error_eje']:
                    raise Abortado(f"axis_error = 0x{hb['error_eje']:08X}")

    def _vigilar_silencio(self, t_ahora):
        if self.ultimo_encoder is None:
            return
        if t_ahora - self.ultimo_encoder > SILENCIO_MAX_S:
            raise Abortado(f'el motor {self.id} lleva '
                           f'{t_ahora - self.ultimo_encoder:.3f} s sin emitir')

    # -- fases ----------------------------------------------------------- #
    def _bucle(self, duracion, par_de_t, etiqueta, condicion_fin=None):
        """Bucle común: manda par a `rate_hz`, drena, vigila y registra.

        `par_de_t(t, dt) -> (par_total, par_excitacion)`. Se registran los dos:
        el ajuste usa el TOTAL (es lo que recibió la planta) y la excitación se
        guarda para poder dibujarla y para saber qué parte puso el regulador.
        """
        periodo = 1.0 / self.args.rate_hz
        t_ini = time.monotonic()
        siguiente = t_ini
        while True:
            ahora = time.monotonic()
            t = ahora - t_ini
            if t >= duracion or self.parar:
                return
            if condicion_fin is not None and condicion_fin(t):
                return
            par, exc = par_de_t(t, periodo)
            par = _sat(par, self.args.par_max)
            self._enviar(CMD_SET_INPUT_TORQUE, trama_par(par))
            siguiente += periodo
            for t_rx, pos, vel in self._procesar(self.tr.drenar(siguiente), ahora):
                # ZOH exacto: la muestra se etiqueta con el par que estaba puesto
                # cuando llegó. El retardo entre "mandado" y "aplicado" lo estima
                # después el ajuste; aquí no se adivina.
                self.filas.append((t_rx, par, exc, pos, vel, etiqueta))
            self._vigilar_silencio(time.monotonic())

    def calentar(self):
        """Par 0 un segundo: fija el cero de posición y confirma que el motor habla."""
        self._bucle(CALENTAMIENTO_S, lambda t, dt: (0.0, 0.0), 'calentamiento')
        if self.ultimo_encoder is None:
            raise Abortado(f'el motor {self.id} no emitió ni una trama de encoder. '
                           f'¿node_id correcto? ¿bus/velocidad correctos? '
                           f'(probar scripts/escanear_can.py)')
        self.evento(f'cero de posición: {self.pos0:+.4f} rev')

    def sondar(self):
        """Estima J invirtiendo el par MIENTRAS EL EJE SIGUE GIRANDO EN EL MISMO SENTIDO.

        Es la única forma de que la fricción se cancele. Con el eje girando en
        sentido positivo, el Coulomb vale siempre -Tc, gire el par hacia donde
        gire, así que las dos aceleraciones son

            a1 = (+A - Tc - b*w - C) / J        (tramo de arranque)
            a2 = (-A - Tc - b*w - C) / J        (tramo de frenado, w AÚN > 0)

        y al restarlas desaparecen Tc, la gravedad y cualquier otro par
        constante C:   J = 2A / (a1 - a2).

        La versión ingenua —acelerar en un sentido y luego en el otro, cada uno
        desde el reposo— NO sirve: al invertir el sentido de giro el Coulomb
        cambia de signo con el par y ya no se cancela; deja un sesgo de
        A/(A-Tc), que con estos motores es un 25 % largo. Es un error fácil de
        cometer y difícil de ver, porque el número que sale parece razonable.

        De ahí que el tramo de frenado sea CORTO (`--t-sonda` x 0,4): tiene que
        acabar antes de que el eje se pare y empiece a girar al revés. El script
        comprueba que la velocidad no cambió de signo en la ventana de medida y
        avisa si lo hizo.

        El J que sale es tosco (±30 % es normal: los dos tramos no promedian la
        misma velocidad, así que el término b*w no se cancela del todo). No se
        usa para calcular ganancias —eso lo hace el ajuste completo— sino para
        dos cosas donde un factor 2 no importa: sintonizar el regulador y
        dimensionar la amplitud de la excitación.
        """
        A = self.args.par_sonda
        T1 = self.args.t_sonda
        # Presupuestos: la sonda para en cuanto gasta un cuarto del recorrido o
        # de la velocidad permitidos, sin esperar a agotar `--t-sonda`. Sin esto
        # la sonda del flipper se come el recorrido entero y el vigilante aborta
        # el ensayo antes de empezar (pasa con los valores por defecto: 0,1 de
        # par durante 0,4 s desplazan más de una vuelta).
        x_max = 0.25 * self.args.limite_pos
        v_max = 0.4 * self.args.limite_vel
        estado = {'fase': 'mas', 't_flip': None}

        def gastado():
            return (self.pos0 is not None
                    and (abs(self.filas[-1][3] - self.pos0) > x_max
                         or abs(self.vel_actual) > v_max)) if self.filas else False

        def ley(t, dt):
            if estado['fase'] == 'mas':
                if t >= T1 or gastado():
                    estado['fase'] = 'menos'
                    estado['t_flip'] = t
                else:
                    return A, A
            return -A, -A

        def fin(t):
            tf = estado['t_flip']
            return tf is not None and t >= tf + 0.4 * tf

        n0 = len(self.filas)
        self._bucle(2 * T1, ley, 'sonda', condicion_fin=fin)
        filas = [f for f in self.filas[n0:] if f[5] == 'sonda']
        if not filas:
            raise Abortado('la sonda no recogió ninguna muestra')
        t0 = filas[0][0]
        t_flip = estado['t_flip']
        if t_flip is None:
            raise Abortado('la sonda no llegó a invertir el par')

        def pendiente(desde, hasta, etiqueta):
            m = [(f[0] - t0, f[4]) for f in filas if desde <= f[0] - t0 < hasta]
            if len(m) < 6:
                raise Abortado(
                    f'la sonda sólo recogió {len(m)} muestras útiles en el tramo '
                    f'"{etiqueta}" (invirtió el par a los {t_flip:.3f} s). Se quedó '
                    f'sin recorrido o sin velocidad demasiado pronto: bajar '
                    f'--par-sonda (ahora {A}) o subir --limite-pos / --limite-vel. '
                    f'Con el motor a 100 Hz de telemetría hacen falta ~0,15 s por '
                    f'tramo; se puede duplicar el muestreo por USB con '
                    f'odrv0.axis0.config.can.encoder_rate_ms = 5.')
            ts = [q[0] for q in m]
            vs = [q[1] for q in m]
            tm, vm = sum(ts) / len(ts), sum(vs) / len(vs)
            sxx = sum((x - tm) ** 2 for x in ts)
            if sxx <= 0:
                raise Abortado(f'sonda: muestras sin separación temporal en "{etiqueta}"')
            return (sum((x - tm) * (y - vm) for x, y in zip(ts, vs)) / sxx, vs)

        # Se descarta el arranque de cada tramo: es el transitorio del escalón de
        # par (retardo del bus + lazo de corriente + ruptura de la fricción).
        T2 = 0.4 * t_flip
        a1, _ = pendiente(0.3 * t_flip, t_flip, 'arranque')
        a2, vs2 = pendiente(t_flip + 0.2 * T2, t_flip + T2, 'frenado')

        if min(vs2) <= 0 <= max(vs2):
            self.evento('AVISO: el eje cruzó el cero durante el frenado de la sonda; '
                        'el J estimado sale alto. Bajar --t-sonda.')

        delta = a1 - a2
        if delta <= 1e-6:
            raise Abortado(
                f'la sonda no movió el motor (pendientes {a1:.3f} y {a2:.3f} rev/s²). '
                f'El par de sonda ({A}) no llega a vencer la fricción de arranque: '
                f'subir --par-sonda.')
        self.J_sonda = 2 * A / delta
        self.evento(f'sonda: J ≈ {self.J_sonda:.5g} par/(rev/s²) '
                    f'(par invertido a los {t_flip:.3f} s, pendientes '
                    f'{a1:+.2f} / {a2:+.2f} rev/s²)')

        # Frenar antes de seguir: la sonda deja el eje con velocidad residual.
        self._bucle(1.0, lambda t, dt: (0.0, 0.0), 'frenado')

    def tantear_amplitud(self):
        """Mide la COMPLIANCIA del flipper a f0 con unos ciclos de prueba, y escala.

        Es lo que sustituye a "medir la rigidez de gravedad k y despejar". Ese
        camino se probó y no funciona: una rampa de par estática da
        (T+ - T-) = k*Δx + 2*Tc, o sea que la fricción de Coulomb NO se cancela
        —se DUPLICA— porque en las dos rampas el eje se mueve en sentidos
        opuestos y el rozamiento se opone a los dos. (En la sonda de inercia sí
        se cancela, y por eso allí el par se invierte SIN que el eje cambie de
        sentido de giro.) Con los números del flipper simulado, la k así medida
        salía 6 veces la real, la amplitud salía enorme y el ensayo se estrellaba
        contra el tope.

        Lo que de verdad hace falta no es k, sino cuánto se mueve el flipper por
        unidad de par A LA FRECUENCIA MÁS BAJA DEL BARRIDO, que es la que más lo
        desplaza. Eso se mide directamente: dos ciclos de seno a f0 con una
        amplitud pequeña, se mira el pico a pico y se escala.

            A = A_tanteo * (2 * excursión_objetivo) / (pico_a_pico medido)

        Mide la planta entera tal como se va a excitar —inercia, muelle, fricción
        y todo— en vez de reconstruirla a partir de un parámetro estimado con un
        ensayo de otra naturaleza. El escalado se limita a un factor 4 para no
        extrapolar de más desde una medida pequeña.
        """
        f0 = self.args.f0
        ciclos = 2.0
        A0 = 0.5 * self.args.par_sonda
        objetivo = 0.35 * self.args.limite_pos
        techo = 0.6 * self.args.limite_pos
        rastro = []

        def ley(t, dt):
            u = A0 * math.sin(2 * math.pi * f0 * t)
            if t < FUNDIDO_S:
                u *= 0.5 * (1.0 - math.cos(math.pi * t / FUNDIDO_S))
            if self.pos_actual is not None:
                rastro.append((t, self.pos_actual - self.pos0))
            return self.regulador(self.vel_actual, dt, self.pos_actual) + u, u

        def fin(t):
            return bool(rastro) and abs(rastro[-1][1]) > techo

        self._bucle(ciclos / f0, ley, 'tanteo', condicion_fin=fin)
        if len(rastro) < 20:
            raise Abortado('el tanteo no recogió muestras suficientes')

        # Sólo el último ciclo completo: el primero lleva el fundido de entrada.
        t_fin = rastro[-1][0]
        ultimo = [x for t, x in rastro if t >= max(0.0, t_fin - 1.0 / f0)]
        pico_a_pico = max(ultimo) - min(ultimo)
        if pico_a_pico < 1e-4:
            raise Abortado(
                f'el tanteo no movió el flipper ({pico_a_pico:.5f} rev con {A0:.4g} de '
                f'par). La fricción de arranque se lo come: subir --par-sonda.')

        factor = min(max((2 * objetivo) / pico_a_pico, 0.25), 4.0)
        self.amplitud_tanteo = A0 * factor
        self.evento(f'tanteo a {f0} Hz: {A0:.4g} de par -> {pico_a_pico:.4f} rev pico a '
                    f'pico; amplitud escalada x{factor:.2f}')

        self._bucle(self.args.t_relevo,
                    lambda t, dt: (self.regulador(self.vel_actual, dt, self.pos_actual),
                                   0.0),
                    'recentrado')

    def dimensionar_amplitud(self):
        """Fija la amplitud de la excitación, por dos caminos distintos.

        En la ORUGA no hay muelle: a la frecuencia más baja del barrido la planta
        es pura inercia, el balanceo de velocidad vale A/(J*w0) y basta despejar
        A para que ese balanceo sea `--swing`. El J de la sonda llega de sobra.

        En el FLIPPER el recurso escaso no es la velocidad sino el RECORRIDO, y
        la planta a baja frecuencia la domina la gravedad, no la inercia. Ahí la
        amplitud la fija el tanteo, que mide la compliancia real a f0 en vez de
        deducirla (ver `tantear_amplitud`).
        """
        if self.args.amplitud is not None:
            return
        if self.args.perfil == 'friccion':
            # La escalera busca el régimen permanente de cada peldaño, no una
            # banda de frecuencias: la amplitud es simplemente el par más alto
            # que se quiere barrer.
            self.args.amplitud = self.args.par_sonda
            self.evento(f'amplitud de la escalera: {self.args.amplitud:.4g}')
            return
        w0 = 2 * math.pi * self.args.f0
        if self.familia == 'flipper':
            A = self.amplitud_tanteo
            detalle = f'excursión objetivo ±{0.35 * self.args.limite_pos:.3f} rev'
        else:
            A = self.args.swing * self.J_sonda * w0
            detalle = f'balanceo objetivo {self.args.swing} rev/s a {self.args.f0} Hz'
        self.args.amplitud = min(A, 0.6 * self.args.par_max)
        self.evento(f'amplitud calculada: {self.args.amplitud:.4g} ({detalle})')

    def asentar(self):
        """Lleva el eje al punto de operación y deja mandando al regulador LENTO.

        En dos etapas, y no por gusto. El regulador lento (0,4 rad/s) es el que
        tiene que estar puesto durante la excitación, para no contaminar la banda
        que se mide; pero es tan lento que, arrancando con el integrador a cero,
        tarda más de un minuto en juntar el par que hace falta para vencer la
        fricción de arranque, y mientras tanto el eje ni se mueve. (Medido en la
        simulación: con 6 s de asentamiento se quedaba en 0,04 de los 3 rev/s
        pedidos, con un par de sostén por debajo del Coulomb.)

        Así que primero manda un regulador RÁPIDO, que alcanza el punto de
        operación en un par de segundos, y después se le pasa su integrador al
        lento: el lento arranca ya con el par de sostén correcto y sólo tiene que
        mantenerlo. El rápido nunca coincide con la excitación, así que no puede
        meterse en la identificación.
        """
        pos_ref = self.pos0 if self.familia == 'flipper' else None
        rapido = Regulador(self.J_sonda, self.args.w_reg * self.args.factor_rapido,
                           self.args.vel_operacion, self.args.par_max, pos_ref=pos_ref)
        self._bucle(self.args.t_asiento,
                    lambda t, dt: (rapido(self.vel_actual, dt, self.pos_actual), 0.0),
                    'asentamiento')

        self.regulador = Regulador(self.J_sonda, self.args.w_reg,
                                   self.args.vel_operacion, self.args.par_max,
                                   pos_ref=pos_ref)
        # El relevo: el lento hereda el par de sostén que encontró el rápido.
        self.regulador.i = rapido(self.vel_actual, 1e-9, self.pos_actual)
        self.regulador.vel_filtrada = rapido.vel_filtrada
        self._bucle(self.args.t_relevo,
                    lambda t, dt: (self.regulador(self.vel_actual, dt, self.pos_actual),
                                   0.0),
                    'relevo')
        deriva = ('' if pos_ref is None else
                  f', desviación {self.pos_actual - self.pos0:+.4f} rev del punto '
                  f'de partida')
        self.evento(f'punto de operación: {self.vel_actual:+.3f} rev/s '
                    f'(objetivo {self.args.vel_operacion}), par de sostén '
                    f'{self.regulador.i:+.4g}{deriva}')

    def excitar(self):
        exc = Excitacion(self.args.perfil, self.args)
        A = self.args.amplitud
        tope = 0.8 * self.args.limite_vel
        estado = {'peldano': -1, 'cortado': False}

        def ley(t, dt):
            u = A * exc(t)
            if self.regulador is None:
                # Escalera de fricción: sin regulador (sostendría el punto de
                # operación, que es justo la incógnita) y con un corte por
                # peldaño. Los peldaños más fuertes embalarían la oruga hasta el
                # límite de velocidad y abortarían el ensayo; en vez de eso, el
                # peldaño que se pasa de vueltas se corta a par cero y se pierde
                # sólo ese punto de la curva, no el ensayo entero.
                i = int(t / self.args.t_paso)
                if i != estado['peldano']:
                    estado['peldano'], estado['cortado'] = i, False
                if abs(self.vel_actual) > tope:
                    estado['cortado'] = True
                if estado['cortado']:
                    return 0.0, 0.0
                return u, u
            return self.regulador(self.vel_actual, dt, self.pos_actual) + u, u

        dur = exc.duracion_total()
        self.evento(f'excitación: {self.args.perfil}, {dur:.1f} s, amplitud {A:.4g}')
        self._bucle(dur, ley, self.args.perfil)

    # -- guardado --------------------------------------------------------- #
    def _consejo_amplitud(self):
        """Qué tocar en la siguiente pasada, a la vista de lo que se movió."""
        filas = [f for f in self.filas if f[5] == self.args.perfil]
        if not filas:
            return []
        vels = [f[4] for f in filas]
        exc = [abs(f[2]) for f in filas]
        balanceo = (max(vels) - min(vels)) / 2
        excursion = max(abs(f[3] - self.pos0) for f in filas)
        consejos = [f'balanceo de velocidad: ±{balanceo:.3f} rev/s']
        if self.familia == 'flipper':
            consejos.append(f'excursión de posición: ±{excursion:.4f} rev '
                            f'(límite ±{self.args.limite_pos:g})')
        if max(exc) > 0:
            if self.familia == 'flipper':
                objetivo = 0.5 * self.args.limite_pos
                if self.abortado_por_excursion:
                    consejos.append(
                        'el ensayo ABORTÓ contra el límite de excursión, así que '
                        'la cifra de arriba está recortada y no sirve para escalar: '
                        f'repetir con --amplitud {self.args.amplitud / 3:.4g} '
                        '(un tercio) o con más --limite-pos.')
                elif excursion > 1e-6:
                    factor = objetivo / excursion
                    consejos.append(
                        f'para llegar a la mitad del límite de excursión, '
                        f'--amplitud {self.args.amplitud * factor:.4g} '
                        f'(x{factor:.2f})')
            if balanceo < 0.05 * self.args.limite_vel:
                consejos.append('APENAS SE MOVIÓ: la fricción de arranque se está '
                                'comiendo la excitación. Correr --perfil friccion '
                                'para medir Tc y usar --amplitud ~3*Tc.')
        return consejos

    def guardar(self, motivo):
        os.makedirs(os.path.dirname(self.args.salida) or '.', exist_ok=True)
        base = self.args.salida
        with open(base, 'w') as f:
            f.write('t_s,par_cmd,par_exc,pos_rev,vel_rev_s,fase\n')
            if self.filas:
                t0 = self.filas[0][0]
                for t, par, e, pos, vel, fase in self.filas:
                    f.write(f'{t - t0:.6f},{par:.6f},{e:.6f},'
                            f'{pos:.6f},{vel:.6f},{fase}\n')

        meta = dict(
            motor=self.id, familia=self.familia, perfil=self.args.perfil,
            amplitud=self.args.amplitud, J_sonda=self.J_sonda, amplitud_tanteo=self.amplitud_tanteo,
            par_sosten=self.regulador.i if self.regulador else None,
            vel_operacion=self.args.vel_operacion, w_reg=self.args.w_reg,
            f0=self.args.f0, f1=self.args.f1, duracion=self.args.duracion,
            t_paso=self.args.t_paso, rate_hz=self.args.rate_hz,
            pos0_rev=self.pos0, muestras=len(self.filas), motivo_fin=motivo,
            simulado=bool(self.args.simular),
            transporte=('simulado' if self.args.simular else
                        'pi3hat' if self.args.pi3hat else f'socketcan:{self.args.canal}'),
            tiempos_del_kernel=self.tr.tiempos_del_kernel,
            unidades='crudas del driver: rev, rev/s, par del driver (ver cabecera)',
            fecha=datetime.now().isoformat(timespec='seconds'),
            eventos=self.eventos,
        )
        if self.args.simular:
            meta['verdad_simulada'] = TransporteSimulado.VERDAD[self.familia]
        with open(base.replace('.csv', '.json'), 'w') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        dt = [self.filas[i + 1][0] - self.filas[i][0] for i in range(len(self.filas) - 1)]
        print(f'\n{len(self.filas)} muestras -> {base}')
        if dt:
            dt_med = sorted(dt)[len(dt) // 2]
            print(f'periodo de muestreo mediano: {dt_med * 1000:.2f} ms '
                  f'({1 / dt_med:.0f} Hz)')
        for linea in self._consejo_amplitud():
            print(f'  {linea}')
        print(f'metadatos -> {base.replace(".csv", ".json")}')

    def correr(self):
        motivo = 'completo'
        try:
            self.armar()
            self.calentar()
            self.sondar()
            if self.args.perfil != 'friccion':
                self.asentar()
                if self.familia == 'flipper':
                    self.tantear_amplitud()
            self.dimensionar_amplitud()
            self.excitar()
        except Abortado as e:
            motivo = f'abortado: {e}'
            print(f'\n!! ABORTA: {e}', file=sys.stderr)
        except KeyboardInterrupt:
            motivo = 'abortado: Ctrl+C'
            print('\n!! Ctrl+C', file=sys.stderr)
        finally:
            self.desarmar()
            self.tr.cerrar()
        self.guardar(motivo)
        return 0 if motivo == 'completo' else 1


# ---------------------------------------------------------------------- #
def main(argv=None):
    p = argparse.ArgumentParser(
        description='Excita un motor en par y registra su respuesta para identificar.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Antes del primer ensayo con hardware: leer la sección de SEGURIDAD '
               'de la cabecera de este archivo y de LEEME.md.')
    p.add_argument('--motor', type=int, required=True,
                   help='node_id (1-4 orugas, 5-8 flippers)')
    p.add_argument('--perfil', default='chirp',
                   choices=['escalon', 'chirp', 'prbs', 'friccion'])
    p.add_argument('--canal', default='can0', help='interfaz SocketCAN (can0, vcan0)')
    p.add_argument('--pi3hat', action='store_true', help='usar los buses del pi3hat')
    p.add_argument('--simular', action='store_true',
                   help='planta simulada, sin hardware (para probar la cadena)')
    p.add_argument('--amplitud', type=float,
                   help='amplitud del par de excitación (por defecto, calculada '
                        'con el J de la sonda en las orugas)')
    p.add_argument('--swing', type=float,
                   help='balanceo de velocidad buscado a f0 [rev/s], para '
                        'dimensionar la amplitud sola')
    p.add_argument('--f0', type=float, help='frecuencia inicial del chirp [Hz]')
    p.add_argument('--f1', type=float, help='frecuencia final del chirp [Hz]')
    p.add_argument('--duracion', type=float, help='duración de la excitación [s]')
    p.add_argument('--t-paso', type=float, default=4.0,
                   help='duración de cada peldaño del perfil friccion [s]; tiene '
                        'que ser varias veces la constante de tiempo J/b o los '
                        'peldaños no llegan al régimen y la curva sale sesgada')
    p.add_argument('--t-sonda', type=float, default=0.4,
                   help='duración máxima de cada tramo del pulso de sonda de inercia [s]')
    p.add_argument('--par-sonda', type=float, help='par del pulso de sonda')
    p.add_argument('--t-asiento', type=float, default=6.0,
                   help='tiempo del regulador rápido para alcanzar el punto de '
                        'operación [s]')
    p.add_argument('--t-relevo', type=float, default=3.0,
                   help='tiempo que se le da al regulador lento tras el relevo [s]')
    p.add_argument('--factor-rapido', type=float, default=12.0,
                   help='cuántas veces más rápido es el regulador de asentamiento')
    p.add_argument('--w-reg', type=float, default=0.4,
                   help='ancho de banda del regulador lento [rad/s]; debe quedar '
                        'una década por debajo de f0')
    p.add_argument('--vel-operacion', type=float,
                   help='velocidad en la que se sostiene el eje [rev/s]')
    p.add_argument('--rate-hz', type=float, default=200.0,
                   help='frecuencia de envío del comando de par')
    p.add_argument('--limite-vel', type=float,
                   help='|vel| máxima antes de abortar [rev/s]')
    p.add_argument('--limite-pos', type=float,
                   help='|excursión| máxima antes de abortar [rev]')
    p.add_argument('--limite-corriente', type=float,
                   help='límite de corriente que se fija por CAN [A]')
    p.add_argument('--par-max', type=float, help='saturación del par mandado')
    p.add_argument('--semilla', type=int, default=1, help='semilla del PRBS / simulación')
    p.add_argument('--salida', help='CSV de salida (por defecto, nombre automático)')
    a = p.parse_args(argv)

    fam = familia(a.motor)
    if fam == 'flipper' and a.perfil == 'friccion':
        sys.exit(
            'El perfil `friccion` es sólo para las orugas (IDs 1-4).\n'
            'La escalera aplica par constante sin regulador —tiene que ser así, '
            'porque\nel punto de operación es justo la incógnita— y en un flipper eso '
            'lo lleva\ncontra el tope en el primer peldaño.\n\n'
            'En el flipper no hace falta: como la excitación cruza el cero de '
            'velocidad,\nTc sale identificado en el propio ensayo de chirp. Correr '
            '`--perfil chirp`.')
    for clave, valor in POR_DEFECTO[fam].items():
        if getattr(a, clave, None) is None:
            setattr(a, clave, valor)
    if a.par_max is None:
        # Techo global: 4 veces el par de sonda deja sitio a la excitación y al
        # par de sostén del regulador sin permitir un disparate.
        a.par_max = 4.0 * a.par_sonda if a.amplitud is None else \
            3.0 * a.amplitud + 2.0 * a.par_sonda

    if 2 * math.pi * a.f0 < 4 * a.w_reg:
        print(f'AVISO: f0 ({a.f0} Hz = {2 * math.pi * a.f0:.1f} rad/s) está demasiado '
              f'cerca del regulador ({a.w_reg} rad/s). Subir --f0 o bajar --w-reg, '
              f'o el regulador se comerá la parte baja del barrido.')

    if a.salida is None:
        sello = datetime.now().strftime('%Y%m%d_%H%M%S')
        sufijo = '_sim' if a.simular else ''
        a.salida = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'datos',
                                f'm{a.motor}_{fam}_{a.perfil}{sufijo}_{sello}.csv')

    print(f'motor {a.motor} ({fam})  perfil {a.perfil}  '
          f'amplitud {a.amplitud if a.amplitud is not None else "(auto)"}')
    print(f'límites: |vel| < {a.limite_vel} rev/s, |Δpos| < {a.limite_pos:g} rev, '
          f'I < {a.limite_corriente} A, |par| < {a.par_max:.4g}')
    if not a.simular:
        # El 8:1 sigue SIN VERIFICAR (ver protocolo_can.RELACION_REDUCCION), así
        # que se enseñan las dos lecturas y decide quien mira el motor.
        if fam == 'flipper':
            g = a.limite_pos * 360.0
            print(f'  ¡OJO! ±{a.limite_pos} rev son ±{g / 8:.0f}° del eje de salida si '
                  f'el encoder mide en el ROTOR (8:1), o ±{g:.0f}° si ya mide en la '
                  f'salida.')
            print('  Dejar el flipper a media carrera y con espacio libre en ambos '
                  'sentidos.')
        else:
            print(f'  ¡OJO! {a.vel_operacion} rev/s son {a.vel_operacion * 60 / 8:.0f} rpm '
                  f'del eje de salida si el encoder mide en el ROTOR (8:1), o '
                  f'{a.vel_operacion * 60:.0f} rpm si ya mide en la salida.')
            print('  La oruga tiene que estar EN EL AIRE y girando libre.')
        try:
            input('Enter para empezar, Ctrl+C para salir... ')
        except KeyboardInterrupt:
            return 1

    ensayo = Ensayo(a)
    signal.signal(signal.SIGTERM, lambda *_: setattr(ensayo, 'parar', True))
    return ensayo.correr()


if __name__ == '__main__':
    sys.exit(main())
