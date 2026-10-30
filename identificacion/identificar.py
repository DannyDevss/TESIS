#!/usr/bin/env python3
"""identificar.py — Ajusta el modelo de la planta y calcula las ganancias PI.

Come los CSV de `registrar_ensayo.py` y devuelve (a) los parámetros físicos del
motor con su carga y (b) los valores de `vel_gain`, `vel_integrator_gain` y
`pos_gain` que hay que escribir en el driver. Es el gemelo en Python de
`identificar.m`: mismo método y mismos números, para no depender de MATLAB ni de
la System Identification Toolbox.

---------------------------------------------------------------------------
MODELOS (uno por familia, y no son intercambiables)
---------------------------------------------------------------------------
Oruga (IDs 1-4), primer orden — la rueda gira libre, no hay término de posición:

        J*w' + b*w + Tc*sgn(w) = u

Flipper (IDs 5-8), segundo orden — la GRAVEDAD actúa como un muelle. Alrededor
del ángulo de trabajo x0, el par de gravedad m*g*l*sin(x) se linealiza en
k*(x-x0) con k = m*g*l*cos(x0), y eso convierte la planta en resonante:

        J*w' + b*w + k*(x-x0) + Tc*sgn(w) = u

Ignorar esa k y ajustar el flipper como si fuera de primer orden es el error que
hace que las ganancias del flipper "no acaben de ir": el modelo predice una
planta que no resuena y la real sí.

---------------------------------------------------------------------------
MÉTODO: MÍNIMOS CUADRADOS SOBRE LA ECUACIÓN INTEGRADA
---------------------------------------------------------------------------
La vía obvia es derivar la velocidad para tener w' y hacer mínimos cuadrados.
Es mala idea: derivar amplifica el ruido justo en la banda alta, que es donde el
chirp aporta la información, y el resultado depende del filtro que se elija.

Aquí se integra la ecuación una vez en lugar de derivarla:

        J*(w-w0) + b*∫w + k*∫(x-x0) + Tc*∫tanh(w/eps) - u_off*t = ∫u

Todos los regresores son integrales de señales MEDIDAS, y la integración
promedia el ruido en vez de amplificarlo. Los parámetros salen de un sistema
lineal, sin optimización iterativa ni valores iniciales que ajustar a mano.

  - `tanh(w/eps)` en vez de `sgn(w)`: el escalón de Coulomb en w=0 no es
    identificable con datos ruidosos; la tangente hiperbólica lo suaviza sin
    cambiar el comportamiento lejos del cero.
  - `u_off`: un par constante que no se modeló (el sesgo de gravedad del flipper
    mal compensado, un rozamiento asimétrico). Sin este término, el ajuste lo
    reparte entre J y b y los sesga a los dos.

---------------------------------------------------------------------------
VALIDACIÓN
---------------------------------------------------------------------------
El ajuste NO se valida con los datos del ajuste. `--validar` toma un segundo
ensayo, hecho con otra señal (chirp para ajustar, PRBS para validar), simula el
modelo con SU entrada y compara con SU salida medida. El número que sale, el
"fit" en porcentaje, es el que se puede enseñar: un modelo que ajusta al 95 % la
señal con la que se ajustó y al 40 % una distinta, no vale.

---------------------------------------------------------------------------
USO
---------------------------------------------------------------------------
    # Cadena entera sin hardware, para comprobar que el análisis funciona:
    python3 identificacion/registrar_ensayo.py --motor 1 --perfil chirp --simular
    python3 identificacion/registrar_ensayo.py --motor 1 --perfil prbs  --simular
    python3 identificacion/identificar.py identificacion/datos/m1_*chirp_sim*.csv \
        --validar identificacion/datos/m1_*prbs_sim*.csv

    # Con datos reales, pidiendo un ancho de banda concreto:
    python3 identificacion/identificar.py datos/m1_oruga_chirp_2026....csv \
        --validar datos/m1_oruga_prbs_2026....csv --bw 120

    # Fricción (perfil `friccion`), que va aparte:
    python3 identificacion/identificar.py datos/m1_oruga_friccion_....csv
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

EPS_COULOMB = 0.02      # rev/s; ancho del suavizado de sgn(w)


# ---------------------------------------------------------------------- #
# Carga y remuestreo
# ---------------------------------------------------------------------- #
def cargar(ruta):
    """CSV + JSON -> dict con las señales ya en rejilla uniforme.

    El remuestreo NO es cosmético: las tramas del motor llegan cada ~10 ms pero
    con jitter, y todo lo que viene después (integración, FFT, simulación)
    supone paso constante. Se remuestrea al periodo MEDIANO, que es robusto a
    las tramas perdidas de vez en cuando; si se usara la media, una sola pausa
    larga desplazaría toda la rejilla.
    """
    datos = np.genfromtxt(ruta, delimiter=',', names=True,
                          dtype=None, encoding='utf-8')
    meta_ruta = ruta.replace('.csv', '.json')
    meta = json.load(open(meta_ruta)) if os.path.exists(meta_ruta) else {}

    fase = np.array([str(f) for f in datos['fase']])
    perfil = meta.get('perfil', 'desconocido')
    # Sólo la fase de excitación. El calentamiento, la sonda de inercia, el
    # frenado y el asentamiento del regulador son preparación, no datos: la
    # sonda además mete escalones de par que el modelo no debe ver.
    sel = np.isin(fase, [perfil])
    if sel.sum() < 50:
        sys.exit(f'{ruta}: sólo {sel.sum()} muestras de excitación. '
                 f'¿El ensayo abortó? Mirar "motivo_fin" en el JSON.')

    t = datos['t_s'][sel].astype(float)
    # La entrada del modelo es el par TOTAL, no sólo la excitación: durante el
    # ensayo el regulador lento también mete par, y la planta recibe la suma. Si
    # se ajustara contra `par_exc`, ese par de sostén aparecería como un error de
    # modelo y se repartiría entre J y b.
    u = datos['par_cmd'][sel].astype(float)
    u_exc = datos['par_exc'][sel].astype(float)
    x = datos['pos_rev'][sel].astype(float)
    w = datos['vel_rev_s'][sel].astype(float)
    t -= t[0]

    dt = np.median(np.diff(t))
    n = int(t[-1] / dt) + 1
    tu = np.arange(n) * dt
    # La entrada es escalonada (ZOH): interpolarla linealmente inventaría rampas
    # que el motor nunca vio. Se toma el último valor vigente.
    idx = np.searchsorted(t, tu, side='right') - 1
    idx = np.clip(idx, 0, len(t) - 1)

    return dict(ruta=ruta, meta=meta, perfil=perfil,
                familia=meta.get('familia', 'oruga'),
                t=tu, dt=dt, fs=1.0 / dt,
                u=u[idx], u_exc=u_exc[idx],
                x=np.interp(tu, t, x), w=np.interp(tu, t, w),
                motivo=meta.get('motivo_fin', '?'))


def _integrar(y, dt):
    """Integral acumulada (trapecios), empezando en 0."""
    out = np.zeros_like(y)
    out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1])) * dt
    return out


# ---------------------------------------------------------------------- #
# Ajuste
# ---------------------------------------------------------------------- #
def ajustar(d):
    """Mínimos cuadrados sobre la ecuación integrada -> parámetros físicos."""
    t, dt, u, x, w = d['t'], d['dt'], d['u'], d['x'], d['w']
    x0, w0 = x[0], w[0]

    col_J = w - w0
    col_b = _integrar(w, dt)
    col_k = _integrar(x - x0, dt)
    col_Tc = _integrar(np.tanh(w / EPS_COULOMB), dt)
    col_off = -t
    objetivo = _integrar(u, dt)

    # El Coulomb sólo es identificable si el eje CRUZA el cero de velocidad. Si
    # gira siempre en el mismo sentido —que es justo lo que se busca en la oruga,
    # para que la fricción sea localmente lineal— entonces tanh(w/eps) vale ~1
    # todo el rato y su columna es indistinguible de la del par constante. Meter
    # las dos haría el sistema casi singular y el reparto entre ambas, arbitrario.
    cruza_cero = (np.max(w) > 3 * EPS_COULOMB) and (np.min(w) < -3 * EPS_COULOMB)

    nombres = ['J', 'b']
    columnas = [col_J, col_b]
    if d['familia'] == 'flipper':
        nombres.append('k')
        columnas.append(col_k)
    if cruza_cero:
        nombres.append('Tc')
        columnas.append(col_Tc)
    nombres.append('u_off')
    columnas.append(col_off)
    A = np.column_stack(columnas)

    theta, *_ = np.linalg.lstsq(A, objetivo, rcond=None)
    p = dict(zip(nombres, theta))
    p.setdefault('k', 0.0)
    p.setdefault('Tc', 0.0)
    p['cruza_cero'] = bool(cruza_cero)

    # Número de condición: si dispara, los regresores no son independientes en
    # estos datos (típico: excitación demasiado lenta, w y ∫w casi paralelos) y
    # los parámetros son un reparto arbitrario aunque el ajuste "cuadre".
    p['cond'] = float(np.linalg.cond(A))
    p['nombres'] = nombres
    return p


def simular(p, u, dt, familia, w0=0.0, x0=0.0):
    """Integra el modelo con la entrada `u` (Heun). Devuelve la velocidad.

    `w0` NO es opcional en la práctica: el ensayo arranca en el punto de
    operación que dejó el regulador (3 rev/s en la oruga), no parado. Simular
    desde cero contra unos datos que empiezan a 3 rev/s da un error enorme y un
    "fit" ridículo que no dice nada del modelo.
    """
    J = p['J']
    b, k, Tc, off = p['b'], p.get('k', 0.0), p['Tc'], p.get('u_off', 0.0)

    def acel(u_i, w, x):
        return (u_i + off - b * w - k * x - Tc * math.tanh(w / EPS_COULOMB)) / J

    w = np.zeros(len(u))
    x = np.zeros(len(u))
    w[0], x[0] = w0, x0
    for i in range(len(u) - 1):
        a1 = acel(u[i], w[i], x[i])
        wp, xp = w[i] + a1 * dt, x[i] + w[i] * dt
        a2 = acel(u[i + 1], wp, xp)
        w[i + 1] = w[i] + 0.5 * (a1 + a2) * dt
        x[i + 1] = x[i] + 0.5 * (w[i] + wp) * dt
    return w


def simular_ensayo(p, d):
    """Simulación libre sobre un ensayo, arrancando de SU punto de operación."""
    return simular(p, d['u'], d['dt'], d['familia'], w0=d['w'][0], x0=0.0)


def simular_por_ventanas(p, d, horizonte=2.0):
    """Simulación reenganchada al estado medido cada `horizonte` segundos.

    La simulación libre de 25 s es una prueba dura y algo injusta para estas
    plantas: con b/J del orden de 2 rad/s, la oruga es casi un integrador y su
    ganancia en continua es enorme, así que un error minúsculo en el par
    constante se traduce en una deriva grande de velocidad al cabo de medio
    minuto. El modelo puede describir perfectamente la DINÁMICA y aun así sacar
    un "fit" pobre por esa deriva.

    Reenganchando el estado cada pocos segundos se mide lo que de verdad importa
    para sintonizar un lazo: si el modelo predice la respuesta en horizontes
    comparables a la constante de tiempo de la planta. Se reportan los dos
    números, y el bueno para decidir es este.
    """
    n = max(int(horizonte / d['dt']), 10)
    salida = np.empty_like(d['w'])
    for i0 in range(0, len(d['w']), n):
        i1 = min(i0 + n, len(d['w']))
        salida[i0:i1] = simular(p, d['u'][i0:i1], d['dt'], d['familia'],
                                w0=d['w'][i0], x0=d['x'][i0] - d['x'][0])
    return salida


def fit_porcentaje(y, y_est):
    """El "fit" de MATLAB (`compare`): 100*(1 - |y-y_est| / |y-media(y)|)."""
    err = np.linalg.norm(y - y_est)
    ref = np.linalg.norm(y - np.mean(y))
    return 100.0 * (1.0 - err / ref) if ref > 0 else float('nan')


# ---------------------------------------------------------------------- #
# Fricción (perfil `friccion`)
# ---------------------------------------------------------------------- #
def analizar_friccion(d):
    """De cada peldaño de par, la velocidad en régimen -> recta b*w + Tc.

    Se descarta el primer 60 % de cada peldaño: es el transitorio. Lo que
    interesa es el punto de equilibrio, donde w'=0 y el par entra entero contra
    la fricción.
    """
    u, w, t = d['u'], d['w'], d['t']
    cambios = np.flatnonzero(np.abs(np.diff(u)) > 1e-9) + 1
    bordes = np.concatenate([[0], cambios, [len(u)]])

    pares, vels, descartados = [], [], []
    for i0, i1 in zip(bordes[:-1], bordes[1:]):
        if i1 - i0 < 20:
            continue
        ini = i0 + int(0.6 * (i1 - i0))
        w_ss = np.mean(w[ini:i1])
        # Aún acelerando -> ese peldaño no llegó al régimen, no sirve. El umbral
        # es exigente (2 % de la velocidad por segundo) porque un peldaño a medio
        # asentar mide una velocidad más baja de la que le toca, y eso INFLA b y
        # hunde Tc: con peldaños de 1,5 s sobre una planta de 0,6 s de constante
        # de tiempo, el error en Tc pasaba del 25 %.
        deriva = np.polyfit(t[ini:i1], w[ini:i1], 1)[0]
        if abs(deriva) > 0.02 * max(abs(w_ss), 0.05):
            descartados.append((u[i0], w_ss, deriva))
            continue
        pares.append(u[i0])
        vels.append(w_ss)

    pares, vels = np.array(pares), np.array(vels)
    moviendo = np.abs(vels) > 3 * EPS_COULOMB
    if moviendo.sum() < 3:
        return dict(pares=pares, vels=vels, descartados=descartados,
                    b=float('nan'), Tc=float('nan'),
                    aviso=f'sólo {moviendo.sum()} peldaños en régimen y en movimiento '
                          f'({len(descartados)} descartados por no asentarse). '
                          f'Subir --t-paso, o --amplitud si es que no se movía.')

    A = np.column_stack([vels[moviendo], np.sign(vels[moviendo])])
    (b, Tc), *_ = np.linalg.lstsq(A, pares[moviendo], rcond=None)
    return dict(pares=pares, vels=vels, descartados=descartados,
                b=float(b), Tc=float(Tc), aviso=None)


# ---------------------------------------------------------------------- #
# Ganancias
# ---------------------------------------------------------------------- #
def ganancias(J, wc, familia, k=0.0):
    """De la inercia identificada a los tres números del driver.

    El lazo de velocidad del driver es  T = vel_gain*(w_ref - w), con w en rev/s
    y T en las unidades de par del driver. La planta, en ESAS MISMAS unidades,
    es 1/(J*s). El lazo abierto queda vel_gain/(J*s) y su cruce por 0 dB es

        wc = vel_gain / J        =>   vel_gain = J * wc          [wc en rad/s]

    Sin 2*pi y sin la reducción, porque J se identificó en unidades del driver
    (ver la cabecera de `registrar_ensayo.py`). Si se quisiera el J del SI en el
    mismo eje: J_SI = J / (2*pi).

    El integrador se coloca por MARGEN DE FASE, no por receta: el cero del PI en
    wc/4 quita ~14° de fase en el cruce, que es un peaje asumible a cambio de
    matar el error estacionario.

        vel_integrator_gain = (wc/4) * vel_gain

    El manual del GIM6010-8 propone `0,5 * ancho_de_banda * vel_gain` sin decir
    si el ancho de banda va en Hz o en rad/s, y la diferencia entre las dos
    lecturas es un factor 2*pi. Se imprimen las dos para comparar: la de rad/s
    sale agresiva (cero en wc/2) y la de Hz, conservadora (cero en wc/12).

    El lazo de posición es proporcional puro y entrega velocidad
    (w_ref = pos_gain * error_de_posicion), así que pos_gain está en 1/s y, con
    el lazo de velocidad bastante más rápido, el ancho de banda de posición es
    aproximadamente su propio valor. Se deja en wc/4 para respetar la separación
    entre lazos en cascada.
    """
    vel_gain = J * wc
    vig = 0.25 * wc * vel_gain
    pos_gain = wc / 4.0
    g = dict(wc=wc, vel_gain=vel_gain, vel_integrator_gain=vig, pos_gain=pos_gain,
             vig_manual_rads=0.5 * wc * vel_gain,
             vig_manual_hz=0.5 * (wc / (2 * math.pi)) * vel_gain)
    if familia == 'flipper' and k > 0:
        # Caída bajo gravedad con el lazo P de posición: en régimen la velocidad
        # es 0, así que todo el par sale del error de posición.
        g['rigidez_par_por_rev'] = vel_gain * pos_gain
        g['wn_planta'] = math.sqrt(k / J)
    return g


# ---------------------------------------------------------------------- #
# Informe
# ---------------------------------------------------------------------- #
def informe(d, p, val, args):
    fam = d['familia']
    print('=' * 72)
    print(f"ENSAYO   {os.path.basename(d['ruta'])}")
    print(f"  motor {d['meta'].get('motor', '?')} ({fam}), perfil {d['perfil']}, "
          f"{len(d['t'])} muestras a {d['fs']:.0f} Hz ({d['dt'] * 1000:.1f} ms)")
    print(f"  fin del ensayo: {d['motivo']}")
    if d['meta'].get('simulado'):
        print('  *** DATOS SIMULADOS: sirven para probar la cadena, no para la tesis ***')
    print()

    print('PARÁMETROS IDENTIFICADOS (unidades del driver: rev, rev/s, par del driver)')
    print(f"  J   = {p['J']:.6g}      inercia          [par / (rev/s^2)]")
    print(f"  b   = {p['b']:.6g}      fricción viscosa [par / (rev/s)]")
    if fam == 'flipper':
        print(f"  k   = {p['k']:.6g}      rigidez de gravedad [par / rev]")
    if p['cruza_cero']:
        print(f"  Tc  = {p['Tc']:.6g}      fricción de Coulomb [par]")
    else:
        print('  Tc  = (no identificable: el eje no cruzó el cero de velocidad;')
        print('        está absorbido en `off`. Para medirlo, --perfil friccion)')
    print(f"  off = {p['u_off']:.6g}      par constante no modelado")
    print(f"  J en unidades SI del mismo eje: {p['J'] / (2 * math.pi):.6g} kg·m²")

    polo = p['b'] / p['J']
    print(f"  polo mecánico b/J = {polo:.2f} rad/s ({polo / (2 * math.pi):.2f} Hz)")
    if fam == 'flipper' and p['k'] > 0:
        wn = math.sqrt(p['k'] / p['J'])
        zeta = p['b'] / (2 * math.sqrt(p['k'] * p['J']))
        print(f"  resonancia de gravedad wn = {wn:.2f} rad/s "
              f"({wn / (2 * math.pi):.2f} Hz), zeta = {zeta:.3f}")
    print(f"  cond(A) = {p['cond']:.3g}"
          + ('   <-- ALTO: regresores casi dependientes, desconfiar del reparto '
             'entre J y b (excitar más alto en frecuencia)' if p['cond'] > 1e4 else ''))

    verdad = d['meta'].get('verdad_simulada')
    if verdad:
        print('\n  Contraste con la planta simulada (esto es la prueba del análisis):')
        for clave in ('J', 'b', 'k', 'Tc'):
            if clave == 'Tc' and not p['cruza_cero']:
                continue
            if clave in verdad and (clave != 'k' or fam == 'flipper'):
                real, est = verdad[clave], p.get(clave, 0.0)
                err = 100 * (est - real) / real if real else float('nan')
                print(f'    {clave:3s} real {real:<10.6g} estimado {est:<10.6g} '
                      f'({err:+.1f} %)')

    print('\nCALIDAD DEL AJUSTE  (fit %, como el `compare` de MATLAB)')
    print(f"  {'':28} {'libre':>8} {'por ventanas de ' + f'{args.horizonte:g} s':>20}")
    print(f"  {'sobre los propios datos':28} "
          f"{fit_porcentaje(d['w'], simular_ensayo(p, d)):8.1f} "
          f"{fit_porcentaje(d['w'], simular_por_ventanas(p, d, args.horizonte)):20.1f}")
    if val:
        dv = val
        print(f"  {'VALIDACIÓN (' + dv['perfil'] + ')':28} "
              f"{fit_porcentaje(dv['w'], simular_ensayo(p, dv)):8.1f} "
              f"{fit_porcentaje(dv['w'], simular_por_ventanas(p, dv, args.horizonte)):20.1f}")
        print(f"  <- {os.path.basename(dv['ruta'])}")
        print('  El número que vale es el de VALIDACIÓN por ventanas: señal distinta')
        print('  a la del ajuste, y sin la deriva que castiga a la simulación libre')
        print('  en una planta casi integradora. Por debajo del ~70 % desconfiar.')
    else:
        print('VALIDACIÓN: no se pidió. Correr otro ensayo con --perfil prbs y pasarlo')
        print('  con --validar; el fit sobre los propios datos siempre sale bien.')

    print('\nGANANCIAS PROPUESTAS')
    wc_max = 2 * math.pi * d['fs'] / 10
    print(f"  (el muestreo de {d['fs']:.0f} Hz deja usar hasta ~{wc_max:.0f} rad/s "
          f"de ancho de banda con holgura)")
    print(f"  {'wc [rad/s]':>11} {'[Hz]':>7} {'vel_gain':>12} "
          f"{'vel_integrator_gain':>21} {'pos_gain':>10}")
    for wc in args.bw:
        g = ganancias(p['J'], wc, fam, p['k'])
        marca = '  <-- elegido' if wc == args.bw[0] else ''
        print(f"  {wc:11.0f} {wc / (2 * math.pi):7.1f} {g['vel_gain']:12.5g} "
              f"{g['vel_integrator_gain']:21.5g} {g['pos_gain']:10.4g}{marca}")

    g = ganancias(p['J'], args.bw[0], fam, p['k'])
    print(f"\n  Para wc = {args.bw[0]:.0f} rad/s, la fórmula del manual daría")
    print(f"    vel_integrator_gain = {g['vig_manual_rads']:.5g}  (leyendo su "
          f"'ancho de banda' en rad/s: agresivo)")
    print(f"    vel_integrator_gain = {g['vig_manual_hz']:.5g}  (leyéndolo en Hz: "
          f"conservador)")
    print(f"    aquí se propone      {g['vel_integrator_gain']:.5g}  (cero del PI en wc/4)")

    print('\n  Comandos para el driver:')
    print(f"    odrv0.axis0.controller.config.vel_gain = {g['vel_gain']:.5g}")
    print(f"    odrv0.axis0.controller.config.vel_integrator_gain = "
          f"{g['vel_integrator_gain']:.5g}")
    if fam == 'flipper':
        print(f"    odrv0.axis0.controller.config.pos_gain = {g['pos_gain']:.5g}")
        if p['k'] > 0:
            caida = p['k'] / (g['vel_gain'] * g['pos_gain'])
            print(f"\n  Caída del flipper bajo su propio peso con el lazo P de posición:")
            print(f"    ~{caida:.4f} rev por rev de desviación respecto al equilibrio")
            print(f"    ({caida * 100:.1f} %). Si molesta, es lo que arregla el "
                  f"integrador de velocidad, no subir pos_gain.")
    else:
        print(f"    # pos_gain no se toca: las orugas van en velocidad "
              f"(CONTROL_VELOCIDAD en driver_movimiento)")

    print('\n  Estos números son el PUNTO DE PARTIDA medido, no el final. En el motor:')
    print('  verificar con un escalón de velocidad y, si hace falta, recortar. La')
    print('  carga real de la oruga cambia con el terreno; la del flipper, con el ángulo.')
    print('=' * 72)


def informe_friccion(d, r):
    print('=' * 72)
    print(f"FRICCIÓN  {os.path.basename(d['ruta'])}  (motor {d['meta'].get('motor', '?')})")
    print(f"  {len(r['pares'])} peldaños en régimen estacionario, "
          f"{len(r['descartados'])} descartados por no llegar a asentarse")
    if r['aviso']:
        print(f"  AVISO: {r['aviso']}")
    else:
        print(f"\n  b  = {r['b']:.6g}   fricción viscosa [par / (rev/s)]")
        print(f"  Tc = {r['Tc']:.6g}   fricción de Coulomb [par]")
        print(f"\n  Par mínimo para arrancar el eje: ~{abs(r['Tc']):.4g}. Por debajo de")
        print('  ese par el motor no se mueve, y ninguna ganancia arregla eso: es el')
        print('  suelo de la zona muerta del reductor 8:1.')
    print(f"\n  {'par':>10} {'vel en régimen [rev/s]':>24}")
    for u, w in zip(r['pares'], r['vels']):
        print(f"  {u:10.4f} {w:24.4f}")
    print('=' * 72)


# ---------------------------------------------------------------------- #
def graficar(d, p, val, r_fric, destino):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(sin matplotlib: no se generan gráficas)')
        return

    if r_fric is not None:
        fig, ax = plt.subplots(figsize=(7, 4.5))
        ax.plot(r_fric['vels'], r_fric['pares'], 'o', label='peldaños medidos')
        if not math.isnan(r_fric['b']):
            w = np.linspace(min(r_fric['vels']), max(r_fric['vels']), 200)
            ax.plot(w, r_fric['b'] * w + r_fric['Tc'] * np.sign(w), '-',
                    label=f"b·w + Tc·sgn(w)   b={r_fric['b']:.4g}, Tc={r_fric['Tc']:.4g}")
        ax.set_xlabel('velocidad en régimen [rev/s]')
        ax.set_ylabel('par aplicado')
        ax.set_title(f"Curva de fricción — motor {d['meta'].get('motor', '?')}")
        ax.grid(alpha=.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(destino, dpi=130)
        print(f'gráfica -> {destino}')
        return

    w_est = simular_por_ventanas(p, d, 2.0)
    fig, axs = plt.subplots(3, 1, figsize=(9, 9))

    axs[0].plot(d['t'], d['u'], lw=.8, label='par total (entrada del modelo)')
    axs[0].plot(d['t'], d['u_exc'], lw=.6, alpha=.7, label='sólo la excitación')
    axs[0].legend(fontsize=8)
    axs[0].set_ylabel('par')
    axs[0].set_title(f"{os.path.basename(d['ruta'])} — entrada")
    axs[0].grid(alpha=.3)

    axs[1].plot(d['t'], d['w'], lw=.8, label='medido')
    axs[1].plot(d['t'], w_est, lw=.8,
                label=f"modelo, ventanas de 2 s (fit {fit_porcentaje(d['w'], w_est):.1f} %)")
    axs[1].set_ylabel('velocidad [rev/s]')
    axs[1].set_xlabel('t [s]')
    axs[1].legend()
    axs[1].grid(alpha=.3)

    # Respuesta en frecuencia empírica (estimador H1: Pyu/Puu) contra el modelo.
    # Es la comprobación que descubre los desajustes que el fit temporal esconde:
    # si el modelo falla sólo en la banda alta, en el tiempo casi no se nota.
    try:
        from scipy import signal as sg
        nper = min(1024, len(d['t']) // 4)
        f, Puu = sg.welch(d['u_exc'], d['fs'], nperseg=nper)
        _, Pyu = sg.csd(d['u_exc'], d['w'], d['fs'], nperseg=nper)
        valido = (Puu > Puu.max() * 1e-3) & (f > 0)
        H = np.abs(Pyu[valido] / Puu[valido])
        axs[2].loglog(f[valido], H, lw=.9, label='medido (H1)')
        s = 2j * np.pi * f[valido]
        Hm = np.abs(s / (p['J'] * s ** 2 + p['b'] * s + p['k'])) \
            if d['familia'] == 'flipper' else np.abs(1 / (p['J'] * s + p['b']))
        axs[2].loglog(f[valido], Hm, lw=.9, label='modelo')
        axs[2].set_xlabel('f [Hz]')
        axs[2].set_ylabel('|w/u|')
        axs[2].legend()
        axs[2].grid(alpha=.3, which='both')
    except ImportError:
        axs[2].text(.5, .5, 'scipy no disponible', ha='center')

    if val:
        wv = simular_por_ventanas(p, val, 2.0)
        axs[1].set_title(f"validación cruzada: fit {fit_porcentaje(val['w'], wv):.1f} % "
                         f"sobre {os.path.basename(val['ruta'])}")
    fig.tight_layout()
    fig.savefig(destino, dpi=130)
    print(f'gráfica -> {destino}')


def _unico(patron):
    """Acepta comodines para no tener que copiar el sello de tiempo a mano."""
    coincidencias = sorted(glob.glob(patron)) if any(c in patron for c in '*?[') else [patron]
    if not coincidencias:
        sys.exit(f'no existe: {patron}')
    if len(coincidencias) > 1:
        print(f'(el patrón casa con {len(coincidencias)}; se usa el más reciente: '
              f'{os.path.basename(coincidencias[-1])})')
    return coincidencias[-1]


def main(argv=None):
    p = argparse.ArgumentParser(
        description='Ajusta el modelo de la planta y calcula vel_gain / '
                    'vel_integrator_gain / pos_gain.')
    p.add_argument('csv', help='CSV del ensayo de ajuste (admite comodines)')
    p.add_argument('--validar', help='CSV de otro ensayo, con OTRA señal')
    p.add_argument('--bw', type=float, nargs='+', default=[100.0, 50.0, 150.0, 200.0],
                   help='anchos de banda a tabular [rad/s]; el primero es el elegido')
    p.add_argument('--horizonte', type=float, default=2.0,
                   help='horizonte de la simulación por ventanas [s]')
    p.add_argument('--sin-grafica', action='store_true')
    a = p.parse_args(argv)

    d = cargar(_unico(a.csv))

    if d['perfil'] == 'friccion':
        r = analizar_friccion(d)
        informe_friccion(d, r)
        if not a.sin_grafica:
            graficar(d, None, None, r, d['ruta'].replace('.csv', '.png'))
        return 0

    par = ajustar(d)

    val = cargar(_unico(a.validar)) if a.validar else None

    informe(d, par, val, a)
    if not a.sin_grafica:
        graficar(d, par, val, None, d['ruta'].replace('.csv', '.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
