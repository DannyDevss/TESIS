#!/usr/bin/env python3
"""verificar_pi3hat.py — Comprobación de la placa pi3hat ANTES de energizar motores.

Correr en la Raspberry Pi:

    python3 src/ugv_bridge/scripts/verificar_pi3hat.py            # sólo lectura
    python3 src/ugv_bridge/scripts/verificar_pi3hat.py --motores  # sondea los 8 motores

Responde las tres preguntas que no se pueden contestar desde el PC de desarrollo,
y de las que depende el modo 'pi3hat' del driver:

  1. ¿Qué versión de moteus_pi3hat hay y qué API expone?
  2. ¿La IMU entrega sólo el cuaternión, o también aceleración lineal cruda?
     (de eso depende si el driver usa el dato real o proyecta la gravedad)
  3. ¿Con qué ID de arbitraje sale al bus una trama `raw` de la librería?
     (de eso depende que en el bus aparezca 0x141 y no otro ID, ver la cabecera
     de pi3hat_backend.py)

Con --motores escucha además el bus unos segundos y reporta qué node_id emiten y
en qué estado están. No transmite nada: los drivers ODrive publican su heartbeat
y su encoder de fábrica. Es la forma de validar el cableado sin mover nada.
"""
import argparse
import asyncio
import math
import sys

sys.path.insert(0, __file__.rsplit('/scripts/', 1)[0] + '/..')

from ugv_bridge import pi3hat_backend as backend  # noqa: E402

# Dos motores por bus, emparejados por esquina: la oruga y el flipper de la
# misma esquina comparten bus. Igual que MAPA_BUSES_POR_DEFECTO del driver.
MAPA_BUSES = {1: 1, 5: 1, 2: 2, 6: 2, 3: 3, 7: 3, 4: 4, 8: 4}


def titulo(t):
    print(f'\n{"=" * 70}\n{t}\n{"=" * 70}')


async def main_async(args):
    titulo('1. LIBRERÍAS')
    try:
        import moteus
        import moteus_pi3hat
    except ImportError as e:
        print(f'  FALTA: {e}')
        print('  Instalar con:  pip3 install moteus moteus-pi3hat')
        return 1
    print(f'  moteus        : {getattr(moteus, "__version__", "?")}')
    print(f'  moteus_pi3hat : {getattr(moteus_pi3hat, "__version__", "?")}')

    import inspect
    firma = inspect.signature(moteus_pi3hat.Pi3HatRouter.__init__).parameters
    print(f'  Pi3HatRouter acepta: {", ".join(firma)}')
    print(f'  CanConfiguration    : '
          f'{"SÍ" if hasattr(moteus_pi3hat, "CanConfiguration") else "NO"}'
          '   (hace falta para apagar CAN-FD)')

    titulo('2. ID DE ARBITRAJE CAN (crítico para el protocolo ODrive)')
    # Prueba DIRECTA: se arma la trama igual que TransportePi3Hat y se le pide a
    # la propia librería que la convierta, para ver con qué ID saldría al bus.
    # Es mucho mas fiable que leer su codigo fuente buscando patrones.
    try:
        from moteus.transport import Transport

        cmd = moteus.Command()
        cmd.raw = True
        cmd.reply_required = False
        cmd.arbitration_id = 0x0AD
        cmd.bus = 1
        cmd.data = bytes(8)
        trama = Transport._command_to_frame(None, cmd)
        emitido = trama.arbitration_id
        print('  ID pedido : 0x0AD   (node_id 5, Set_Input_Vel: (5<<5)|0x0D)')
        print(f'  ID emitido: 0x{emitido:X}')
        if emitido == 0x0AD:
            print('  OK: la librería emite el ID tal cual con raw=True.')
        else:
            print('  ATENCION: el ID emitido NO coincide con el pedido.')
            print('  Revisar TransportePi3Hat.intercambiar en pi3hat_backend.py:')
            print('  esta version de la libreria compone el ID de otra forma.')
    except Exception as e:
        print(f'  No se pudo comprobar automaticamente ({type(e).__name__}: {e}).')
        print('  Verificar con un analizador CAN que el ID emitido sea 0x141.')

    titulo('3. APERTURA DE LA PLACA')
    mapa = MAPA_BUSES if args.motores else {}
    if args.id:
        # Banco de pruebas con UN motor: sondear sólo ese ID evita confundir
        # "no responde" con "no está conectado".
        mapa = {m: b for m, b in mapa.items() if m in args.id}
        if not mapa:
            print(f'  Los IDs {args.id} no están en MAPA_BUSES.')
            return 1
    if args.bus:
        mapa = {m: args.bus for m in mapa}
    servo_bus_map = {}
    for mid, bus in mapa.items():
        servo_bus_map.setdefault(bus, []).append(mid)
    try:
        # Mismo camino que el driver: abre la placa con los buses en CAN 2.0
        # clásico a 1 Mbps, no en el CAN-FD que moteus_pi3hat pone por defecto.
        router = backend.abrir_router(mapa)
        await router.cycle([])
    except Exception as e:
        print(f'  ERROR al abrir el pi3hat: {e}')
        print('  Revisar permisos SPI/GPIO (ver docs de troubleshooting del pi3hat).')
        return 1
    print(f'  Placa abierta. servo_bus_map = {servo_bus_map or "{} (sólo IMU)"}')

    titulo('4. IMU: ¿QUÉ CAMPOS ENTREGA?')
    imu = await router.attitude()
    campos = [a for a in dir(imu) if not a.startswith('_')]
    print(f'  Atributos de attitude(): {campos}')

    tiene_quat = all(hasattr(imu, c) for c in 'wxyz') or hasattr(imu, 'attitude')
    print(f'  Cuaternión de orientación : {"SÍ" if tiene_quat else "NO"}')

    accel = None
    for nombre in ('accel_mps2', 'acceleration_mps2', 'accel'):
        v = getattr(imu, nombre, None)
        if v is not None and hasattr(v, 'x'):
            accel = nombre
            break
    if accel:
        v = getattr(imu, accel)
        print(f'  Aceleración lineal cruda  : SÍ, en "{accel}" '
              f'= ({v.x:.2f}, {v.y:.2f}, {v.z:.2f}) m/s^2')
        print('  -> el driver usará el dato REAL del acelerómetro.')
    else:
        print('  Aceleración lineal cruda  : NO')
        print('  -> el driver proyectará la gravedad según la actitud (fallback).')

    titulo('5. ORIENTACIÓN ACTUAL (para calibrar imu_montaje_*)')
    try:
        w, x, y, z = imu.w, imu.x, imu.y, imu.z
    except AttributeError:
        a = imu.attitude
        w, x, y, z = a.w, a.x, a.y, a.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    print(f'  roll={math.degrees(roll):7.1f}  pitch={math.degrees(pitch):7.1f}  '
          f'yaw={math.degrees(yaw):7.1f}  (grados)')
    print('\n  Con el robot NIVELADO, roll y pitch deben dar ~0.')
    print('  Si roll sale ~±180, poner en config/geometria_robot.yaml:')
    print('      imu_montaje_roll: 3.14159265')

    if not args.motores:
        titulo('LISTO (modo sólo lectura)')
        print('  Para sondear los motores:  --motores')
        return 0

    titulo(f'6. ESCUCHA DE {len(mapa)} MOTOR(ES) (sin transmitir nada)')
    from ugv_bridge import protocolo_can as proto

    # Los drivers ODrive emiten heartbeat (100 ms) y encoder (10 ms) de fábrica,
    # así que no hay que preguntarles: basta con escuchar. Y al no transmitir, el
    # diagnóstico no puede mover un motor ni dejar el bus en bus-off.
    mascara = 0
    for bus in set(mapa.values()):
        mascara |= (1 << bus)

    vistos = {}
    fin_escucha = asyncio.get_event_loop().time() + args.segundos
    while asyncio.get_event_loop().time() < fin_escucha:
        try:
            resultados = await asyncio.wait_for(
                router.cycle([], force_can_check=mascara), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        for r in resultados or []:
            arb = getattr(r, 'arbitration_id', None)
            if arb is None:
                continue
            nodo = proto.nodo_de_id(arb)
            if nodo is not None:
                vistos.setdefault(nodo, {})[proto.cmd_de_id(arb)] = \
                    bytes(getattr(r, 'data', b''))

    nombres = {1: 'track_fl', 2: 'track_fr', 3: 'track_rl', 4: 'track_rr',
               5: 'flipper_fl', 6: 'flipper_fr', 7: 'flipper_rl', 8: 'flipper_rr'}
    for mid, bus in sorted(mapa.items()):
        etiqueta = f'  node {mid} ({nombres.get(mid, "?"):11s}) bus {bus}:'
        mensajes = vistos.get(mid)
        if not mensajes:
            print(f'{etiqueta} SIN SEÑAL')
            continue
        hb = proto.parsear_heartbeat(mensajes.get(proto.CMD_HEARTBEAT, b''))
        enc = proto.parsear_encoder(mensajes.get(proto.CMD_GET_ENCODER_ESTIMATES, b''))
        detalle = []
        if hb:
            estado = {proto.ESTADO_IDLE: 'IDLE',
                      proto.ESTADO_CLOSED_LOOP: 'LAZO CERRADO'}.get(
                          hb['estado_eje'], str(hb['estado_eje']))
            detalle.append(f'estado={estado}')
            if hb['error_eje']:
                detalle.append(f"ERROR=0x{hb['error_eje']:X}")
        if enc:
            detalle.append(f"pos={enc['posicion_rad']:+.3f} rad")
        print(f'{etiqueta} EMITE  {"  ".join(detalle) or "(sin decodificar)"}')

    ajenos = sorted(n for n in vistos if n not in mapa)
    if ajenos:
        print(f'\n  Además emiten node_id que no están en el mapa: {ajenos}')
        print('  Reconfigurar node_id por USB, o corregir el mapa de buses.')

    faltan = [m for m in mapa if m not in vistos]
    if faltan:
        print(f'\n  {len(faltan)} motor(es) sin señal: {faltan}')
        print('  Revisar: alimentación del driver (15-60 V por el XT30),')
        print('  CAN_H/CAN_L en el conector del motor, node_id configurado, y')
        print('  que el CAN esté habilitado (odrv0.config.enable_can_a por USB).')
        print('  Para barrer otras velocidades: scripts/escanear_can.py')
    else:
        print(f'\n  Los {len(mapa)} motores emiten. El mapa de buses es correcto.')
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--motores', action='store_true',
                   help='sondea los motores (trama de lectura, no mueve nada)')
    p.add_argument('--id', type=int, nargs='+', metavar='N',
                   help='sondear sólo estos IDs (banco con un motor suelto)')
    p.add_argument('--bus', type=int, metavar='N',
                   help='forzar el bus del pi3hat de los IDs sondeados')
    p.add_argument('--segundos', type=float, default=3.0,
                   help='cuánto escuchar a los motores (default: 3)')
    args = p.parse_args()
    if args.id or args.bus:      # pedir un ID concreto ya implica sondear
        args.motores = True
    sys.exit(asyncio.run(main_async(args)))


if __name__ == '__main__':
    main()
