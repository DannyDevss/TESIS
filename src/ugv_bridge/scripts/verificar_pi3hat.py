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

Con --motores manda además una trama de LECTURA DE ESTADO (0x9C, sin efecto sobre
el movimiento) a los 8 IDs y reporta cuáles contestan y por qué bus. Es la forma
de validar el cableado sin mover nada.
"""
import argparse
import asyncio
import math
import sys

sys.path.insert(0, __file__.rsplit('/scripts/', 1)[0] + '/..')

from ugv_bridge import pi3hat_backend as backend  # noqa: E402

MAPA_BUSES = {1: 1, 2: 1, 3: 1, 4: 1, 5: 2, 6: 2, 7: 2, 8: 2}


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

    titulo('2. ID DE ARBITRAJE CAN (crítico para el protocolo RMD)')
    # Prueba DIRECTA: se arma la trama igual que TransportePi3Hat y se le pide a
    # la propia librería que la convierta, para ver con qué ID saldría al bus.
    # Es mucho mas fiable que leer su codigo fuente buscando patrones.
    try:
        from moteus.transport import Transport

        cmd = moteus.Command()
        cmd.raw = True
        cmd.reply_required = False
        cmd.arbitration_id = 0x141
        cmd.bus = 1
        cmd.data = bytes(8)
        trama = Transport._command_to_frame(None, cmd)
        emitido = trama.arbitration_id
        print(f'  ID pedido : 0x141   (0x140 + motor 1)')
        print(f'  ID emitido: 0x{emitido:X}')
        if emitido == 0x141:
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

    titulo(f'6. SONDEO DE {len(mapa)} MOTOR(ES) (trama 0x9C, no mueve nada)')
    from ugv_bridge import protocolo_can as proto

    comandos = []
    for mid, bus in sorted(mapa.items()):
        cmd = moteus.Command()
        cmd.raw = True                                  # ID emitido tal cual
        cmd.arbitration_id = proto.id_comando(mid)      # 0x140 + id_motor
        cmd.bus = bus
        cmd.data = proto.trama_leer_estado()
        cmd.reply_required = False
        comandos.append(cmd)

    mascara = 0
    for bus in set(mapa.values()):
        mascara |= (1 << bus)
    # Techo de tiempo OBLIGATORIO: con force_can_check y ningún motor
    # contestando, el ciclo se queda colgado indefinidamente (comprobado en la
    # placa). Es el mismo techo que aplica TransportePi3Hat en el driver.
    try:
        resultados = await asyncio.wait_for(
            router.cycle(comandos, force_can_check=mascara), timeout=2.0)
    except asyncio.TimeoutError:
        print('  El ciclo del pi3hat venció sin ninguna respuesta.')
        resultados = []

    vistos = {}
    for r in resultados or []:
        arb = getattr(r, 'arbitration_id', None)
        if arb is None:
            continue
        mid = proto.motor_de_id_respuesta(arb)
        if mid is not None:
            vistos[mid] = r

    for mid, bus in sorted(mapa.items()):
        nombre = {1: 'track_fl', 2: 'track_fr', 3: 'track_rl', 4: 'track_rr',
                  5: 'flipper_fl', 6: 'flipper_fr', 7: 'flipper_rl',
                  8: 'flipper_rr'}[mid]
        if mid in vistos:
            estado = proto.parsear_respuesta(bytes(vistos[mid].data))
            print(f'  ID {mid} ({nombre:11s}) bus {bus}: RESPONDE  {estado}')
        else:
            print(f'  ID {mid} ({nombre:11s}) bus {bus}: SIN RESPUESTA')

    faltan = [m for m in mapa if m not in vistos]
    if faltan:
        print(f'\n  {len(faltan)} motor(es) sin responder: {faltan}')
        print('  Revisar: alimentación, terminación del bus, IDs configurados en')
        print('  cada driver, y el mapa de buses (mapa_buses en el launch).')
    else:
        print('\n  Los 8 motores responden. El mapa de buses es correcto.')
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
    args = p.parse_args()
    if args.id or args.bus:      # pedir un ID concreto ya implica sondear
        args.motores = True
    sys.exit(asyncio.run(main_async(args)))


if __name__ == '__main__':
    main()
