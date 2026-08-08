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
  3. ¿Cómo compone la librería el ID de arbitraje CAN a partir de source/destination?
     (de eso depende que en el bus aparezca 0x141 y no otro ID, ver
     pi3hat_backend._partir_id_arbitraje)

Con --motores manda además una trama de LECTURA DE ESTADO (0x9C, sin efecto sobre
el movimiento) a los 8 IDs y reporta cuáles contestan y por qué bus. Es la forma
de validar el cableado sin mover nada.
"""
import argparse
import asyncio
import inspect
import math
import sys

sys.path.insert(0, __file__.rsplit('/scripts/', 1)[0] + '/..')

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

    titulo('2. ID DE ARBITRAJE CAN (crítico para el protocolo RMD)')
    # Se busca en el código de la librería cómo se compone el ID, para confirmar
    # que _partir_id_arbitraje() del backend sigue siendo válido.
    encontrado = False
    for nombre, obj in inspect.getmembers(moteus_pi3hat):
        if not inspect.isclass(obj) and not inspect.isfunction(obj):
            continue
        try:
            fuente = inspect.getsource(obj)
        except (OSError, TypeError):
            continue
        for linea in fuente.splitlines():
            if 'source' in linea and ('<< 8' in linea or 'destination' in linea):
                print(f'  {nombre}: {linea.strip()}')
                encontrado = True
    if not encontrado:
        print('  No se pudo leer el código (¿extensión compilada?).')
        print('  Verificar entonces con un analizador CAN que el ID emitido sea 0x141.')
    print('\n  Esperado por pi3hat_backend: arbitration_id = (source << 8) | destination')
    print('  -> para RMD 0x141:  source=0x01, destination=0x41')

    titulo('3. APERTURA DE LA PLACA')
    mapa = MAPA_BUSES if args.motores else {}
    servo_bus_map = {}
    for mid, bus in mapa.items():
        servo_bus_map.setdefault(bus, []).append(mid)
    try:
        router = moteus_pi3hat.Pi3HatRouter(servo_bus_map=servo_bus_map)
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

    titulo('6. SONDEO DE LOS 8 MOTORES (trama 0x9C, no mueve nada)')
    from ugv_bridge import protocolo_can as proto
    from ugv_bridge.pi3hat_backend import _partir_id_arbitraje

    comandos = []
    for mid, bus in sorted(mapa.items()):
        source, dest = _partir_id_arbitraje(proto.id_comando(mid))
        cmd = moteus.Command()
        cmd.destination = dest
        cmd.source = source
        cmd.bus = bus
        cmd.data = proto.trama_leer_estado()
        cmd.reply_required = False
        cmd.raw = True
        comandos.append(cmd)

    mascara = 0
    for bus in set(mapa.values()):
        mascara |= (1 << bus)
    resultados = await router.cycle(comandos, force_can_check=mascara)

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
                   help='sondea los 8 motores (trama de lectura, no mueve nada)')
    args = p.parse_args()
    sys.exit(asyncio.run(main_async(args)))


if __name__ == '__main__':
    main()
