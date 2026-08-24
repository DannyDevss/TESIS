#!/usr/bin/env python3
"""escanear_can.py — Busca motores en los buses del pi3hat sin suponer nada.

`verificar_pi3hat.py --motores` responde "¿contestan los motores donde los
espero?". Este script responde la pregunta de antes: "¿hay ALGÚN motor ahí?",
cuando no contesta nadie y no se sabe si falla el bus, la velocidad o el ID.

Barre, para una velocidad de bus dada, todos los buses de la placa mandando la
trama de lectura de estado (0x9C, NO mueve nada) a un rango de IDs, y muestra
CUALQUIER trama que llegue de vuelta — con su ID de arbitraje crudo, sin dar por
hecho que la respuesta es 0x240+id. Así también se ve un motor que conteste con
otro esquema de IDs.

Correr en la Raspberry, una velocidad por vez (la velocidad se fija al abrir la
placa, así que cada barrido necesita su propio proceso):

    source /opt/ros/jazzy/setup.bash && source ~/TESIS/install/setup.bash
    python3 -u src/ugv_bridge/scripts/escanear_can.py --bitrate 1000000
    python3 -u src/ugv_bridge/scripts/escanear_can.py --bitrate 500000

Por qué manda pocas tramas por ciclo: en un bus donde no hay NADIE, ninguna
trama recibe ACK. El controlador CAN suma 8 al contador de errores de
transmisión por cada intento fallido y a los 255 se va a "bus-off", desde donde
ya no transmite. Con `automatic_retransmission=False` y lotes pequeños se evita
llegar ahí durante el propio barrido.
"""
import argparse
import asyncio
import sys

sys.path.insert(0, __file__.rsplit('/scripts/', 1)[0] + '/..')

from ugv_bridge import pi3hat_backend as backend  # noqa: E402
from ugv_bridge import protocolo_can as proto  # noqa: E402

# Buses físicos del pi3hat (los conectores JST van del 1 al 5).
BUSES = [1, 2, 3, 4, 5]

# Velocidades habituales en drivers RMD/SteadyWin.
BITRATES = [1000000, 500000, 250000, 125000]

# Tramas por ciclo. Ver la nota sobre bus-off en la cabecera.
LOTE = 4


async def barrer(router, bus, ids, timeout):
    """Manda 0x9C a `ids` por `bus` y devuelve las tramas que lleguen."""
    vistas = []
    for i in range(0, len(ids), LOTE):
        lote = ids[i:i + LOTE]
        comandos = []
        for id_motor in lote:
            cmd = backend._moteus.Command()
            cmd.raw = True
            cmd.arbitration_id = proto.id_comando(id_motor)
            cmd.bus = bus
            cmd.data = proto.trama_leer_estado()
            cmd.reply_required = False
            comandos.append(cmd)
        try:
            resultados = await asyncio.wait_for(
                router.cycle(comandos, force_can_check=(1 << bus)),
                timeout=timeout)
        except asyncio.TimeoutError:
            continue
        for r in resultados or []:
            arb = getattr(r, 'arbitration_id', None)
            if arb is None:
                continue
            vistas.append((arb, bytes(getattr(r, 'data', b''))))
    return vistas


async def main_async(args):
    ids = list(range(args.id_min, args.id_max + 1))
    buses = args.buses or BUSES

    print(f'Barriendo buses {buses}, IDs {args.id_min}..{args.id_max}, '
          f'a {args.bitrate // 1000} kbps.')
    print('Trama 0x9C (lectura de estado): no mueve nada.\n')

    # Todos los buses en la misma velocidad, sin reintentos automáticos.
    backend._importar()
    cfg = backend.config_can_clasico(buses)
    if cfg is None:
        print('Esta versión de moteus_pi3hat no expone CanConfiguration; '
              'no se puede fijar la velocidad del bus.')
        return 1
    for c in cfg.values():
        c.slow_bitrate = args.bitrate
        c.fast_bitrate = args.bitrate
        c.automatic_retransmission = False
    router = backend._moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={bus: ids for bus in buses}, can=cfg)

    encontrado = False
    for bus in buses:
        vistas = await barrer(router, bus, ids, args.timeout)
        if not vistas:
            print(f'  bus {bus}: nada')
            continue
        encontrado = True
        for arb, datos in vistas:
            id_motor = proto.motor_de_id_respuesta(arb)
            quien = f'motor {id_motor}' if id_motor is not None else 'ID desconocido'
            print(f'  bus {bus}: RESPUESTA 0x{arb:X} ({quien}) datos={datos.hex()}')

    if not encontrado:
        print(f'\nNadie contestó a {args.bitrate // 1000} kbps. '
              f'Probar otra velocidad:')
        print('  ' + '  '.join(f'--bitrate {b}' for b in BITRATES
                               if b != args.bitrate))
        print('\nSi ninguna velocidad da nada, el problema es físico:')
        print('  - el driver del motor no está energizado (24-48 V, no los 5 V del pi3hat)')
        print('  - CAN_H/CAN_L cruzados, o sin los 120 ohm de terminación')
        print('  - el motor está en otro conector JST del que se está barriendo')
        return 1
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bitrate', type=int, default=1000000,
                   help='velocidad del bus en bit/s (default: 1000000)')
    p.add_argument('--buses', type=int, nargs='+', metavar='N',
                   help=f'buses a barrer (default: {BUSES})')
    p.add_argument('--id-min', type=int, default=1, help='primer ID (default: 1)')
    p.add_argument('--id-max', type=int, default=8, help='último ID (default: 8)')
    p.add_argument('--timeout', type=float, default=1.0,
                   help='techo de tiempo por ciclo, en segundos (default: 1.0)')
    args = p.parse_args()
    sys.exit(asyncio.run(main_async(args)))


if __name__ == '__main__':
    main()
