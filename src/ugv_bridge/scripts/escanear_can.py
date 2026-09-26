#!/usr/bin/env python3
"""escanear_can.py — Busca motores en los buses del pi3hat sin suponer nada.

`verificar_pi3hat.py --motores` responde "¿están los motores donde los espero?".
Este script responde la pregunta de antes: "¿hay ALGÚN motor ahí?", cuando no
aparece nadie y no se sabe si falla el bus, la velocidad o el node_id.

ESCUCHA SIN TRANSMITIR. Los drivers ODrive emiten de fábrica su heartbeat cada
100 ms y sus estimaciones de encoder cada 10 ms, sin que nadie se lo pida (ver
sección 4.1.5 del manual), así que para encontrarlos basta con abrir la oreja.
Eso hace el barrido más fiable y además inofensivo:

  - no hace falta acertar el node_id, porque el motor lo dice en cada trama;
  - no se transmite nada, así que ninguna trama se queda sin ACK y el
    controlador CAN no puede irse a "bus-off" durante el propio diagnóstico;
  - no se le manda al motor ni una consigna, así que no se puede mover.

Correr en la Raspberry, una velocidad por vez (la velocidad del bus se fija al
abrir la placa, así que cada barrido necesita su propio proceso):

    source /opt/ros/jazzy/setup.bash && source ~/TESIS/install/setup.bash
    python3 -u src/ugv_bridge/scripts/escanear_can.py                 # 500 kbps
    python3 -u src/ugv_bridge/scripts/escanear_can.py --bitrate 1000000

Si no aparece nada a ninguna velocidad, el problema es físico: alimentación del
driver, CAN_H/CAN_L cruzados en el conector del motor, o CAN deshabilitado en el
propio driver (odrv0.config.enable_can_a por USB).
"""
import argparse
import asyncio
import sys

sys.path.insert(0, __file__.rsplit('/scripts/', 1)[0] + '/..')

from ugv_bridge import pi3hat_backend as backend  # noqa: E402
from ugv_bridge import protocolo_can as proto  # noqa: E402

# Buses físicos del pi3hat (los conectores JST van del 1 al 5).
BUSES = [1, 2, 3, 4, 5]

# Velocidades habituales. 500 kbps es la de fábrica del GIM6010-8.
BITRATES = [500000, 1000000, 250000, 125000]

NOMBRE_CMD = {
    proto.CMD_HEARTBEAT: 'heartbeat',
    proto.CMD_GET_ENCODER_ESTIMATES: 'encoder',
    proto.CMD_GET_IQ: 'iq',
}


async def escuchar(router, buses, segundos):
    """Escucha sin transmitir. Devuelve [(arb_id, datos), ...].

    El pi3hat no dice por qué bus entró cada trama, así que se apunta sólo el ID
    de arbitraje: para localizar el bus, escuchar de uno en uno con --buses.
    """
    mascara = 0
    for bus in buses:
        mascara |= (1 << bus)

    vistas = []
    fin = asyncio.get_event_loop().time() + segundos
    while asyncio.get_event_loop().time() < fin:
        try:
            resultados = await asyncio.wait_for(
                router.cycle([], force_can_check=mascara), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        for r in resultados or []:
            arb = getattr(r, 'arbitration_id', None)
            if arb is not None:
                vistas.append((arb, bytes(getattr(r, 'data', b''))))
    return vistas


def describir(arb, datos):
    """Una línea legible por trama."""
    nodo = proto.nodo_de_id(arb)
    cmd = proto.cmd_de_id(arb)
    nombre = NOMBRE_CMD.get(cmd, f'cmd 0x{cmd:03X}')
    info = proto.parsear(cmd, datos)
    if cmd == proto.CMD_HEARTBEAT and info:
        estado = {proto.ESTADO_IDLE: 'IDLE',
                  proto.ESTADO_CLOSED_LOOP: 'LAZO CERRADO'}.get(
                      info['estado_eje'], str(info['estado_eje']))
        extra = f'estado={estado}'
        if info['error_eje']:
            extra += f" ERROR=0x{info['error_eje']:X}"
    elif cmd == proto.CMD_GET_ENCODER_ESTIMATES and info:
        extra = (f"pos={info['posicion_rad']:+.3f} rad "
                 f"vel={info['velocidad_rad_s']:+.3f} rad/s")
    else:
        extra = datos.hex()
    return f'  node {nodo:<3} {nombre:<10} 0x{arb:03X}  {extra}'


async def main_async(args):
    buses = args.buses or BUSES
    print(f'Escuchando buses {buses} a {args.bitrate // 1000} kbps '
          f'durante {args.segundos:.0f} s, sin transmitir nada.\n')

    backend._importar()
    cfg = backend.config_can_clasico(buses)
    if cfg is None:
        print('Esta versión de moteus_pi3hat no expone CanConfiguration; '
              'no se puede fijar la velocidad del bus.')
        return 1
    for c in cfg.values():
        c.slow_bitrate = args.bitrate
        c.fast_bitrate = args.bitrate
    # servo_bus_map vacío: no se va a comandar a nadie, sólo escuchar.
    router = backend._moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={bus: [] for bus in buses}, can=cfg)

    vistas = await escuchar(router, buses, args.segundos)

    if not vistas:
        print(f'Nadie emitió nada a {args.bitrate // 1000} kbps.')
        print('\nProbar otra velocidad:')
        print('  ' + '  '.join(f'--bitrate {b}' for b in BITRATES if b != args.bitrate))
        print('\nSi a ninguna velocidad aparece nada, es físico o de configuración:')
        print('  - el driver no está alimentado (15-60 V por el XT30, no por USB)')
        print('  - CAN_H/CAN_L cruzados en el conector del motor')
        print('  - CAN deshabilitado en el driver: comprobar por USB con odrivetool')
        print('    odrv0.config.enable_can_a y odrv0.can.config.baud_rate')
        return 1

    nodos = {}
    for arb, datos in vistas:
        nodo = proto.nodo_de_id(arb)
        nodos.setdefault(nodo, []).append((arb, datos))

    print(f'{len(vistas)} tramas de {len(nodos)} nodo(s). Una muestra de cada tipo:\n')
    for nodo in sorted(n for n in nodos if n is not None):
        vistos = {}
        for arb, datos in nodos[nodo]:
            vistos.setdefault(proto.cmd_de_id(arb), (arb, datos))
        for _, (arb, datos) in sorted(vistos.items()):
            print(describir(arb, datos))

    print('\nHay tráfico: el cableado y la velocidad del bus son correctos.')
    print(f'node_id encontrados: {sorted(n for n in nodos if n is not None)}')
    return 0


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bitrate', type=int, default=500000,
                   help='velocidad del bus en bit/s (default: 500000, el de fábrica '
                        'del GIM6010-8)')
    p.add_argument('--buses', type=int, nargs='+', metavar='N',
                   help=f'buses a escuchar (default: {BUSES})')
    p.add_argument('--segundos', type=float, default=5.0,
                   help='cuánto escuchar (default: 5)')
    args = p.parse_args()
    sys.exit(asyncio.run(main_async(args)))


if __name__ == '__main__':
    main()
