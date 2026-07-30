#!/usr/bin/env python3
"""motor_emulator.py — Emula los 8 motores SteadyWin al otro lado del bus CAN.

Proceso independiente SIN ROS: escucha el bus (vcan0 por defecto), decodifica
las tramas de comando con `protocolo_can`, integra un modelo físico simple por
motor y responde las tramas de estado exactamente como lo haría el motor real.

Con esto el driver (`RMD_Hardware(modo='can')`) ejercita su código de
empaquetado/desempaquetado de bytes de verdad, cosa que el gemelo digital
(`modo='gemelo'`) no prueba.

Uso (no necesita ROS, solo python3-can y la interfaz vcan0 arriba):

    ros2 run ugv_bridge motor_emulator                # o bien:
    python3 -m ugv_bridge.motor_emulator --canal vcan0 --verbose

Depurar el tráfico desde otra terminal:  candump vcan0
"""
import argparse
import random
import sys
import time

from ugv_bridge import protocolo_can as proto
from ugv_bridge.driver_movimiento import IDS_ORUGAS, IDS_FLIPPERS, ID_A_NOMBRE

MODO_VELOCIDAD = 'velocidad'
MODO_POSICION = 'posicion'


class MotorEmulado:
    """Modelo físico mínimo de un motor: el mismo del gemelo digital."""

    def __init__(self, id_motor):
        self.id = id_motor
        self.pos = 0.0            # rad, multivuelta (acumulada)
        self.vel = 0.0            # rad/s
        self.temperatura = 35.0   # °C
        self.modo = MODO_VELOCIDAD
        self.objetivo = 0.0       # rad/s o rad según el modo

    def comandar_velocidad(self, vel_rad_s):
        self.modo = MODO_VELOCIDAD
        self.objetivo = vel_rad_s

    def comandar_posicion(self, pos_rad):
        self.modo = MODO_POSICION
        self.objetivo = pos_rad

    def parar(self):
        self.comandar_velocidad(0.0)

    def integrar(self, dt):
        if self.modo == MODO_VELOCIDAD:
            # Lazo de velocidad: sigue el objetivo con ruido de encoder.
            self.vel = self.objetivo + random.uniform(-0.02, 0.02)
            self.pos += self.vel * dt
        else:
            # Lazo de posición: primer orden, constante de tiempo ~0.12 s.
            error = self.objetivo - self.pos
            paso = error * min(dt * 8.0, 1.0)
            self.pos += paso
            self.vel = (paso / dt) if dt > 0 else 0.0

    @property
    def torque(self):
        if self.modo == MODO_VELOCIDAD:
            return abs(self.objetivo) * 0.3 + random.uniform(0.0, 0.2)
        return abs(self.objetivo - self.pos) * 2.0 + random.uniform(0.0, 0.3)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Emulador CAN de los 8 motores del UGV')
    parser.add_argument('--canal', default='vcan0', help='interfaz SocketCAN (default: vcan0)')
    parser.add_argument('--verbose', action='store_true', help='loguear cada trama')
    # parse_known_args: ros2 launch añade '--ros-args -r ...' aunque este proceso
    # no use ROS; se ignoran en vez de morir con exit 2.
    args, _ = parser.parse_known_args(argv)

    try:
        import can
    except ImportError:
        sys.exit('[EMULADOR] Falta python3-can. Instalar: sudo apt install python3-can')

    try:
        bus = can.interface.Bus(channel=args.canal, interface='socketcan')
    except OSError as e:
        sys.exit(
            f'[EMULADOR] No se pudo abrir {args.canal}: {e}\n'
            f'           ¿Existe la interfaz? Crear con: scripts/setup_vcan.sh'
        )

    motores = {mid: MotorEmulado(mid) for mid in IDS_ORUGAS + IDS_FLIPPERS}
    print(f'[EMULADOR] {len(motores)} motores en {args.canal} '
          f'(orugas {IDS_ORUGAS}, flippers {IDS_FLIPPERS}). Ctrl+C para salir.')

    t_prev = time.monotonic()
    t_stats = t_prev
    rx = tx = ignoradas = 0

    try:
        while True:
            msg = bus.recv(timeout=1.0)

            # La física avanza con el tiempo real, llegue o no tráfico.
            ahora = time.monotonic()
            dt = min(ahora - t_prev, 0.1)
            t_prev = ahora
            for m in motores.values():
                m.integrar(dt)

            if msg is not None:
                rx += 1
                id_motor = proto.motor_de_id_comando(msg.arbitration_id)
                comando = proto.parsear_comando(msg.data) if id_motor in motores else None
                if comando is None:
                    ignoradas += 1
                else:
                    respuesta = atender(motores[id_motor], comando, args.verbose)
                    if respuesta is not None:
                        bus.send(can.Message(
                            arbitration_id=proto.id_respuesta(id_motor),
                            data=respuesta,
                            is_extended_id=False,
                        ))
                        tx += 1

            if ahora - t_stats >= 5.0:
                t_stats = ahora
                pos = ' '.join(f'{ID_A_NOMBRE[m.id]}={m.pos:+.2f}' for m in motores.values())
                print(f'[EMULADOR] rx={rx} tx={tx} ignoradas={ignoradas} | pos(rad): {pos}')
    except KeyboardInterrupt:
        print('\n[EMULADOR] Cerrando bus.')
    finally:
        bus.shutdown()


def atender(motor, comando, verbose=False):
    """Aplica un comando al motor emulado y devuelve los bytes de respuesta."""
    cmd = comando['cmd']

    if cmd == proto.CMD_VELOCIDAD:
        motor.comandar_velocidad(comando['velocidad_rad_s'])
    elif cmd == proto.CMD_POSICION:
        motor.comandar_posicion(comando['posicion_rad'])
    elif cmd == proto.CMD_PARO:
        motor.parar()
    elif cmd == proto.CMD_APAGADO:
        motor.parar()

    if verbose:
        print(f'[EMULADOR] motor {motor.id} cmd=0x{cmd:02X} '
              f'pos={motor.pos:+.3f} vel={motor.vel:+.3f}')

    if cmd == proto.CMD_LEER_MULTIVUELTA:
        return proto.trama_respuesta_multivuelta(motor.pos)

    # 0xA2/0xA4/0x9C/0x81/0x80 responden todos la trama de estado.
    return proto.trama_respuesta_estado(
        cmd, motor.temperatura, motor.torque, motor.vel, motor.pos)


if __name__ == '__main__':
    main()
