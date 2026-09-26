#!/usr/bin/env python3
"""motor_emulator.py — Emula los 8 motores SteadyWin al otro lado del bus CAN.

Proceso independiente SIN ROS: escucha el bus (vcan0 por defecto), decodifica las
tramas con `protocolo_can`, integra un modelo físico simple por motor y EMITE los
mensajes periódicos igual que el driver ODrive real.

Con esto el driver (`RMD_Hardware(modo='can')`) ejercita su código de
empaquetado/desempaquetado de bytes de verdad, cosa que el gemelo digital
(`modo='gemelo'`) no prueba.

---------------------------------------------------------------------------
LO QUE CAMBIÓ AL PASAR A ODrive
---------------------------------------------------------------------------
El motor ya NO contesta a cada trama: emite por su cuenta. Este emulador hace lo
mismo (ver sección 4.1.5 del manual del GIM6010-8):

    Get_Encoder_Estimates (0x009)   cada  10 ms
    Heartbeat (0x001)               cada 100 ms

Y respeta el ESTADO DEL EJE: en IDLE acepta las consignas y no se mueve, sin dar
ningún error. Es exactamente el comportamiento del hardware, y el que hace que
olvidarse de armar los ejes se note aquí y no en el robot.

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

# Periodos de los mensajes periódicos, en segundos (valores de fábrica).
PERIODO_ENCODER_S = 0.010
PERIODO_HEARTBEAT_S = 0.100


class MotorEmulado:
    """Modelo físico mínimo de un motor ODrive: el mismo del gemelo digital."""

    def __init__(self, id_motor):
        self.id = id_motor
        self.pos = 0.0            # rad en el eje de salida, multivuelta
        self.vel = 0.0            # rad/s en el eje de salida
        self.temperatura = 35.0   # °C (no se publica por CAN; sólo para el log)
        self.estado_eje = proto.ESTADO_IDLE
        self.control_mode = proto.CONTROL_VELOCIDAD
        self.input_mode = proto.ENTRADA_PASSTHROUGH
        self.objetivo = 0.0       # rad/s o rad según control_mode
        self.error_eje = 0

    # ---------------- comandos que llegan del host ---------------- #
    def comandar_velocidad(self, vel_rad_s):
        self.objetivo = vel_rad_s

    def comandar_posicion(self, pos_rad):
        self.objetivo = pos_rad

    def poner_estado(self, estado):
        self.estado_eje = estado
        if estado == proto.ESTADO_IDLE:
            # Sin par: la consigna deja de aplicarse y el eje se queda quieto.
            self.vel = 0.0

    def poner_modo(self, control_mode, input_mode):
        # Sólo un CAMBIO de modo descarta la consigna anterior (era de otras
        # unidades). Reescribir el mismo modo no la toca: el driver reenvía la
        # secuencia de armado cuando un eje se cae a IDLE, y si eso borrara la
        # consigna, rearmar un flipper lo dejaría clavado donde estaba.
        cambio = control_mode != self.control_mode
        self.control_mode = control_mode
        self.input_mode = input_mode
        if cambio:
            self.objetivo = self.pos if control_mode == proto.CONTROL_POSICION else 0.0

    def parar(self):
        self.objetivo = 0.0
        self.poner_estado(proto.ESTADO_IDLE)

    # ---------------- física ---------------- #
    def integrar(self, dt):
        if self.estado_eje != proto.ESTADO_CLOSED_LOOP:
            # En IDLE el eje no tiene par: no sigue ninguna consigna.
            self.vel = 0.0
            return

        if self.control_mode == proto.CONTROL_VELOCIDAD:
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
        if self.control_mode == proto.CONTROL_VELOCIDAD:
            return abs(self.objetivo) * 0.3 + random.uniform(0.0, 0.2)
        return abs(self.objetivo - self.pos) * 2.0 + random.uniform(0.0, 0.3)

    # ---------------- mensajes que emite ---------------- #
    def trama_encoder(self):
        return proto.trama_encoder(self.pos, self.vel)

    def trama_heartbeat(self):
        return proto.trama_heartbeat(estado_eje=self.estado_eje,
                                     error_eje=self.error_eje)


def atender(motor, comando, verbose=False):
    """Aplica un mensaje host->motor al motor emulado.

    A diferencia del protocolo anterior, NO devuelve una respuesta: en ODrive el
    motor contesta emitiendo sus mensajes periódicos, no trama a trama.
    """
    if comando is None:
        return
    cmd = comando['cmd']

    if cmd == proto.CMD_SET_INPUT_VEL:
        motor.comandar_velocidad(comando['velocidad_rad_s'])
    elif cmd == proto.CMD_SET_INPUT_POS:
        motor.comandar_posicion(comando['posicion_rad'])
    elif cmd == proto.CMD_SET_AXIS_STATE:
        motor.poner_estado(comando['estado'])
    elif cmd == proto.CMD_SET_CONTROLLER_MODE:
        motor.poner_modo(comando['control_mode'], comando['input_mode'])
    elif cmd == proto.CMD_CLEAR_ERRORS:
        motor.error_eje = 0
    elif cmd == proto.CMD_ESTOP:
        motor.parar()

    if verbose:
        print(f'[EMULADOR] motor {motor.id} cmd=0x{cmd:03X} '
              f'estado={motor.estado_eje} pos={motor.pos:+.3f} vel={motor.vel:+.3f}')


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
    print(f'[EMULADOR] {len(motores)} motores ODrive en {args.canal} '
          f'(orugas {IDS_ORUGAS}, flippers {IDS_FLIPPERS}). Ctrl+C para salir.')
    print(f'[EMULADOR] emitiendo encoder cada {PERIODO_ENCODER_S * 1000:.0f} ms '
          f'y heartbeat cada {PERIODO_HEARTBEAT_S * 1000:.0f} ms')

    t_prev = time.monotonic()
    t_encoder = t_heartbeat = t_stats = t_prev
    rx = tx = ignoradas = 0

    def emitir(id_motor, cmd_id, datos):
        bus.send(can.Message(
            arbitration_id=proto.id_arbitraje(id_motor, cmd_id),
            data=datos,
            is_extended_id=False,
        ))

    try:
        while True:
            # Timeout corto: hay que volver aquí para emitir los periódicos aunque
            # el host no mande nada.
            msg = bus.recv(timeout=PERIODO_ENCODER_S / 2.0)

            ahora = time.monotonic()
            dt = min(ahora - t_prev, 0.1)
            t_prev = ahora
            for m in motores.values():
                m.integrar(dt)

            if msg is not None:
                rx += 1
                nodo = proto.nodo_de_id(msg.arbitration_id)
                cmd_id = proto.cmd_de_id(msg.arbitration_id)
                comando = (proto.parsear_comando(cmd_id, msg.data)
                           if nodo in motores else None)
                if comando is None:
                    ignoradas += 1
                else:
                    atender(motores[nodo], comando, args.verbose)

            if ahora - t_encoder >= PERIODO_ENCODER_S:
                t_encoder = ahora
                for m in motores.values():
                    emitir(m.id, proto.CMD_GET_ENCODER_ESTIMATES, m.trama_encoder())
                    tx += 1

            if ahora - t_heartbeat >= PERIODO_HEARTBEAT_S:
                t_heartbeat = ahora
                for m in motores.values():
                    emitir(m.id, proto.CMD_HEARTBEAT, m.trama_heartbeat())
                    tx += 1

            if ahora - t_stats >= 5.0:
                t_stats = ahora
                armados = sum(1 for m in motores.values()
                              if m.estado_eje == proto.ESTADO_CLOSED_LOOP)
                pos = ' '.join(f'{ID_A_NOMBRE[m.id]}={m.pos:+.2f}' for m in motores.values())
                print(f'[EMULADOR] rx={rx} tx={tx} ignoradas={ignoradas} '
                      f'armados={armados}/{len(motores)} | pos(rad): {pos}')
    except KeyboardInterrupt:
        print('\n[EMULADOR] Cerrando bus.')
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
