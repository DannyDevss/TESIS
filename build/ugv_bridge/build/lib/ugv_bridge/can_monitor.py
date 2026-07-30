#!/usr/bin/env python3
"""can_monitor.py — Ventana para VER (por motor) y EDITAR (por flipper) el bus CAN.

Nodo ROS 2 + ventana Qt + espía SocketCAN, pensado para verse al lado de RViz.
Resuelve dos problemas de un candump corriendo:

  1. TRAMAS SEPARADAS Y LEGIBLES. En vez de una lista que baja sin parar, muestra
     un PANEL POR MOTOR: 8 filas fijas (una por motor) que se actualizan en su
     sitio con su último comando, velocidad, torque, temperatura y posición. Así
     se distingue de un vistazo qué está haciendo cada motor. El log crudo
     (candump gráfico) queda como pestaña secundaria opcional.

  2. EDITOR DE FLIPPERS FÁCIL. Un panel con un control por flipper (slider +
     grados) que mueve el flipper DE VERDAD: publica /cmd_flippers -> flipper_node
     -> tramas CAN (vcan0) -> motor_emulator -> RViz. Junto a cada flipper se ve
     la TRAMA exacta (bytes 0xA4…) que genera, así editar el flipper es editar su
     trama y verla al instante.

Flujo:

    editor de flippers ─▶ /cmd_flippers ─▶ flipper_node ─▶ vcan0 (0x14x)
                                                            │
                                        motor_emulator ◀────┘
                                        └─▶ vcan0 (0x24x) ─▶ flipper_node ─▶ RViz
                                                    │
                                        can_monitor ┘  (panel por motor + log)

Uso (junto con RViz):   ros2 launch ugv_bridge can_studio.launch.py
Uso suelto:             ros2 run ugv_bridge can_monitor --canal vcan0

Requisitos: python3-can, PySide6, rclpy y vcan0 arriba (scripts/setup_vcan.sh).
"""
import argparse
import collections
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from ugv_bridge import protocolo_can as proto
from ugv_bridge.driver_movimiento import (
    ID_A_NOMBRE, IDS_ORUGAS, IDS_FLIPPERS, NOMBRES_FLIPPERS,
)

# Nombre legible de cada código de comando (byte 0 de la trama).
NOMBRE_CMD = {
    proto.CMD_VELOCIDAD: 'VELOCIDAD',
    proto.CMD_POSICION: 'POSICION',
    proto.CMD_LEER_ESTADO: 'LEER_ESTADO',
    proto.CMD_LEER_MULTIVUELTA: 'LEER_MULTIV',
    proto.CMD_PARO: 'PARO',
    proto.CMD_APAGADO: 'APAGADO',
}


def _rad2deg(r):
    return r * 180.0 / math.pi


def decodificar(arb_id, data):
    """Trama CAN cruda -> (sentido, id_motor, tipo, valor_str) para la tabla.

    sentido: '-> motor' (comando 0x14x) o '<- motor' (respuesta 0x24x).
    Devuelve None si el ID no corresponde a ningún motor conocido.
    """
    id_cmd = proto.motor_de_id_comando(arb_id)              # comando 0x140+id
    id_resp = arb_id - proto.ID_RESP_BASE                    # respuesta 0x240+id
    todos = IDS_ORUGAS + IDS_FLIPPERS

    if id_cmd in todos:
        info = proto.parsear_comando(data)
        tipo = NOMBRE_CMD.get(info['cmd'], f"0x{info['cmd']:02X}") if info else '??'
        if info and info['cmd'] == proto.CMD_VELOCIDAD:
            valor = f"{info['velocidad_rad_s']:+.2f} rad/s"
        elif info and info['cmd'] == proto.CMD_POSICION:
            valor = f"{_rad2deg(info['posicion_rad']):+.1f}°"
        else:
            valor = '—'
        return ('-> motor', id_cmd, tipo, valor)

    if id_resp in todos:
        info = proto.parsear_respuesta(data)
        if info and info['cmd'] == proto.CMD_LEER_MULTIVUELTA:
            return ('<- motor', id_resp, 'RESP_MULTIV',
                    f"{_rad2deg(info['posicion_rad']):+.1f}°")
        if info and 'velocidad_rad_s' in info:
            return ('<- motor', id_resp, 'RESP_ESTADO',
                    f"v={info['velocidad_rad_s']:+.2f}  τ={info['torque_nm']:.2f}  "
                    f"T={info['temperatura_c']:.0f}°C")
        return ('<- motor', id_resp, 'RESP', '—')

    return None


class MonitorNode(Node):
    """Publica los comandos que compone el editor (el robot se mueve de verdad)."""

    def __init__(self):
        super().__init__('can_monitor')
        self.pub_flippers = self.create_publisher(Float64MultiArray, '/cmd_flippers', 10)
        self.pub_tracks = self.create_publisher(Float64MultiArray, '/cmd_tracks', 10)

    def enviar_flippers(self, pos_rad_fl_fr_rl_rr):
        self.pub_flippers.publish(Float64MultiArray(data=list(pos_rad_fl_fr_rl_rr)))

    def enviar_tracks(self, vel_rad_s_fl_fr_rl_rr):
        self.pub_tracks.publish(Float64MultiArray(data=list(vel_rad_s_fl_fr_rl_rr)))


def main(argv=None):
    parser = argparse.ArgumentParser(description='Monitor por motor + editor de flippers del bus CAN')
    parser.add_argument('--canal', default='vcan0', help='interfaz SocketCAN (default: vcan0)')
    parser.add_argument('--max-log', type=int, default=400, help='filas del log crudo (default: 400)')
    args, _ = parser.parse_known_args(argv)

    try:
        import can
    except ImportError:
        sys.exit('[MONITOR] Falta python3-can. Instalar: sudo apt install python3-can')
    try:
        from PySide6.QtWidgets import (
            QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
            QTableWidget, QTableWidgetItem, QPushButton, QLabel, QCheckBox,
            QHeaderView, QGroupBox, QTabWidget, QSlider, QDoubleSpinBox,
        )
        from PySide6.QtCore import Qt, QTimer
        from PySide6.QtGui import QColor, QFont
    except ImportError:
        sys.exit('[MONITOR] Falta PySide6. Instalar: pip install PySide6')

    # Espía del bus: un socket que recibe TODAS las tramas de vcan0.
    try:
        bus_rx = can.interface.Bus(channel=args.canal, interface='socketcan')
    except OSError as e:
        sys.exit(f'[MONITOR] No se pudo abrir {args.canal}: {e}\n'
                 f'          ¿Existe la interfaz? Crear con: scripts/setup_vcan.sh')

    cola = collections.deque(maxlen=8000)
    parar = threading.Event()

    def lector():
        while not parar.is_set():
            msg = bus_rx.recv(timeout=0.2)
            if msg is not None:
                cola.append((time.monotonic(), msg.arbitration_id, bytes(msg.data)))

    threading.Thread(target=lector, daemon=True).start()

    # Nodo ROS (para que el editor publique comandos reales).
    rclpy.init()
    node = MonitorNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    TODOS = IDS_ORUGAS + IDS_FLIPPERS
    COLS_MOTOR = ['motor', 'ID', 'últ. cmd', 'valor cmd', 'vel (rad/s)',
                  'torque (Nm)', 'temp (°C)', 'pos (°)', 'tramas']
    COLS_LOG = ['t (s)', 'sentido', 'ID', 'motor', 'tipo', 'valor', 'bytes']

    AZUL = QColor(90, 170, 255)     # comando -> motor
    VERDE = QColor(120, 210, 140)   # respuesta <- motor

    class Ventana(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle(f'Monitor CAN por motor + editor de flippers — {args.canal}')
            self.resize(920, 760)
            self.t0 = time.monotonic()
            self.total = 0
            self._cuenta_ventana = 0
            self._t_rate = self.t0
            # Estado por motor para el panel (se actualiza en su sitio).
            self.estado = {mid: {'cmd': '—', 'valor': '—', 'vel': '—', 'torque': '—',
                                 'temp': '—', 'pos': '—', 'n': 0} for mid in TODOS}
            self.flip_rad = [0.0, 0.0, 0.0, 0.0]  # fl, fr, rl, rr

            root = QVBoxLayout(self)

            # Barra: stats + pausa.
            barra = QHBoxLayout()
            self.lbl = QLabel('esperando tráfico…')
            self.chk_pausa = QCheckBox('Pausar')
            barra.addWidget(self.lbl, 1)
            barra.addWidget(self.chk_pausa)
            root.addLayout(barra)

            # Pestañas: panel por motor (principal) y log crudo (secundario).
            tabs = QTabWidget()
            tabs.addTab(self._tab_motores(), 'Por motor (en vivo)')
            tabs.addTab(self._tab_log(), 'Log crudo')
            root.addWidget(tabs, 1)

            # Editor de flippers (siempre visible).
            root.addWidget(self._editor_flippers())
            # Editor de orugas (compacto).
            root.addWidget(self._editor_orugas())

            self.timer = QTimer(self)
            self.timer.timeout.connect(self._drenar)
            self.timer.start(50)  # 20 Hz

        # ---------------------------- pestañas ---------------------------- #
        def _tab_motores(self):
            self.tbl = QTableWidget(len(TODOS), len(COLS_MOTOR))
            self.tbl.setHorizontalHeaderLabels(COLS_MOTOR)
            self.tbl.verticalHeader().setVisible(False)
            self.tbl.setEditTriggers(QTableWidget.NoEditTriggers)
            self.tbl.setFont(QFont('monospace', 9))
            self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            for r, mid in enumerate(TODOS):
                base = [ID_A_NOMBRE[mid], f'0x{proto.id_comando(mid):03X}',
                        '—', '—', '—', '—', '—', '—', '0']
                for c, txt in enumerate(base):
                    it = QTableWidgetItem(txt)
                    if c == 0 and mid in IDS_FLIPPERS:
                        it.setForeground(QColor(230, 200, 120))  # flippers resaltados
                    self.tbl.setItem(r, c, it)
            self.tbl.resizeColumnsToContents()
            return self.tbl

        def _tab_log(self):
            w = QWidget()
            lay = QVBoxLayout(w)
            fila = QHBoxLayout()
            self.chk_auto = QCheckBox('Auto-scroll'); self.chk_auto.setChecked(True)
            btn_limpiar = QPushButton('Limpiar'); btn_limpiar.clicked.connect(
                lambda: self.log.setRowCount(0))
            fila.addWidget(self.chk_auto); fila.addStretch(1); fila.addWidget(btn_limpiar)
            lay.addLayout(fila)
            self.log = QTableWidget(0, len(COLS_LOG))
            self.log.setHorizontalHeaderLabels(COLS_LOG)
            self.log.verticalHeader().setVisible(False)
            self.log.setEditTriggers(QTableWidget.NoEditTriggers)
            self.log.setFont(QFont('monospace', 9))
            self.log.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
            self.log.horizontalHeader().setSectionResizeMode(6, QHeaderView.Stretch)
            lay.addWidget(self.log)
            return w

        # ---------------------------- editores ---------------------------- #
        def _editor_flippers(self):
            caja = QGroupBox('Editar flippers (mueve el flipper y muestra su trama)')
            g = QGridLayout(caja)
            g.addWidget(QLabel('flipper'), 0, 0)
            g.addWidget(QLabel('ángulo'), 0, 1)
            g.addWidget(QLabel('grados'), 0, 2)
            g.addWidget(QLabel('trama que se envía (0xA4)'), 0, 3)
            self.sliders, self.spins, self.tramas_lbl = [], [], []
            for i, nombre in enumerate(NOMBRES_FLIPPERS):
                g.addWidget(QLabel(nombre), i + 1, 0)
                s = QSlider(Qt.Horizontal); s.setRange(-180, 180); s.setValue(0)
                sp = QDoubleSpinBox(); sp.setRange(-180, 180); sp.setDecimals(0)
                sp.setSuffix('°'); sp.setValue(0)
                lbl = QLabel('—'); lbl.setFont(QFont('monospace', 9))
                s.valueChanged.connect(lambda v, i=i: self._flip_desde_slider(i, v))
                sp.valueChanged.connect(lambda v, i=i: self._flip_desde_spin(i, v))
                g.addWidget(s, i + 1, 1)
                g.addWidget(sp, i + 1, 2)
                g.addWidget(lbl, i + 1, 3)
                self.sliders.append(s); self.spins.append(sp); self.tramas_lbl.append(lbl)
                self._actualizar_trama_lbl(i)
            botones = QHBoxLayout()
            b0 = QPushButton('Todos a 0° (plano)'); b0.clicked.connect(self._flippers_cero)
            b45 = QPushButton('Todos a 45°'); b45.clicked.connect(lambda: self._flippers_todos(45))
            botones.addWidget(b0); botones.addWidget(b45); botones.addStretch(1)
            g.addLayout(botones, len(NOMBRES_FLIPPERS) + 1, 0, 1, 4)
            return caja

        def _editor_orugas(self):
            caja = QGroupBox('Orugas (velocidad)')
            fila = QHBoxLayout(caja)
            self.sp_izq = QDoubleSpinBox(); self.sp_izq.setRange(-20, 20)
            self.sp_izq.setSuffix(' rad/s'); self.sp_izq.setSingleStep(0.5)
            self.sp_der = QDoubleSpinBox(); self.sp_der.setRange(-20, 20)
            self.sp_der.setSuffix(' rad/s'); self.sp_der.setSingleStep(0.5)
            b_ir = QPushButton('Aplicar'); b_ir.clicked.connect(self._enviar_orugas)
            b_stop = QPushButton('Parar'); b_stop.clicked.connect(self._parar_orugas)
            fila.addWidget(QLabel('izq (fl,rl)')); fila.addWidget(self.sp_izq)
            fila.addWidget(QLabel('der (fr,rr)')); fila.addWidget(self.sp_der)
            fila.addWidget(b_ir); fila.addWidget(b_stop); fila.addStretch(1)
            return caja

        # ---------------------------- callbacks --------------------------- #
        def _actualizar_trama_lbl(self, i):
            hexstr = proto.trama_cmd_posicion(self.flip_rad[i]).hex()
            par = ' '.join(hexstr[j:j + 2] for j in range(0, len(hexstr), 2))
            self.tramas_lbl[i].setText(f'0x{proto.id_comando(IDS_FLIPPERS[i]):03X}  {par}')

        def _set_flip(self, i, grados):
            self.flip_rad[i] = math.radians(grados)
            self._actualizar_trama_lbl(i)
            node.enviar_flippers(self.flip_rad)

        def _flip_desde_slider(self, i, v):
            self.spins[i].blockSignals(True); self.spins[i].setValue(v); self.spins[i].blockSignals(False)
            self._set_flip(i, v)

        def _flip_desde_spin(self, i, v):
            self.sliders[i].blockSignals(True); self.sliders[i].setValue(int(v)); self.sliders[i].blockSignals(False)
            self._set_flip(i, v)

        def _flippers_cero(self):
            self._flippers_todos(0)

        def _flippers_todos(self, grados):
            for i in range(4):
                self.sliders[i].blockSignals(True); self.sliders[i].setValue(grados); self.sliders[i].blockSignals(False)
                self.spins[i].blockSignals(True); self.spins[i].setValue(grados); self.spins[i].blockSignals(False)
                self.flip_rad[i] = math.radians(grados)
                self._actualizar_trama_lbl(i)
            node.enviar_flippers(self.flip_rad)

        def _enviar_orugas(self):
            izq, der = self.sp_izq.value(), self.sp_der.value()
            node.enviar_tracks([izq, der, izq, der])  # orden fl, fr, rl, rr

        def _parar_orugas(self):
            self.sp_izq.setValue(0); self.sp_der.setValue(0)
            node.enviar_tracks([0.0, 0.0, 0.0, 0.0])

        # ---------------------------- refresco ---------------------------- #
        def _drenar(self):
            if self.chk_pausa.isChecked():
                return
            nuevas_log = 0
            while cola:
                t, arb_id, data = cola.popleft()
                self.total += 1
                self._cuenta_ventana += 1
                fila = decodificar(arb_id, data)
                if fila is None:
                    continue
                sentido, mid, tipo, valor = fila
                # Actualiza el panel por motor (en su sitio, no scroll).
                e = self.estado[mid]
                e['n'] += 1
                if sentido == '-> motor':
                    e['cmd'], e['valor'] = tipo, valor
                elif tipo == 'RESP_MULTIV':
                    e['pos'] = valor
                elif tipo == 'RESP_ESTADO':
                    # "v=.. τ=.. T=.." -> desglosar a columnas
                    try:
                        partes = valor.replace('v=', '').replace('τ=', '').replace('T=', '')
                        vel, torque, temp = partes.split()
                        e['vel'], e['torque'], e['temp'] = vel, torque, temp.replace('°C', '')
                    except ValueError:
                        pass
                # Log crudo (pestaña secundaria).
                self._append_log(t, arb_id, sentido, mid, tipo, valor, data)
                nuevas_log += 1

            self._pintar_motores()
            if nuevas_log and self.chk_auto.isChecked():
                self.log.scrollToBottom()

            ahora = time.monotonic()
            if ahora - self._t_rate >= 1.0:
                rate = self._cuenta_ventana / (ahora - self._t_rate)
                self._cuenta_ventana = 0; self._t_rate = ahora
                estado = 'PAUSA' if self.chk_pausa.isChecked() else 'en vivo'
                self.lbl.setText(f'{self.total} tramas · {rate:.0f} tramas/s · '
                                 f'{args.canal} · {estado}')

        def _pintar_motores(self):
            for r, mid in enumerate(TODOS):
                e = self.estado[mid]
                for c, val in ((2, e['cmd']), (3, e['valor']), (4, e['vel']),
                               (5, e['torque']), (6, e['temp']), (7, e['pos']),
                               (8, str(e['n']))):
                    self.tbl.item(r, c).setText(val)

        def _append_log(self, t, arb_id, sentido, mid, tipo, valor, data):
            r = self.log.rowCount()
            self.log.insertRow(r)
            celdas = [f'{t - self.t0:7.2f}', sentido, f'0x{arb_id:03X}',
                      ID_A_NOMBRE.get(mid, ''), tipo, valor, data.hex()]
            color = AZUL if sentido == '-> motor' else VERDE
            for c, txt in enumerate(celdas):
                it = QTableWidgetItem(txt); it.setForeground(color)
                self.log.setItem(r, c, it)
            while self.log.rowCount() > args.max_log:
                self.log.removeRow(0)

        def closeEvent(self, ev):
            parar.set()
            try:
                bus_rx.shutdown()
            except Exception:
                pass
            ev.accept()

    app = QApplication(sys.argv)
    win = Ventana()
    win.show()
    codigo = app.exec()

    parar.set()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
