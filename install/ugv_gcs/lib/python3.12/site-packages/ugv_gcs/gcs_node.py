#!/usr/bin/env python3
"""
gcs_node.py — GCS mínima como nodo ROS2.

Prueba el puente Qt <-> ROS: corre rclpy en un hilo y, vía señales Qt,
dibuja en vivo un mini-mapa 2D del robot a partir de /odom y /scan.
Es el esqueleto sobre el que luego se enchufa la GUI completa (simu.py).
"""
import sys
import math
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from PySide6.QtWidgets import QApplication, QWidget
from PySide6.QtCore import QObject, Signal, Qt, QTimer, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush


class RosSignals(QObject):
    """Puente: los callbacks de ROS (otro hilo) emiten estas señales Qt."""
    pose_ready = Signal(float, float, float)            # x, y, yaw (rad)
    scan_ready = Signal(list, float, float, float)      # ranges, amin, ainc, rmax


class GcsNode(Node):
    def __init__(self, signals):
        super().__init__('ugv_gcs')
        self.signals = signals
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.get_logger().info('ugv_gcs iniciado: suscrito a /odom y /scan')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        self.signals.pose_ready.emit(p.x, p.y, yaw)

    def on_scan(self, msg):
        self.signals.scan_ready.emit(
            list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_max)


class MiniMap(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('GCS RMD-S3 — Mini-mapa (ROS2)')
        self.resize(640, 640)
        self.x = self.y = self.yaw = 0.0
        self.ranges, self.amin, self.ainc, self.rmax = [], 0.0, 0.0, 2.5
        self.half_m = 2.7          # semilado visible (m)
        self.trail = []
        t = QTimer(self)
        t.timeout.connect(self.update)
        t.start(33)                # repinta ~30 fps en el hilo gráfico

    def set_pose(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw
        self.trail.append((x, y))
        if len(self.trail) > 3000:
            self.trail.pop(0)

    def set_scan(self, ranges, amin, ainc, rmax):
        self.ranges, self.amin, self.ainc, self.rmax = ranges, amin, ainc, rmax

    def w2p(self, x, y):
        """Mundo (m) -> pixel. +x derecha, +y arriba."""
        s = min(self.width(), self.height()) / (2 * self.half_m)
        return QPointF(self.width() / 2 + x * s, self.height() / 2 - y * s)

    def paintEvent(self, _):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        qp.fillRect(self.rect(), QColor(18, 20, 24))

        # Arena 5x5 m
        qp.setPen(QPen(QColor(70, 80, 90), 2))
        a, b = self.w2p(-2.5, 2.5), self.w2p(2.5, -2.5)
        qp.drawRect(int(a.x()), int(a.y()), int(b.x() - a.x()), int(b.y() - a.y()))

        # Traza recorrida
        qp.setPen(QPen(QColor(60, 120, 200), 1))
        for i in range(1, len(self.trail)):
            qp.drawLine(self.w2p(*self.trail[i - 1]), self.w2p(*self.trail[i]))

        # Rayos del telémetro (/scan)
        qp.setPen(QPen(QColor(220, 180, 60), 1))
        o = self.w2p(self.x, self.y)
        for i, r in enumerate(self.ranges):
            if r <= 0.0 or r >= self.rmax:
                continue
            ang = self.yaw + self.amin + i * self.ainc
            qp.drawLine(o, self.w2p(self.x + r * math.cos(ang),
                                    self.y + r * math.sin(ang)))

        # Robot + flecha de rumbo
        qp.setBrush(QBrush(QColor(80, 200, 120)))
        qp.setPen(Qt.NoPen)
        qp.drawEllipse(o, 7, 7)
        qp.setPen(QPen(QColor(80, 200, 120), 2))
        qp.drawLine(o, self.w2p(self.x + 0.35 * math.cos(self.yaw),
                                self.y + 0.35 * math.sin(self.yaw)))


def main():
    rclpy.init()
    signals = RosSignals()
    node = GcsNode(signals)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    win = MiniMap()
    signals.pose_ready.connect(win.set_pose)   # cross-thread: Qt lo encola seguro
    signals.scan_ready.connect(win.set_scan)
    win.show()
    code = app.exec()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
