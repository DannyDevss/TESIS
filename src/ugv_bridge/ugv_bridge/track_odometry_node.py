#!/usr/bin/env python3
"""track_odometry_node.py — Odometría de las orugas (JointState -> nav_msgs/Odometry).

Deriva la pose planar del chasis integrando las velocidades de las 4 orugas con un
modelo de tracción diferencial (skid-steer):

    v_izq  = r * mean(w_track_fl, w_track_rl)     # lado izquierdo
    v_der  = r * mean(w_track_fr, w_track_rr)     # lado derecho
    v      = (v_izq + v_der) / 2                   # velocidad lineal (m/s)
    w      = (v_der - v_izq) / track_width         # velocidad angular (rad/s)

Publica `/odom` (Odometry) que consume el EKF de robot_localization como `odom0`.
Esta odometría PATINA (las orugas resbalan): el EKF la fusiona con la IMU para
corregir. Por eso este nodo publica sólo el mensaje, **no** el TF odom->base_link
(ese lo emite el EKF cuando está activo; ver `config/ekf.yaml`).

Parámetros:
    radio_oruga   (double, 0.05) radio efectivo de la oruga/rueda motriz (m).
    ancho_orugas  (double, 0.30) separación entre orugas izq/der (m).
    publish_tf    (bool, false)  si true, emite TF odom->base_link (usar SÓLO sin EKF).

Los dos primeros salen de config/geometria_robot.yaml (única fuente de verdad de
la geometría, compartida con el URDF y el guardián de colisiones). Antes se
llamaban wheel_radius/track_width; se renombraron para que coincidan con las
claves del YAML y no haya dos nombres para el mismo número.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

# Nombres de las orugas por lado (deben coincidir con driver_movimiento.NOMBRES_ORUGAS).
ORUGAS_IZQ = ['track_fl', 'track_rl']
ORUGAS_DER = ['track_fr', 'track_rr']


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class TrackOdometryNode(Node):
    def __init__(self):
        super().__init__('track_odometry_node')

        self.declare_parameter('radio_oruga', 0.05)
        self.declare_parameter('ancho_orugas', 0.30)
        self.declare_parameter('publish_tf', False)
        self.r = self.get_parameter('radio_oruga').value
        self.track_width = self.get_parameter('ancho_orugas').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Pose integrada (frame odom).
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.t_prev = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        self.get_logger().info(
            f'track_odometry_node iniciado (r={self.r} m, ancho={self.track_width} m, '
            f'publish_tf={self.publish_tf})'
        )

    def on_joint_states(self, msg: JointState):
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        if not all(n in vel for n in ORUGAS_IZQ + ORUGAS_DER):
            return  # aún no llegan las orugas en este mensaje

        v_izq = self.r * sum(vel[n] for n in ORUGAS_IZQ) / len(ORUGAS_IZQ)
        v_der = self.r * sum(vel[n] for n in ORUGAS_DER) / len(ORUGAS_DER)
        v = (v_izq + v_der) / 2.0
        w = (v_der - v_izq) / self.track_width

        # dt a partir del stamp del propio JointState.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_prev is None:
            self.t_prev = t
            return
        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0 or dt > 0.5:
            return  # descarta saltos/pausas

        # Integración de la pose (modelo de velocidad constante en el intervalo).
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        self.publicar_odom(v, w, msg.header.stamp)

    def publicar_odom(self, v, w, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quat(self.yaw)
            self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TrackOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
