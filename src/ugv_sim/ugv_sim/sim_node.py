#!/usr/bin/env python3
"""sim_node.py — Simulador del UGV: /scan + /odom + TF a 20 Hz."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from ugv_core.robot_env import RobotEnv, DT, MAX_RANGE, SENSOR_ANGLES


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class SimNode(Node):
    def __init__(self):
        super().__init__('ugv_sim')
        self.env = RobotEnv(seed=0)
        self.obs, _ = self.env.reset()
        self.accion = 1

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        self.timer = self.create_timer(DT, self.tick)
        self.get_logger().info('ugv_sim iniciado: /scan + /odom a 20 Hz')

    def on_cmd_vel(self, msg: Twist):
        w = msg.angular.z
        if w > 0.15:
            self.accion = 0      # izquierda
        elif w < -0.15:
            self.accion = 2      # derecha
        else:
            self.accion = 1      # recto

    def tick(self):
        self.obs, _r, term, trunc, _info = self.env.step(self.accion)
        if term or trunc:
            self.obs, _ = self.env.reset()
        stamp = self.get_clock().now().to_msg()
        self.publicar_odom_tf(stamp)
        self.publicar_scan(stamp)

    def env_yaw(self):
        # env: avance = (sin h, -cos h). Yaw ROS = ángulo de ese vector desde +X.
        h = math.radians(self.env.heading)
        return math.atan2(-math.cos(h), math.sin(h))

    def publicar_odom_tf(self, stamp):
        x = self.env.x / 100.0   # cm -> m
        y = self.env.y / 100.0
        q = yaw_to_quat(self.env_yaw())

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

        # TF odom -> base_link (el robot en el mundo)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation = q
        self.tf.sendTransform(t)

        # TF base_link -> laser (montaje fijo del sensor)
        tl = TransformStamped()
        tl.header.stamp = stamp
        tl.header.frame_id = 'base_link'
        tl.child_frame_id = 'laser'
        tl.transform.rotation.w = 1.0
        self.tf.sendTransform(tl)

    def publicar_scan(self, stamp):
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'laser'
        msg.angle_min = math.radians(SENSOR_ANGLES[0])
        msg.angle_max = math.radians(SENSOR_ANGLES[-1])
        msg.angle_increment = math.radians(45.0)
        msg.range_min = 0.0
        msg.range_max = MAX_RANGE / 100.0
        msg.ranges = [float(d) * MAX_RANGE / 100.0 for d in self.obs]
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
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
