import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import asyncio
import math
import moteus_pi3hat

def cuaternion_a_euler(w, x, y, z):
    
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class Pi3HatImuNode(Node):
    def __init__(self):
        super().__init__('pi3hat_imu_node')
        # Foxglove lee automáticamente el tipo Imu en este tópico
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

    def publish_imu_data(self, w, x, y, z):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link' # Crucial para el 3D en Foxglove

        msg.orientation.w = float(w)
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)

        # Desactivamos covarianzas (confianza ciega en el sensor)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.publisher_.publish(msg)

async def main_loop(args=None):
    rclpy.init(args=args)
    node = Pi3HatImuNode()
    node.get_logger().info("Nodo IMU iniciado. Publicando en /imu/data")

    try:
        # Esto fallará si se ejecuta en Distrobox (sin hardware), pero funcionará en la Pi
        pi3hat = moteus_pi3hat.Pi3HatRouter(servo_bus_map={})
        await pi3hat.cycle([])
    except Exception as e:
        node.get_logger().error(f"Hardware Pi3Hat no detectado. Si estás en Distrobox, esto es normal. Detalle: {e}")
        return # Detenemos el nodo si no hay hardware

    try:
        while rclpy.ok():
            await pi3hat.cycle([])
            imu = await pi3hat.attitude()

            try:
                w, x, y, z = imu.w, imu.x, imu.y, imu.z
            except AttributeError:
                w, x, y, z = imu.attitude.w, imu.attitude.x, imu.attitude.y, imu.attitude.z

            # 1. Enviar a Foxglove (Cuaterniones)
            node.publish_imu_data(w, x, y, z)

            # 2. Imprimir en Terminal (Euler + Cuaterniones)
            roll, pitch, yaw = cuaternion_a_euler(w, x, y, z)
            print(f"\r[IMU] Roll:{roll: 6.1f} | Pitch:{pitch: 6.1f} | Yaw:{yaw: 6.1f}  (W:{w: .2f} X:{x: .2f} Y:{y: .2f} Z:{z: .2f})    ", end="", flush=True)

            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(0.02) # 50 Hz
            
    except KeyboardInterrupt:
        print("\n\nTest finalizado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    asyncio.run(main_loop(args))

if __name__ == '__main__':
    main()
