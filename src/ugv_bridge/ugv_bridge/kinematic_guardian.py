#!/usr/bin/env python3
"""kinematic_guardian.py — Filtro de seguridad que impide que los flippers choquen.

Escucha lo que el operador/IA PIDE en /joint_states_raw y publica en /joint_states
sólo lo que es geométricamente seguro; si un comando haría chocar el flipper
delantero con el trasero del mismo lado, se congela ese lado en la última pose
segura conocida.

La geometría (posición de los motores, largo del flipper, radio de las ruedas)
NO está hardcodeada: se declara como parámetros ROS 2 y se carga desde
config/geometria_robot.yaml, el MISMO archivo del que sale el URDF. Así el
guardián nunca protege contra una geometría distinta a la que se dibuja.

    ros2 run ugv_bridge kinematic_guardian --ros-args \
        --params-file src/ugv_bridge/config/geometria_robot.yaml
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class KinematicGuardian(Node):
    def __init__(self):
        super().__init__('kinematic_guardian')

        # --- Geometría (config/geometria_robot.yaml) ---
        # Los defaults replican la geometría actual, para que el nodo siga
        # funcionando si se lanza suelto sin --params-file.
        self.declare_parameter('distancia_motor_x', 0.23)
        self.declare_parameter('distancia_motor_z', -0.03)
        self.declare_parameter('largo_flipper', 0.26)
        self.declare_parameter('radio_rueda_flipper', 0.09)

        self.motor_x = self.get_parameter('distancia_motor_x').value
        self.motor_z = self.get_parameter('distancia_motor_z').value
        self.largo_flipper = self.get_parameter('largo_flipper').value
        self.radio_rueda = self.get_parameter('radio_rueda_flipper').value

        # Umbral de proximidad: los flippers se modelan como cápsulas de radio
        # radio_rueda alrededor del segmento motor->punta. Se tocan cuando la
        # distancia entre segmentos baja de la SUMA de radios (2*radio_rueda),
        # que es justo el alto de la caja de colisión del URDF. Al cuadrado para
        # ahorrarse la raíz cuadrada en el bucle.
        self.threshold_sq = (2.0 * self.radio_rueda) ** 2

        self.get_logger().info(
            f'kinematic_guardian: motor_x={self.motor_x} m, motor_z={self.motor_z} m, '
            f'largo_flipper={self.largo_flipper} m, radio_rueda={self.radio_rueda} m '
            f'(umbral {2.0 * self.radio_rueda:.3f} m)'
        )

        # 1. Escuchamos lo que el Humano/IA QUIERE hacer
        self.sub_raw = self.create_subscription(
            JointState,
            '/joint_states_raw', # El tópico del GUI o de Benjamín
            self.command_callback,
            10)
            
        # 2. Publicamos lo que el robot REALMENTE va a hacer
        self.pub_safe = self.create_publisher(
            JointState, 
            '/joint_states', # El tópico que lee Foxglove y el hardware
            10)

        # Memoria del último estado seguro
        self.last_safe_positions = {
            'flipper_fl': 0.0, 'flipper_fr': 0.0,
            'flipper_rl': 0.0, 'flipper_rr': 0.0
        }

    def command_callback(self, msg):
        safe_msg = JointState()
        safe_msg.header = msg.header
        safe_msg.name = msg.name
        
        # Extraer posiciones solicitadas
        req = dict(zip(msg.name, msg.position))
        
        # --- VERIFICACIÓN LADO IZQUIERDO ---
        if 'flipper_fl' in req and 'flipper_rl' in req:
            if self.is_collision(req['flipper_fl'], req['flipper_rl']):
                self.get_logger().warn('PELIGRO: Colisión Lado Izquierdo prevenida')
                req['flipper_fl'] = self.last_safe_positions['flipper_fl']
                req['flipper_rl'] = self.last_safe_positions['flipper_rl']
                
        # --- VERIFICACIÓN LADO DERECHO ---
        if 'flipper_fr' in req and 'flipper_rr' in req:
            if self.is_collision(req['flipper_fr'], req['flipper_rr']):
                self.get_logger().warn('PELIGRO: Colisión Lado Derecho prevenida')
                req['flipper_fr'] = self.last_safe_positions['flipper_fr']
                req['flipper_rr'] = self.last_safe_positions['flipper_rr']

        # Guardar como seguras y enviarlas
        for name in req:
            if name in self.last_safe_positions:
                self.last_safe_positions[name] = req[name]
            safe_msg.position.append(req[name])
            
        self.pub_safe.publish(safe_msg)

    def _point_to_segment_sq_dist(self, px, pz, x1, z1, x2, z2):
        """Calcula la distancia al cuadrado entre un punto y un segmento de línea.
           (Usamos al cuadrado para evitar el costo de la raíz cuadrada en el procesador)"""
        l2 = (x2 - x1)**2 + (z2 - z1)**2
        if l2 == 0:
            return (px - x1)**2 + (pz - z1)**2
        
        # Proyectar el punto sobre la línea y limitarlo a los extremos (0 a 1)
        t = max(0.0, min(1.0, ((px - x1)*(x2 - x1) + (pz - z1)*(z2 - z1)) / l2))
        proj_x = x1 + t * (x2 - x1)
        proj_z = z1 + t * (z2 - z1)
        
        return (px - proj_x)**2 + (pz - proj_z)**2

    def _segments_intersect(self, x1, z1, x2, z2, x3, z3, x4, z4):
        """Algoritmo clásico para evaluar si dos líneas finitas se cruzan como una X"""
        def ccw(ax, az, bx, bz, cx, cz):
            return (cz - az) * (bx - ax) > (bz - az) * (cx - ax)
        
        return (ccw(x1, z1, x3, z3, x4, z4) != ccw(x2, z2, x3, z3, x4, z4)) and \
               (ccw(x1, z1, x2, z2, x3, z3) != ccw(x1, z1, x2, z2, x4, z4))

    def is_collision(self, front_angle, rear_angle):
        """Motor de colisiones geométrico 2D exacto basado en URDF"""
        # 1. Definir los puntos base de los motores (parámetros = mismo YAML que el URDF)
        p1_x, p1_z = self.motor_x, self.motor_z    # Motor Delantero
        p2_x, p2_z = -self.motor_x, self.motor_z   # Motor Trasero

        # 2. Calcular la posición de las puntas en el espacio (Geometría Analítica)
        L = self.largo_flipper  # Largo exacto del flipper en metros

        # El delantero mira hacia adelante (+0 radianes base)
        t1_x = p1_x + L * math.cos(front_angle)
        t1_z = p1_z + L * math.sin(front_angle)
        
        # El trasero mira hacia atrás (+Pi radianes base por la rotación del joint)
        t2_x = p2_x + L * math.cos(rear_angle + math.pi)
        t2_z = p2_z + L * math.sin(rear_angle + math.pi)
        
        # 3. Prueba 1: ¿Se cruzaron violentamente como espadas?
        if self._segments_intersect(p1_x, p1_z, t1_x, t1_z, p2_x, p2_z, t2_x, t2_z):
            return True
            
        # 4. Prueba 2: ¿Se están rozando los bordes físicos?
        # Umbral = (2 * radio_rueda)^2, calculado una sola vez en __init__.
        threshold_sq = self.threshold_sq

        # Calculamos las 4 distancias posibles de punta a cuerpo
        d1 = self._point_to_segment_sq_dist(p1_x, p1_z, p2_x, p2_z, t2_x, t2_z)
        d2 = self._point_to_segment_sq_dist(t1_x, t1_z, p2_x, p2_z, t2_x, t2_z)
        d3 = self._point_to_segment_sq_dist(p2_x, p2_z, p1_x, p1_z, t1_x, t1_z)
        d4 = self._point_to_segment_sq_dist(t2_x, t2_z, p1_x, p1_z, t1_x, t1_z)
        
        if min(d1, d2, d3, d4) < threshold_sq:
            return True
            
        return False

def main(args=None):
    rclpy.init(args=args)
    node = KinematicGuardian()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()