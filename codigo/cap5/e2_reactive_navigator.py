# Nodo reactivo - Parte E (Actividad E2)
# /cmd_vel usa TwistStamped porque el puente ros_gz de esta simulación lo espera así.
# angle_min = 0, angle_max = 2*pi, 1 grado por muestra: el frente está al inicio y al final del arreglo.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np

FRONT_HALF_WIDTH = 30   # muestras a cada lado de 0 rad (~30 grados)
OBSTACLE_DIST = 0.5     # metros

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def make_cmd(self, linear, angular):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        return cmd

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=float)
        k = FRONT_HALF_WIDTH
        front_vals = np.concatenate((ranges[-k:], ranges[:k]))
        front_vals = front_vals[np.isfinite(front_vals)]
        front = np.min(front_vals) if len(front_vals) > 0 else np.inf

        if front < OBSTACLE_DIST:
            self.pub.publish(self.make_cmd(0.0, 0.8))   # frenar y girar a la izquierda
        else:
            self.pub.publish(self.make_cmd(0.2, 0.0))   # avanzar

def main():
    rclpy.init()
    node = ReactiveNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(node.make_cmd(0.0, 0.0))   # detener el robot al salir
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
