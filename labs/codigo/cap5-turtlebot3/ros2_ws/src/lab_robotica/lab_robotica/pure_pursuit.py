#!/usr/bin/env python3
"""Seguimiento de waypoints tipo Pure Pursuit simplificado para TurtleBot3.

Se selecciona el siguiente waypoint cuando el robot entra en el radio lookahead.
La corrección angular se calcula hacia ese punto. Esta adaptación conserva la
lógica de la guía del capítulo 4 y la expresa para un robot diferencial.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from .control_utils import clamp, wrap_angle, yaw_from_quaternion


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('lookahead', 0.5)
        self.declare_parameter('kd', 1.5)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('max_angular', 1.0)

        self.lookahead = float(self.get_parameter('lookahead').value)
        self.kd = float(self.get_parameter('kd').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        # Ruta usada en la implementación ROS del informe del capítulo 4.
        self.path = [(0.0, 0.0), (1.0, 0.5), (2.0, 0.5), (2.5, 2.0)]
        self.index = 1  # El primer waypoint es la posición inicial.

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.finished = False

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info(
            f'Pure Pursuit iniciado: lookahead={self.lookahead:.2f} m'
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def publish_velocity(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.publisher.publish(msg)

    def control(self):
        if not self.odom_received or self.finished:
            return

        target_x, target_y = self.path[self.index]
        distance = math.hypot(target_x - self.x, target_y - self.y)

        # Avanza de waypoint cuando entra en el radio de anticipación.
        if distance < self.lookahead and self.index < len(self.path) - 1:
            self.index += 1
            target_x, target_y = self.path[self.index]
            distance = math.hypot(target_x - self.x, target_y - self.y)

        # Para en el último waypoint al alcanzar la tolerancia.
        if self.index == len(self.path) - 1 and distance <= self.tolerance:
            self.publish_velocity(0.0, 0.0)
            self.finished = True
            self.get_logger().info(
                f'Último waypoint alcanzado: ({self.x:.3f}, {self.y:.3f})'
            )
            return

        desired_yaw = math.atan2(target_y - self.y, target_x - self.x)
        error = wrap_angle(desired_yaw - self.yaw)

        # Control diferencial: linear.x y angular.z.
        linear = self.linear_speed * max(0.0, math.cos(error))
        angular = clamp(self.kd * error, -self.max_angular, self.max_angular)
        self.publish_velocity(linear, angular)

        self.get_logger().info(
            f'waypoint {self.index + 1}/{len(self.path)}; '
            f'error={distance:.3f} m',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
