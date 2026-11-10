#!/usr/bin/env python3
"""Seguimiento de la recta a*x + b*y + c = 0 con TurtleBot3."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from .control_utils import clamp, wrap_angle, yaw_from_quaternion


class LineFollowing(Node):
    def __init__(self):
        super().__init__('line_following')

        # Recta de la guía: x - 2y + 4 = 0.
        self.declare_parameter('a', 1.0)
        self.declare_parameter('b', -2.0)
        self.declare_parameter('c', 4.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('kh', 2.0)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('max_angular', 1.0)

        self.a = float(self.get_parameter('a').value)
        self.b = float(self.get_parameter('b').value)
        self.c = float(self.get_parameter('c').value)
        self.kd = float(self.get_parameter('kd').value)
        self.kh = float(self.get_parameter('kh').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        if math.hypot(self.a, self.b) == 0.0:
            raise ValueError('Los coeficientes a y b no pueden ser ambos cero.')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.timer = self.create_timer(0.1, self.control)

        # Dirección de la recta según la convención de la guía.
        self.line_yaw = math.atan2(self.a, -self.b)
        self.get_logger().info(
            f'Seguimiento de línea: {self.a}x + {self.b}y + {self.c} = 0'
        )

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def control(self):
        if not self.odom_received:
            return

        # Distancia perpendicular con signo: indica en qué lado de la recta está.
        distance = (
            self.a * self.x + self.b * self.y + self.c
        ) / math.hypot(self.a, self.b)

        heading_error = wrap_angle(self.line_yaw - self.yaw)

        # La guía plantea delta = Kd*distance + Kh*error_heading.
        # En el diferencial convertimos esa corrección a velocidad angular.
        steering_correction = self.kd * distance + self.kh * heading_error
        linear = self.linear_speed
        angular = (linear / 0.20) * steering_correction
        angular = clamp(angular, -self.max_angular, self.max_angular)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.publisher.publish(msg)

        self.get_logger().info(
            f'distancia a la recta={distance:.3f} m',
            throttle_duration_sec=2.0
        )

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
