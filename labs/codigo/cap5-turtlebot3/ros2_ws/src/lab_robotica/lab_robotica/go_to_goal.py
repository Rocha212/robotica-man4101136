#!/usr/bin/env python3
"""Control proporcional para llevar TurtleBot3 a una coordenada (x, y)."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from .control_utils import clamp, wrap_angle, yaw_from_quaternion


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('kv', 0.5)
        self.declare_parameter('kh', 2.0)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('max_linear', 0.20)
        self.declare_parameter('max_angular', 1.0)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.kv = float(self.get_parameter('kv').value)
        self.kh = float(self.get_parameter('kh').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.goal_reached = False

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info(
            f'Go to goal iniciado. Meta=({self.goal_x:.2f}, {self.goal_y:.2f})'
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
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.publisher.publish(msg)

    def control(self):
        if not self.odom_received:
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.hypot(dx, dy)

        if distance <= self.tolerance:
            self.publish_velocity(0.0, 0.0)
            if not self.goal_reached:
                self.get_logger().info(
                    f'Meta alcanzada: x={self.x:.3f}, y={self.y:.3f}, '
                    f'error={distance:.3f} m'
                )
                self.goal_reached = True
            return

        self.goal_reached = False
        desired_yaw = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_yaw - self.yaw)

        # Se reduce la velocidad al estar cerca y se evita avanzar
        # rápidamente cuando el robot todavía apunta en otra dirección.
        linear = self.kv * distance * max(0.0, math.cos(heading_error))
        angular = self.kh * heading_error
        linear = clamp(linear, 0.0, self.max_linear)
        angular = clamp(angular, -self.max_angular, self.max_angular)

        self.publish_velocity(linear, angular)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
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
