#!/usr/bin/env python3
"""Nodo ROS 2 que simula una IMU y publica velocidad angular a 100 Hz."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUSimulator(Node):
    """Publica mensajes sensor_msgs/Imu con velocidad angular configurable."""

    def __init__(self):
        super().__init__('imu_simulator')

        self.declare_parameter('angular_velocity_x', 0.0)
        self.declare_parameter('angular_velocity_y', 0.0)
        self.declare_parameter('angular_velocity_z', 0.5)

        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)

        self.get_logger().info(
            'Simulador IMU activo: tópico /imu, frecuencia 100 Hz.'
        )

    def publish_imu(self):
        """Publica una muestra con las velocidades angulares configuradas."""
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'imu_link'

        message.angular_velocity.x = float(
            self.get_parameter('angular_velocity_x').value
        )
        message.angular_velocity.y = float(
            self.get_parameter('angular_velocity_y').value
        )
        message.angular_velocity.z = float(
            self.get_parameter('angular_velocity_z').value
        )

        # Covarianza cero: el laboratorio usa una IMU ideal simulada.
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
