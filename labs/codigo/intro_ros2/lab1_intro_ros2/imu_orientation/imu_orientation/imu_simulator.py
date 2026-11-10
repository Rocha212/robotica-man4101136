#!/usr/bin/env python3
"""
Nodo que simula una IMU y publica velocidad angular en el tópico /imu.

Valores predeterminados:
    wx = 0.0 rad/s
    wy = 0.0 rad/s
    wz = 0.5 rad/s
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUSimulator(Node):
    """Publica mensajes IMU con una velocidad angular configurable."""

    def __init__(self):
        super().__init__('imu_simulator')

        # Parámetros para modificar la velocidad angular sin editar el código.
        self.declare_parameter('angular_velocity_x', 0.0)
        self.declare_parameter('angular_velocity_y', 0.0)
        self.declare_parameter('angular_velocity_z', 0.5)

        self.publisher_ = self.create_publisher(Imu, '/imu', 10)

        # Publica datos cada 0.01 s (100 Hz), como en el informe.
        self.timer = self.create_timer(0.01, self.publish_imu)

        self.get_logger().info(
            'Simulador de IMU iniciado. Publicando datos en /imu.'
        )

    def publish_imu(self):
        """Construye y publica un mensaje sensor_msgs/Imu."""
        msg = Imu()

        msg.angular_velocity.x = float(
            self.get_parameter('angular_velocity_x').value
        )
        msg.angular_velocity.y = float(
            self.get_parameter('angular_velocity_y').value
        )
        msg.angular_velocity.z = float(
            self.get_parameter('angular_velocity_z').value
        )

        self.publisher_.publish(msg)


def main(args=None):
    """Inicializa y ejecuta el nodo simulador."""
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
