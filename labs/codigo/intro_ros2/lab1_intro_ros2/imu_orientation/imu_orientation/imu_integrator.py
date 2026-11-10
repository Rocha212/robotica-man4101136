#!/usr/bin/env python3
"""
Integra la velocidad angular de una IMU y publica una transformación TF.

Suscripción:
    /imu (sensor_msgs/msg/Imu)

Transformación publicada:
    world -> imu_link
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster


class IMUIntegrator(Node):
    """Estima orientación integrando la velocidad angular de la IMU."""

    def __init__(self):
        super().__init__('imu_integrator')

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Orientación inicial: rotación identidad (sin giro).
        self.orientation = Rotation.identity()
        self.last_time = None

        self.get_logger().info(
            'Integrador de IMU iniciado. Esperando mensajes en /imu.'
        )

    def imu_callback(self, msg):
        """Integra la velocidad angular recibida y actualiza la transformación."""
        current_time = self.get_clock().now()

        # La primera muestra solo permite establecer el instante inicial.
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Evita integrar intervalos nulos o negativos.
        if dt <= 0.0:
            return

        angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)

        # El vector de rotación es velocidad angular por intervalo de tiempo.
        delta_rotation = Rotation.from_rotvec(angular_velocity * dt)

        # Actualiza la orientación acumulada.
        self.orientation = self.orientation * delta_rotation

        self.publish_tf(current_time)

    def publish_tf(self, current_time):
        """Publica la orientación estimada como transformación TF."""
        transform = TransformStamped()

        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'imu_link'

        # En esta práctica solo se estima orientación; la traslación es cero.
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # SciPy entrega el cuaternión en el orden x, y, z, w.
        quaternion = self.orientation.as_quat()

        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Inicializa y ejecuta el nodo integrador."""
    rclpy.init(args=args)
    node = IMUIntegrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
