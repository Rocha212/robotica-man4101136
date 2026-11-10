#!/usr/bin/env python3
"""Integra la velocidad angular recibida y publica TF2 world -> imu_link."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster


class IMUIntegrator(Node):
    """Estima orientación integrando las muestras de velocidad angular."""

    def __init__(self):
        super().__init__('imu_integrator')

        self.orientation = R.identity()
        self.last_stamp = None

        self.subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Integrador IMU activo: publicando TF world -> imu_link.'
        )

    def imu_callback(self, message):
        """Calcula dt, integra el vector de velocidad angular y publica TF."""
        stamp = message.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        if self.last_stamp is None:
            self.last_stamp = current_time
            return

        dt = current_time - self.last_stamp
        self.last_stamp = current_time

        if dt <= 0.0:
            return

        angular_velocity = np.array([
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z
        ], dtype=float)

        delta_rotation = R.from_rotvec(angular_velocity * dt)
        self.orientation = self.orientation * delta_rotation

        self.publish_transform(stamp)

    def publish_transform(self, stamp):
        """Publica la orientación estimada como transformación dinámica TF2."""
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'imu_link'

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        quaternion = self.orientation.as_quat()
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
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
