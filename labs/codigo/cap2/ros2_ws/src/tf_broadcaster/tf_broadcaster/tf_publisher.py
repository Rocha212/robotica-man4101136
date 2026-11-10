#!/usr/bin/env python3
"""Publica la transformación dinámica TF world -> robot."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class TFPublisher(Node):
    """Genera una transformación con traslación fija y giro alrededor de Z."""

    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_tf)
        self.angle = 0.0
        self.get_logger().info('Publicador TF iniciado: world -> robot.')

    def broadcast_tf(self):
        """Construye y envía una transformación TF."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'robot'

        transform.transform.translation.x = 2.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # El informe incrementa el ángulo en 0.1 rad por publicación.
        quaternion = R.from_euler('z', self.angle).as_quat()
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self.broadcaster.sendTransform(transform)
        self.angle += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
