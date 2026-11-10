#!/usr/bin/env python3
"""Publica una pose fija en el tópico /pose cada 0.5 segundos."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


class PosePublisher(Node):
    """Publicador ROS 2 de posición y orientación."""

    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.get_logger().info('Publicador de pose iniciado en /pose.')

    def publish_pose(self):
        """Construye y publica la pose definida en el laboratorio."""
        msg = Pose()

        msg.position.x = 1.0
        msg.position.y = 2.0
        msg.position.z = 0.0

        rotation = R.from_euler('z', 90, degrees=True)
        quaternion = rotation.as_quat()

        # geometry_msgs/Quaternion también usa los campos x, y, z, w.
        msg.orientation.x = float(quaternion[0])
        msg.orientation.y = float(quaternion[1])
        msg.orientation.z = float(quaternion[2])
        msg.orientation.w = float(quaternion[3])

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
