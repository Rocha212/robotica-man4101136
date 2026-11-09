import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        self.timer = self.create_timer(
            0.1,
            self.move_robot
        )

    def move_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.2
        msg.twist.angular.z = 0.1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
