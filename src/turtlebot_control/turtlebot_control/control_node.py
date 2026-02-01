#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped


class ClickControlNode(Node):
    def __init__(self):
        super().__init__('click_control_node')

        # Publisher to TurtleBot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to RViz clicked point
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10
        )

        self.get_logger().info('Click-based control node started')
        self.get_logger().info('Click ABOVE center -> forward')
        self.get_logger().info('Click BELOW center -> backward')

        # Virtual "image center" Y (arbitrary reference)
        self.center_y = 0.0

    def click_callback(self, msg: PointStamped):
        twist = Twist()

        y = msg.point.y

        if y > self.center_y:
            twist.linear.x = 0.2
            self.get_logger().info('Clicked ABOVE center -> moving forward')
        else:
            twist.linear.x = -0.2
            self.get_logger().info('Clicked BELOW center -> moving backward')

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ClickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
