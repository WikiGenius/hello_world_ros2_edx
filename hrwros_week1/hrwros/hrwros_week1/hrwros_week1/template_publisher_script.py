#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('node_1')
        self.publisher = self.create_publisher(String, 'topic_1', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.topic_content = "Welcome to Hello (Real) World with ROS!!!"

    def publish_message(self):
        msg = String()
        msg.data = self.topic_content
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    simple_publisher = SimplePublisher()

    rclpy.spin(simple_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
