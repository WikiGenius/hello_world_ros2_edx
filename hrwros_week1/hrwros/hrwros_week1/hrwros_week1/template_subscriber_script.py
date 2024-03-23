#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringSubscriber(Node):

    def __init__(self):
        super().__init__('node_2')
        self.subscription = self.create_subscription(
            String,
            'topic_1',
            self.string_listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def string_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    string_subscriber = StringSubscriber()

    rclpy.spin(string_subscriber)

    # Cleanup and shutdown
    string_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
