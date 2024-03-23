#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hrwros_msgs.msg import SensorInformation

class SensorInfoSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_info_subscriber')
        self.subscription = self.create_subscription(
            SensorInformation,
            'sensor_info',
            self.sensor_info_callback,
            10)
        self.subscription  # prevent unused variable warning

    def sensor_info_callback(self, msg):
        self.get_logger().info('Distance reading from the sensor is: %f' % msg.sensor_data.range)

def main(args=None):
    rclpy.init(args=args)

    sensor_info_subscriber = SensorInfoSubscriber()

    rclpy.spin(sensor_info_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_info_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
