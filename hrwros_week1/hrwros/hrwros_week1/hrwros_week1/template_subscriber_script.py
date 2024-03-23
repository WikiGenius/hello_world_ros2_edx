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
        sensor_range = msg.sensor_data.range
        self.get_logger().info(
            f'Received sensor data: Range = {round(sensor_range, 3)} meters')


def main(args=None):
    rclpy.init(args=args)

    sensor_info_subscriber = SensorInfoSubscriber()

    rclpy.spin(sensor_info_subscriber)

    # Cleanup
    sensor_info_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
