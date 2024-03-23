#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hrwros_msgs.msg import SensorInformation
import random


class SensorInfoPublisher(Node):

    def __init__(self):
        super().__init__('sensor_info_publisher')
        self.publisher = self.create_publisher(
            SensorInformation, 'sensor_info', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_info)

    def publish_sensor_info(self):
        msg = SensorInformation()

        # Assuming you want to set the frame_id to some value,
        # and the range to a fixed or randomly generated number for demonstration.
        msg.sensor_data.header.stamp = self.get_clock().now().to_msg()
        msg.sensor_data.header.frame_id = "distance_sensor_frame"
        msg.sensor_data.radiation_type = msg.sensor_data.ULTRASOUND
        msg.sensor_data.field_of_view = 0.5  # Example value in radians
        msg.sensor_data.min_range = 0.02  # Example min range in meters
        msg.sensor_data.max_range = 2.0  # Example max range in meters

        # Simulate sensor data with randomized range values
        simulated_range = random.uniform(
            msg.sensor_data.min_range,  # Minimum range in meters
            msg.sensor_data.max_range)  # Maximum range in meters

        # Example distance measurement in meters
        msg.sensor_data.range = simulated_range
        
        msg.maker_name = "The Ultrasound Company"
        msg.part_number = 123456

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Publishing Sensor Information: Range = {round(msg.sensor_data.range, 3)} meters')


def main(args=None):
    rclpy.init(args=args)

    sensor_info_publisher = SensorInfoPublisher()

    rclpy.spin(sensor_info_publisher)

    sensor_info_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
