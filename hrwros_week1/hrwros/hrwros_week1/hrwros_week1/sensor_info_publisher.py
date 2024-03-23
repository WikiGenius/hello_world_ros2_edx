#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hrwros_msgs.msg import SensorInformation
from hrwros_utilities.sim_sensor_data import distSensorData as getSensorData
from rclpy.clock import ROSClock

class SensorInfoPublisher(Node):

    def __init__(self):
        super().__init__('sensor_info_publisher')
        self.publisher = self.create_publisher(SensorInformation, 'sensor_info', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sensor_info = SensorInformation()

        # Initialize sensor information here
        self.sensor_info.sensor_data.header.frame_id = 'distance_sensor_frame'
        self.sensor_info.sensor_data.radiation_type = self.sensor_info.sensor_data.ULTRASOUND
        self.sensor_info.sensor_data.field_of_view = 0.5
        self.sensor_info.sensor_data.min_range = 0.02
        self.sensor_info.sensor_data.max_range = 2.0
        self.sensor_info.maker_name = 'The Ultrasound Company'
        self.sensor_info.part_number = 123456

    def timer_callback(self):
        # Update the timestamp in the header
        self.sensor_info.sensor_data.header.stamp = self.get_clock().now().to_msg()

        # Simulate reading sensor data
        self.sensor_info.sensor_data.range = getSensorData(
            self.sensor_info.sensor_data.radiation_type,
            self.sensor_info.sensor_data.min_range,
            self.sensor_info.sensor_data.max_range)

        # Publish the sensor information
        self.publisher.publish(self.sensor_info)
        self.get_logger().info('Publishing sensor information')

def main(args=None):
    rclpy.init(args=args)

    sensor_info_publisher = SensorInfoPublisher()

    try:
        rclpy.spin(sensor_info_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        sensor_info_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
