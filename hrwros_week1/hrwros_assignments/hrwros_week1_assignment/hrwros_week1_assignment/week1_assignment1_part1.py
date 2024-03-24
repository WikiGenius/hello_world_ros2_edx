#! /usr/bin/env python
# All necessary python imports go here.
import rclpy
from rclpy.node import Node

from hrwros_msgs.msg import SensorInformation


class BoxHeightCalculator(Node):

    def __init__(self):
        super().__init__('compute_box_height')
        # Create the subscriber for Part1 here
        self.subscription = self.create_subscription(
            SensorInformation, 'sensor_info', self.sensor_info_callback, 10)
        self.subscription  # prevent unused variable warning

    def sensor_info_callback(self, data):
        # Compute the height of the box from the sensor reading.
        # Tip: You need to substract the reading from the max_range of the sensor.
        MAX_RANGE_SENSOR = data.sensor_data.max_range
        height_box = MAX_RANGE_SENSOR - data.sensor_data.range

        # Compute the height of the box.
        # Boxes that are detected to be shorter than 0.1m are due to sensor noise.
        # Do not publish information about them.
        SHORTER_BOX = 0.1
        if height_box < SHORTER_BOX:
            pass
            self.get_logger().info('Filter PASS')
        else:
            # If implemented correctly only the height of boxes bigger than 0.1m
            # will be printed
            self.get_logger().info('Height of box %0.3f' % height_box)


def main(args=None):
    rclpy.init(args=args)
    box_height_calculator = BoxHeightCalculator()

    # Use a try-except block to catch KeyboardInterrupt and shutdown gracefully
    try:
        rclpy.spin(box_height_calculator)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        box_height_calculator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
