#! /usr/bin/env python
# All necessary python imports go here.
import rclpy
from rclpy.node import Node
# Copy the code from Part1 here
from hrwros_msgs.msg import SensorInformation
# Import the correct message type for part 3
from hrwros_msgs.msg import BoxHeightInformation

class BoxHeightPublisher(Node):

    def __init__(self):
        super().__init__('compute_box_height')
        # Create the publisher for Part3 here
        self.bhi_publisher = self.create_publisher(BoxHeightInformation, 'box_height_info', 10)
        # Copy the subscriber from Part1 here, adjusted for ROS 2
        self.subscription = self.create_subscription(
            SensorInformation, 'sensor_info', self.sensor_info_callback, 10)

    def sensor_info_callback(self, data):
        # Copy the code from Part1 here
        MAX_RANGE_SENSOR = data.sensor_data.max_range
        height_box = MAX_RANGE_SENSOR - data.sensor_data.range

        # Compute the height of the box.
        # Boxes that are detected to be shorter than 10cm are due to sensor noise.
        # Do not publish information about them.
        # Copy the code from Part1 here
        SHORTER_BOX = 0.1
        if height_box < SHORTER_BOX:
            pass
            self.get_logger().info('Filter PASS')
        else:
            # Declare a message object for publishing the box height information.
            box_height_info = BoxHeightInformation()
            # Update height of box.
            box_height_info.box_height = height_box
            # Publish box height using the publisher argument passed to the
            # callback function.
            self.get_logger().info('Height of box %0.3f' % height_box)
            self.bhi_publisher.publish(box_height_info)

def main(args=None):
    rclpy.init(args=args)
    node = BoxHeightPublisher()

    # Use a try-except block to catch KeyboardInterrupt and shutdown gracefully
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
