#! /usr/bin/env python
# All necessary python imports go here.
import rclpy
from rclpy.node import Node
# Copy the code from Part1 here
from hrwros_msgs.msg import <write-your-code-here-Part1>
# Import the correct message type for part 3
from hrwros_week1_assignment.msg import <write-your-code-here-Part3>

class BoxHeightPublisher(Node):

    def __init__(self):
        super().__init__('compute_box_height')
        # Create the publisher for Part3 here
        self.bhi_publisher = self.create_publisher(<write-your-code-here-Part3>, '<use correct topic name here>', 10)
        # Copy the subscriber from Part1 here, adjusted for ROS 2
        self.subscription = self.create_subscription(
            <write-your-code-here-Part1>, '<use correct topic name here>', self.sensor_info_callback, 10)
        self.subscription  # prevent unused variable warning

    def sensor_info_callback(self, data):
        # Copy the code from Part1 here
        height_box = <write-your-code-here-Part1>

        # Compute the height of the box.
        # Boxes that are detected to be shorter than 10cm are due to sensor noise.
        # Do not publish information about them.
        # Copy the code from Part1 here
        if <write-your-code-here-Part1>:
            pass
        else:
            # Declare a message object for publishing the box height information.
            box_height_info = <write-your-code-here-Part3>
            # Update height of box.
            <write-your-code-here-Part3>
            # Publish box height using the publisher argument passed to the
            # callback function.
            <write-your-code-here-Part3>
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
