#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from hrwros_msgs.msg import BoxHeightInformation
from hrwros_msgs.srv import ConvertMetresToFeet

class BoxHeightFeetConverter(Node):

    def __init__(self):
        super().__init__('box_height_in_feet')
        # First wait for the service to become available - Part2.
        self.get_logger().info("Waiting for subscription...")
        # Create a subscriber to the box height topic - Part1.
        self.subscription = self.create_subscription(
            BoxHeightInformation, 'box_height_info', self.box_height_info_callback, 10)

        # Create a client for the service to convert metres to feet - Part2.
        self.metres_to_feet_client = self.create_client(ConvertMetresToFeet, 'metres_to_feet')

    def box_height_info_callback(self, data):
        try:
            # Wait for the service to be available
            while not self.metres_to_feet_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            # Create a request object
            request = ConvertMetresToFeet.Request()
            # Set request data here
            request.distance_metres = data.box_height

            # Call the service here.
            self.future = self.metres_to_feet_client.call_async(request)
            self.future.add_done_callback(self.future_callback)

        except Exception as e:
            self.get_logger().info("Service call failed: %s" % e)

    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(('Height of box %0.3f Feet' % response.distance_feet))
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = BoxHeightFeetConverter()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


