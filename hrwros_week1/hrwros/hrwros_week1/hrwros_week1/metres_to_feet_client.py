#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from hrwros_msgs.srv import ConvertMetresToFeet

class MetresToFeetClientNode(Node):
    def __init__(self):
        super().__init__('metres_to_feet_client')
        self.client = self.create_client(ConvertMetresToFeet, 'metres_to_feet')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ConvertMetresToFeet.Request()

    def send_request(self, x):
        self.req.distance_metres = x
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    metres_to_feet_client_node = MetresToFeetClientNode()
    dist_metres = 0.25
    metres_to_feet_client_node.get_logger().info(f'Requesting conversion of {dist_metres} m to feet')

    metres_to_feet_client_node.send_request(dist_metres)

    while rclpy.ok():
        rclpy.spin_once(metres_to_feet_client_node)
        if metres_to_feet_client_node.future.done():
            try:
                response = metres_to_feet_client_node.future.result()
                if not response.success:
                    metres_to_feet_client_node.get_logger().error("Conversion unsuccessful! Requested distance in metres should be a positive real number.")
                else:
                    metres_to_feet_client_node.get_logger().info(f'{dist_metres} m = {response.distance_feet} feet\nConversion successful!')
            except Exception as e:
                metres_to_feet_client_node.get_logger().info(f'Service call failed: {e}')
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()
