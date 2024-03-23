#!/usr/bin/env python3

from hrwros_msgs.srv import ConvertMetresToFeet
import rclpy
from rclpy.node import Node
import numpy as np

_CONVERSION_FACTOR_METRES_TO_FEET = 3.28  # Metres -> Feet conversion factor.

class MetresToFeetService(Node):

    def __init__(self):
        super().__init__('metres_to_feet_server')
        self.srv = self.create_service(ConvertMetresToFeet, 'metres_to_feet', self.process_service_request)
        self.get_logger().info('Convert metres to feet service is now available.')

    def process_service_request(self, request, response):
        # Perform sanity check. Allow only positive real numbers.
        # Compose the response message accordingly.
        if request.distance_metres < 0:
            response.success = False
            response.distance_feet = -np.Inf  # Default error value.
        else:
            response.distance_feet = _CONVERSION_FACTOR_METRES_TO_FEET * request.distance_metres
            response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)

    metres_to_feet_service = MetresToFeetService()

    rclpy.spin(metres_to_feet_service)

    # Destroy the service server explicitly
    metres_to_feet_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
