#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from hrwros_msgs.action import CounterWithDelay

class CounterWithDelayClientAsync(Node):
    def __init__(self):
        super().__init__('counter_with_delay_ac')
        self._action_client = ActionClient(
            self, CounterWithDelay, 'counter_with_delay')

    def send_goal(self, num_counts):
        goal_msg = CounterWithDelay.Goal()
        goal_msg.num_counts = num_counts

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)  # Set feedback callback here
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.counts_elapsed} counts elapsed')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result_message))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = CounterWithDelayClientAsync()

    action_client.send_goal(5)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
