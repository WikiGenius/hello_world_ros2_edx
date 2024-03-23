#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from hrwros_msgs.action import CounterWithDelay
from hrwros_msgs.msg import CounterWithDelayFeedback, CounterWithDelayResult

class CounterWithDelayActionServer(Node):
    def __init__(self):
        super().__init__('counter_with_delay_action_server')
        self._action_server = ActionServer(
            self,
            CounterWithDelay,
            'counter_with_delay',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        feedback_msg = CounterWithDelayFeedback()
        result = CounterWithDelayResult()

        counter_delay_value = 1.0  # Default delay value
        # Check if the parameter exists and get its value
        if self.has_parameter('counter_delay'):
            counter_delay_value = self.get_parameter('counter_delay').value

        delay_rate = self.create_rate(1.0 / counter_delay_value)

        success = True
        for counter_idx in range(0, goal_handle.request.num_counts):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return CounterWithDelay.Result()

            feedback_msg.counts_elapsed = counter_idx
            goal_handle.publish_feedback(feedback_msg)
            delay_rate.sleep()

        if success:
            result.result_message = "Successfully completed counting."
            goal_handle.succeed()
            return result

def main(args=None):
    rclpy.init(args=args)

    counter_with_delay_action_server = CounterWithDelayActionServer()

    rclpy.spin(counter_with_delay_action_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
