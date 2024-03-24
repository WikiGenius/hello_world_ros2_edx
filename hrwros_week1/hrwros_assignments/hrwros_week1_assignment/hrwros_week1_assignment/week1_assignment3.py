#! /usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time
from hrwros_msgs.action import CounterWithDelay


class CounterWithDelayActionServer(Node):

    def __init__(self):
        super().__init__('counter_with_delay')
        # Default delay value
        counter_delay_value = 1.0
        self.declare_parameter('counter_delay', counter_delay_value)

        self._action_server = ActionServer(
            self,
            CounterWithDelay,
            'counter_with_delay',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        feedback_msg = CounterWithDelay.Feedback()
        result_msg = CounterWithDelay.Result()

        #####################################################################
        #  Assignment 3 - Part3                                             #
        #  modify counter delay using "counter_delay" a private parameter.  #

        if self.has_parameter("counter_delay"):

            counter_delay_value = self.get_parameter('counter_delay').value
            self.get_logger().info(
                f"Parameter found on the parameter server: Using {counter_delay_value:.1f}s for counter delay.")
        else:
            self.get_logger().info(
                "Parameter not found on the parameter server: Using default value of 1.0s for counter delay.")

        # End of Assignment 3 - Part3                                       #
        #####################################################################

        success = True

        self.get_logger().info(
            f'counter_with_delay action server is counting up to {goal_handle.request.num_counts} with {counter_delay_value:.1f}s delay between each count')

        for counter_idx in range(0, goal_handle.request.num_counts):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                success = False
                break

            feedback_msg.counts_elapsed = counter_idx
            goal_handle.publish_feedback(feedback_msg)
            # Directly use counter_delay_value for sleep
            time.sleep(counter_delay_value)

        if success:
            result_msg.result_message = f"Successfully counting - Delay is {counter_delay_value:.1f} s"
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
            return result_msg


def main(args=None):
    rclpy.init(args=args)
    counter_with_delay_action_server = CounterWithDelayActionServer()

    rclpy.spin(counter_with_delay_action_server)

    counter_with_delay_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
