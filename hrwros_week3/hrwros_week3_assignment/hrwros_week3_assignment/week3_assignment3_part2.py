import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

class MoveTurtlebot(Node):

    def __init__(self):
        super().__init__('move_turtlebot')

        # Create an ActionClient for the move_base action server.
        self._client = ActionClient(self, NavigateToPose, '<Add-action-server-name>')

    def send_goal(self, x, y):
        # Create a goal message to send to the action server.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Send the goal to the action server.
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Hooray! Successfully reached the desired goal!')
        else:
            self.get_logger().info('Navigation to the desired goal failed')
            self.get_logger().info(':( -- Sorry, try again!)')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current position: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtlebot()
    node.send_goal(<Add-second-goal-Position-X>, <Add-second-goal-Position-Y>)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt received to stop ROS node.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
