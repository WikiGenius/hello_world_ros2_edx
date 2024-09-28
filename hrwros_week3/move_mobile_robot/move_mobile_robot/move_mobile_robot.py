import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class MoveMobileRobot(Node):
    def __init__(self, wait_timeout_server=5.0) -> None:
        super().__init__('move_mobile_robot')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # Get the goal coordinates from parameters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        
        self._connection = self._client.wait_for_server(wait_timeout_server)
        if not self._connection:
            self.get_logger().error("Action server /navigate_to_pose is not available!")
            exit(1)
        
    def send_goal(self):
        x = self.get_parameter('goal_x').value
        y = self.get_parameter('goal_y').value
        # Create a goal msg
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        # Send a goal msg the the action server
        self.get_logger().info(
            f"Sending Goal... x = {x:.3f}, y = {y:.3f} relative to the map")
        send_goal_future = self._client.send_goal_async(
            goal, self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Hooray! Successfully reached the desired goal!')
        else:
            self.get_logger().info('Navigation to the desired goal failed')
            self.get_logger().info(':( -- Sorry, try again!)')
            
    def feedback_callback(self, feedbackMsg):
        feedback = feedbackMsg.feedback
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        z = feedback.current_pose.pose.position.z
        self.get_logger().info(
            f"position:\n x = {x:.3f}, y = {y:.3f}, z = {z:.3f}")
        qx = feedback.current_pose.pose.orientation.x
        qy = feedback.current_pose.pose.orientation.y
        qz = feedback.current_pose.pose.orientation.z
        qw = feedback.current_pose.pose.orientation.w
        self.get_logger().info(
            f"orientation:\n x = {qx:.3f}, y = {qy:.3f}, z = {qz:.3f}, w = {qw:.3f}")
        navigation_time = feedback.navigation_time
        self.get_logger().info(f"navigation_time: {navigation_time.sec} sec")
        estimated_time_remaining = feedback.estimated_time_remaining
        self.get_logger().info(
            f"estimated_time_remaining: {estimated_time_remaining.sec} sec")
        number_of_recoveries = feedback.number_of_recoveries
        self.get_logger().info(f"number_of_recoveries: {number_of_recoveries}")
        distance_remaining = feedback.distance_remaining
        self.get_logger().info(
            f"distance_remaining: {distance_remaining:.3f} meter")
        self.get_logger().info("="*40)


def main(args=None):
    rclpy.init(args=args)
    node = MoveMobileRobot()
    node.send_goal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
