import rclpy
from rclpy.node import Node
import os

from gazebo_msgs.srv import SpawnEntity, GetEntityState
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('spawn_unknown_obstacle')

        self.spawn_obstacle_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_obstacle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn_entity not available, waiting...')

        self.check_obstacle_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.check_obstacle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /gazebo/get_entity_state not available, waiting...')

        package_share_directory = get_package_share_directory('hrwros_support')
        urdf_file_path = os.path.join(package_share_directory, 'urdf/unknown_obstacle/unknown_obstacle.urdf')
        
        with open(urdf_file_path, 'r') as box_file:
            self.box_xml = box_file.read()

        self.get_logger().info(self.box_xml )
        # Wait for a couple of seconds to prevent the unknown obstacle from
        # being considered as a part of the map.
        self.tmr = self.create_timer(4.0, self.spawn_obstacles)
        
    def spawn_unknown_obstacle(self, obstacle_name, obstacle_model_xml, obstacle_pose):
        spawn_obstacle_request = SpawnEntity.Request()
        spawn_obstacle_request.name = obstacle_name
        spawn_obstacle_request.xml = obstacle_model_xml
        spawn_obstacle_request.robot_namespace = ''
        spawn_obstacle_request.initial_pose = obstacle_pose
        spawn_obstacle_request.reference_frame = 'map'
        # self.get_logger().info(spawn_obstacle_request )

        future = self.spawn_obstacle_client.call_async(spawn_obstacle_request)
        rclpy.spin_until_future_complete(self, future)
        spawn_obstacle_response = future.result()

        if not spawn_obstacle_response.success:
            self.get_logger().error('Could not spawn unknown obstacle')
            self.get_logger().error(spawn_obstacle_response.status_message)
        else:
            self.get_logger().info(spawn_obstacle_response.status_message)

    def check_obstacle_existence(self, obstacle_name):
        check_obstacle_request = GetEntityState.Request()
        check_obstacle_request.name = obstacle_name
        self.get_logger().info(f"{obstacle_name}:")
        self.get_logger().info(f"{check_obstacle_request}:")

        future = self.check_obstacle_client.call_async(check_obstacle_request)
        self.get_logger().info(f"{future}:")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"After spin_until_future_complete")

        check_obstacle_response = future.result()
        self.get_logger().info(f"{obstacle_name}: {check_obstacle_response}")
        return check_obstacle_response.success

    def spawn_obstacles(self):
        self.get_logger().info("timer called" )
        # self.get_logger().info(self.check_obstacle_existence('unknown_obstacle1'))
        if not self.check_obstacle_existence('unknown_obstacle1'):
            obstacle1_pose = Pose()
            obstacle1_pose.position.x = -2.2
            obstacle1_pose.position.y = 1.0
            obstacle1_pose.position.z = 0.0
            obstacle1_pose.orientation.x = 0.0
            obstacle1_pose.orientation.y = 0.0
            obstacle1_pose.orientation.z = 0.0
            obstacle1_pose.orientation.w = 1.0
            self.spawn_unknown_obstacle('unknown_obstacle1', self.box_xml, obstacle1_pose)

        if not self.check_obstacle_existence('unknown_obstacle2'):
            obstacle2_pose = Pose()
            obstacle2_pose.position.x = -5.0
            obstacle2_pose.position.y = 0.5
            obstacle2_pose.position.z = 0.0
            obstacle2_pose.orientation.x = 0.0
            obstacle2_pose.orientation.y = 0.0
            obstacle2_pose.orientation.z = 0.0
            obstacle2_pose.orientation.w = 1.0
            self.spawn_unknown_obstacle('unknown_obstacle2', self.box_xml, obstacle2_pose)

        # Destroy the timer once the obstacles are spawned
        self.destroy_timer(self.tmr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
