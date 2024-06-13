#include "hrwros_gazebo/conveyor_spawner.h"

int main(int argc, char **argv)
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<hrwros::simulation::ConveyorSpawner>();

  // Declare and get parameters
  rclcpp::Parameter spawner_params;
  if (!node->get_parameter("spawner", spawner_params))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get spawner parameters");
    return -1;
  }

  if (!node->init(spawner_params))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize spawner");
    return -2;
  }

  // Run the node
  node->run();

  // Keep the node running
  rclcpp::spin(node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();

  return 0;
}