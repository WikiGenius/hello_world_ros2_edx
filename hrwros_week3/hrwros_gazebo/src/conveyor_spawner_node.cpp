#include "hrwros_gazebo/conveyor_spawner.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hrwros::simulation::ConveyorSpawner>();

  if (!node->init())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize spawner");
    return -1;
  }

  node->run();

  rclcpp::shutdown();
  return 0;
}
