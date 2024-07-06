#include <rclcpp/rclcpp.hpp>
#include "hrwros_gazebo/urdf_creator.h"
#include <hrwros_gazebo_interface/srv/spawn_entity.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("urdf_creator_test");

  std::string reference_frame = node->declare_parameter<std::string>("reference_frame", "world");
  std::string object_name = node->declare_parameter<std::string>("object_name", "test");
  std::string mesh_resource = node->declare_parameter<std::string>("mesh_resource", "package://hrwros_gazebo/meshes/conveyor_objects/gear.stl");
  std::vector<double> initial_position = node->declare_parameter<std::vector<double>>("initial_position", {1.2, 1.0, 1.0});

  auto client = node->create_client<gazebo_msgs::srv::SpawnEntity>("gazebo/spawn_entity");
  if (!client->wait_for_service(std::chrono::duration<double>(10.0)))
  {
    RCLCPP_ERROR(node->get_logger(), "Timeout waiting for service");
    return -1;
  }

  std::string parsed_xml = hrwros::simulation::createObjectURDF("gear", mesh_resource);

  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = object_name;
  request->xml = parsed_xml;
  request->initial_pose.orientation.w = 1.0;
  request->initial_pose.position.x = initial_position[0];
  request->initial_pose.position.y = initial_position[1];
  request->initial_pose.position.z = initial_position[2];
  request->reference_frame = reference_frame;
  request->robot_namespace = "hrwros";

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
    return -1;
  }

  auto response = result.get();
  if (response->success)
  {
    RCLCPP_INFO(node->get_logger(), "%s", response->status_message.c_str());
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "%s", response->status_message.c_str());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
