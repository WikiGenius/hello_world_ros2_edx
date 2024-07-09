#ifndef GILBRETH_SUPPORT_CONVEYOR_SPAWNER_HPP
#define GILBRETH_SUPPORT_CONVEYOR_SPAWNER_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/header.hpp>       // Include for std_msgs::msg::Header
#include <gazebo_msgs/srv/spawn_entity.hpp>  // Include for gazebo_msgs::srv::SpawnEntity
#include <urdf/model.h>
#include <random>
#include <vector>
#include <string>

namespace hrwros
{
namespace simulation
{

struct ObjectParameters
{
  std::string name;
  std::string mesh_resource;
  geometry_msgs::msg::Pose initial_pose;
  double lateral_placement_variance;
  double yaw_placement_variance;
  double spawn_timing_variance;
};

struct SpawnParameters
{
  std::string reference_frame;
  double spawn_period;
  int randomization_seed;
  std::vector<ObjectParameters> objects;
};

class ConveyorSpawner : public rclcpp::Node
{
public:
  ConveyorSpawner();

  bool init();
  void run();

private:
  bool loadSpawnParameters();
  bool loadObjectParameters(const rclcpp::Parameter &param, ObjectParameters &object_params);
  bool connectToROS();

  void start(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void spawnObject();

  int object_counter_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_;
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  SpawnParameters params_;
  std::default_random_engine random_engine_;
};

} // namespace simulation
} // namespace hrwros

#endif // GILBRETH_SUPPORT_CONVEYOR_SPAWNER_HPP
