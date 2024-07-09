#include "hrwros_gazebo/conveyor_spawner.hpp"
#include "hrwros_gazebo/urdf_creator.h"
#include <std_msgs/msg/header.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <vector>
#include <random>

namespace hrwros
{
namespace simulation
{

ConveyorSpawner::ConveyorSpawner()
  : Node("conveyor_spawner"), object_counter_(0)
{
}

bool ConveyorSpawner::init()
{
  // Load the parameters
  if (!loadSpawnParameters())
  {
    return false;
  }

  // Connect to ROS topics/services/actions/etc.
  if (!connectToROS())
  {
    return false;
  }

  // Initialize the randomization engine
  random_engine_.seed(params_.randomization_seed);

  start_server_ = this->create_service<std_srvs::srv::Empty>("start_spawn", std::bind(&ConveyorSpawner::start, this, std::placeholders::_1, std::placeholders::_2));
  stop_server_ = this->create_service<std_srvs::srv::Empty>("stop_spawn", std::bind(&ConveyorSpawner::stop, this, std::placeholders::_1, std::placeholders::_2));
  timer_ = this->create_wall_timer(std::chrono::duration<double>(params_.spawn_period), std::bind(&ConveyorSpawner::spawnObject, this));
  pub_ = this->create_publisher<std_msgs::msg::Header>("spawned_part", 10);

  return true;
}

void ConveyorSpawner::run()
{
  rclcpp::spin(shared_from_this());
}

bool ConveyorSpawner::loadSpawnParameters()
{
  try
  {
    this->declare_parameter<std::string>("reference_frame", "world");
    this->declare_parameter<double>("spawn_period", 1.0);
    this->declare_parameter<int>("randomization_seed", 42);
    this->declare_parameter<std::vector<std::string>>("objects", {});

    this->get_parameter("reference_frame", params_.reference_frame);
    this->get_parameter("spawn_period", params_.spawn_period);
    this->get_parameter("randomization_seed", params_.randomization_seed);
    
    auto object_names = this->get_parameter("objects").as_string_array();
    for (const auto &name : object_names)
    {
      ObjectParameters object_params;
      std::string param_prefix = "objects." + name;
      
      this->declare_parameter<std::string>(param_prefix + ".name", "");
      this->declare_parameter<std::string>(param_prefix + ".mesh_resource", "");
      this->declare_parameter<std::vector<double>>(param_prefix + ".initial_pose.position", {0.0, 0.0, 0.0});
      this->declare_parameter<std::vector<double>>(param_prefix + ".initial_pose.orientation", {0.0, 0.0, 0.0});
      this->declare_parameter<double>(param_prefix + ".lateral_placement_variance", 0.0);
      this->declare_parameter<double>(param_prefix + ".yaw_placement_variance", 0.0);
      this->declare_parameter<double>(param_prefix + ".spawn_timing_variance", 0.0);

      this->get_parameter(param_prefix + ".name", object_params.name);
      this->get_parameter(param_prefix + ".mesh_resource", object_params.mesh_resource);

      std::vector<double> position, orientation;
      this->get_parameter(param_prefix + ".initial_pose.position", position);
      this->get_parameter(param_prefix + ".initial_pose.orientation", orientation);

      object_params.initial_pose.position.x = position[0];
      object_params.initial_pose.position.y = position[1];
      object_params.initial_pose.position.z = position[2];

      tf2::Quaternion q;
      q.setRPY(orientation[0], orientation[1], orientation[2]);
      object_params.initial_pose.orientation = tf2::toMsg(q);

      this->get_parameter(param_prefix + ".lateral_placement_variance", object_params.lateral_placement_variance);
      this->get_parameter(param_prefix + ".yaw_placement_variance", object_params.yaw_placement_variance);
      this->get_parameter(param_prefix + ".spawn_timing_variance", object_params.spawn_timing_variance);

      params_.objects.push_back(object_params);
    }
  }
  catch (const rclcpp::ParameterTypeException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", ex.what());
    return false;
  }

  return true;
}

bool ConveyorSpawner::loadObjectParameters(const rclcpp::Parameter &param, ObjectParameters &object_params)
{
  // Assuming loadObjectParameters is used for custom parameters
  // Custom implementation needed here if different structure
  return true;
}

bool ConveyorSpawner::connectToROS()
{
  const auto SRV_TIMEOUT = std::chrono::seconds(10);
  spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("gazebo/spawn_entity");
  if (!spawn_client_->wait_for_service(SRV_TIMEOUT))
  {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for spawn service");
    return false;
  }

  return true;
}

void ConveyorSpawner::start(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Starting conveyor spawner...");
  timer_->reset();
}

void ConveyorSpawner::stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Stopping conveyor spawner...");
  timer_->cancel();
}

void ConveyorSpawner::spawnObject()
{
  ++object_counter_;

  std::uniform_int_distribution<int> dist(0, params_.objects.size() - 1);
  int idx = dist(random_engine_);
  auto obj = params_.objects[idx];

  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = "object_" + std::to_string(object_counter_);
  request->xml = createObjectURDF(obj.name, obj.mesh_resource); // Ensure createObjectURDF is defined elsewhere
  request->robot_namespace = "hrwros";
  request->initial_pose = obj.initial_pose;

  std::uniform_real_distribution<double> dist_lpv(-obj.lateral_placement_variance, obj.lateral_placement_variance);
  request->initial_pose.position.y += dist_lpv(random_engine_);

  std::uniform_real_distribution<double> dist_ypv(-obj.yaw_placement_variance, obj.yaw_placement_variance);
  tf2::Quaternion q;
  tf2::fromMsg(request->initial_pose.orientation, q);
  tf2::Quaternion dq;
  dq.setRPY(0.0, 0.0, dist_ypv(random_engine_));
  q *= dq;
  request->initial_pose.orientation = tf2::toMsg(q);

  std::uniform_real_distribution<double> dist_tv(0, obj.spawn_timing_variance);
  auto time_var = std::chrono::duration<double>(dist_tv(random_engine_));
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(time_var));

  auto result = spawn_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(shared_from_this(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call spawn service");
    --object_counter_;
    return;
  }

  if (!result.get()->success)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", result.get()->status_message.c_str());
    --object_counter_;
    return;
  }

  std_msgs::msg::Header msg;
  msg.frame_id = obj.name;
  msg.stamp = this->now();
  pub_->publish(msg);
}

} // namespace simulation
} // namespace hrwros
