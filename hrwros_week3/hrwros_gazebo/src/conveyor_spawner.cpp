/*
BSD 3-Clause License

...

Authors: the HRWROS mooc instructors
*/

#include <boost/filesystem.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include "hrwros_gazebo/conveyor_spawner.h"
#include "hrwros_gazebo/urdf_creator.h"
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <urdf/model.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const static std::string GAZEBO_SPAWN_SERVICE = "gazebo/spawn_entity";
const static std::string START_SPAWN_SERVICE = "start_spawn";
const static std::string STOP_SPAWN_SERVICE = "stop_spawn";
const static std::string SPAWNED_PART_TOPIC = "spawned_part";
const static double SRV_TIMEOUT = 10.0f;

namespace hrwros
{
namespace simulation
{

ConveyorSpawner::ConveyorSpawner()
  : Node("conveyor_spawner")
{
}

bool ConveyorSpawner::init(const rclcpp::Parameter& p)
{
  // Load the parameters
  if(!loadSpawnParameters(p, params_))
  {
    return false;
  }

  // Connect to ROS topics/services/actions/etc.
  if(!connectToROS())
  {
    return false;
  }

  // Initialize the randomization engine
  srand(params_.randomization_seed);

  start_server_ = this->create_service<std_srvs::srv::Empty>(START_SPAWN_SERVICE, 
                  std::bind(&ConveyorSpawner::start, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  stop_server_ = this->create_service<std_srvs::srv::Empty>(STOP_SPAWN_SERVICE, 
                 std::bind(&ConveyorSpawner::stop, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  timer_ = this->create_wall_timer(std::chrono::duration<double>(params_.spawn_period), 
           std::bind(&ConveyorSpawner::spawnObject, this));
  pub_ = this->create_publisher<std_msgs::msg::Header>(SPAWNED_PART_TOPIC, 10);

  return true;
}

void ConveyorSpawner::run()
{
  rclcpp::spin(shared_from_this());
}

bool ConveyorSpawner::loadSpawnParameters(const rclcpp::Parameter& p,
                                          SpawnParameters& spawn_params) const
{
  try
  {
    // Get the top-level parameters
    // Get the reference frame
    spawn_params.reference_frame = p.get<std::string>("reference_frame");

    // Get the spawn timing
    spawn_params.spawn_period = p.get<double>("spawn_period");

    // Get the randomization seed
    spawn_params.randomization_seed = p.get<int>("randomization_seed");

    // Get the spawned objects
    auto objects = p.get<std::vector<rclcpp::Parameter>>("objects");
    for(const auto& obj_param : objects)
    {
      ObjectParameters object_params;
      if(!loadObjectParameters(obj_param, object_params))
      {
        return false;
      }

      // Add the object's parameters to the list
      spawn_params.objects.push_back(object_params);
    }
  }
  catch(const rclcpp::exceptions::ParameterNotDeclaredException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception in loading spawner parameters:\n%s'", ex.what());
    return false;
  }

  return true;
}

bool ConveyorSpawner::loadObjectParameters(const rclcpp::Parameter& object,
                                           ObjectParameters& object_params) const
{
  try
  {
    // Get the name
    object_params.name = object.get<std::string>("name");

    // Get the relative URDF path file
    object_params.mesh_resource = object.get<std::string>("mesh_resource");

    // Get the initial pose
    auto initial_pose = object.get<rclcpp::Parameter>("initial_pose");

    auto position = initial_pose.get<std::vector<double>>("position");
    object_params.initial_pose.position.x = position[0];
    object_params.initial_pose.position.y = position[1];
    object_params.initial_pose.position.z = position[2];

    auto orientation = initial_pose.get<std::vector<double>>("orientation");
    tf2::Quaternion q;
    q.setRPY(orientation[0], orientation[1], orientation[2]);
    object_params.initial_pose.orientation = tf2::toMsg(q);

    // Get the lateral placement variance
    object_params.lateral_placement_variance = object.get<double>("lateral_placement_variance");

    // Get the yaw placement_variance
    object_params.yaw_placement_variance = object.get<double>("yaw_placement_variance") * M_PI / 180.0f;

    // Get the spawn timing variance
    object_params.spawn_timing_variance = object.get<double>("spawn_timing_variance");
  }
  catch(const rclcpp::exceptions::ParameterNotDeclaredException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception in loading object parameters:\n%s", ex.what());
    return false;
  }

  return true;
}

bool ConveyorSpawner::connectToROS()
{
  spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>(GAZEBO_SPAWN_SERVICE);
  if(!spawn_client_->wait_for_service(std::chrono::duration<double>(SRV_TIMEOUT)))
  {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for '%s' service", spawn_client_->get_service_name());
    return false;
  }

  return true;
}

void ConveyorSpawner::start(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Starting conveyor spawner...");
  timer_->reset();
}

void ConveyorSpawner::stop(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Stopping conveyor spawner...");
  timer_->cancel();
}

void ConveyorSpawner::spawnObject()
{
  ++object_counter_;

  // Randomize the model to be spawned
  int idx = rand() % params_.objects.size();
  auto obj = params_.objects.begin();
  std::advance(obj, idx);

  // Populate the spawn service request
  auto srv = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  srv->reference_frame = params_.reference_frame;
  srv->robot_namespace = "hrwros";
  srv->entity_name = "object_" + std::to_string(object_counter_);
  srv->initial_pose = obj->initial_pose;
  srv->xml = createObjectURDF(obj->name, obj->mesh_resource);

  // Randomize the object's lateral spawn position
  double r_lpv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double& lpv = obj->lateral_placement_variance;
  double lpv_delta = -lpv + 2.0*r_lpv*lpv;
  srv->initial_pose.position.y += lpv_delta;

  // Randomize the object's spawn yaw angle
  double r_ypv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  double& ypv = obj->yaw_placement_variance;
  double ypv_delta = -ypv + 2.0*r_ypv*ypv;
  tf2::Quaternion q;
  tf2::fromMsg(srv->initial_pose.orientation, q);
  tf2::Quaternion dq;
  dq.setRPY(0.0, 0.0, ypv_delta);
  q *= dq;
  srv->initial_pose.orientation = tf2::toMsg(q);

  // Randomize the delay in spawning the object
  double r_tv = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

  rclcpp::sleep_for(std::chrono::duration<double>(r_tv * obj->spawn_timing_variance));

  // Call the spawn service
  auto result = spawn_client_->async_send_request(srv);
  if(rclcpp::spin_until_future_complete(this->shared_from_this(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call '%s' service", spawn_client_->get_service_name());
    --object_counter_;
    return;
  }

  auto response = result.get();
  if(!response->success)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", response->status_message.c_str());
    --object_counter_;
    return;
  }
  else
  {
    // Publish which part was just spawned onto the conveyor
    std_msgs::msg::Header msg;
    msg.frame_id = obj->name;
    msg.stamp = this->now();
    msg.seq = object_counter_;
    pub_->publish(msg);
  }
}

} // namespace simulation
} // namespace hrwros
