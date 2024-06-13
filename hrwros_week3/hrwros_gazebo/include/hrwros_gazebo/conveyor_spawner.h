#ifndef GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H
#define GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <urdf/model.h>
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


  bool init(const rclcpp::Parameter& p);


  void run();


private:

  bool loadSpawnParameters(const rclcpp::Parameter& p,
                           SpawnParameters& spawn_params) const;


  bool loadObjectParameters(const rclcpp::Parameter& object,
                            ObjectParameters& object_params) const;

  bool connectToROS();


  void start(const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<std_srvs::srv::Empty::Request> request,
             std::shared_ptr<std_srvs::srv::Empty::Response> response);


  void stop(const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response);


  void spawnObject();


  int object_counter_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr spawn_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  SpawnParameters params_;

};

} // namespace simulation
} // namespace hrwros

#endif // GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H
