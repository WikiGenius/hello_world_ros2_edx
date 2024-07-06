#ifndef GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H
#define GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include "hrwros_gazebo/plugins/ROSVacuumGripperPlugin.hh"
#include "hrwros_gazebo_interface/srv/vacuum_gripper_control.hpp"
#include "hrwros_gazebo_interface/msg/vacuum_gripper_state.hpp"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSVacuumGripperPlugin class.
  struct ROSVacuumGripperPluginPrivate
  {
    /// \brief ROS node handle.
    public: std::shared_ptr<rclcpp::Node> rosnode;

    /// \brief Publishes the state of the gripper.
    public: rclcpp::Publisher<hrwros_gazebo::msg::VacuumGripperState>::SharedPtr statePub;

    /// \brief Receives service calls to control the gripper.
    public: rclcpp::Service<hrwros_gazebo::srv::VacuumGripperControl>::SharedPtr controlService;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSVacuumGripperPlugin);

/////////////////////////////////////////////////
ROSVacuumGripperPlugin::ROSVacuumGripperPlugin()
  : VacuumGripperPlugin(),
    dataPtr(new ROSVacuumGripperPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSVacuumGripperPlugin::~ROSVacuumGripperPlugin()
{
  rclcpp::shutdown();
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(rclcpp::get_logger("ROSVacuumGripperPlugin"), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
    return;
  }

  // Load SDF parameters.
  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
  }

  std::string controlTopic = "gripper/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "gripper/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  VacuumGripperPlugin::Load(_parent, _sdf);

  this->dataPtr->rosnode = std::make_shared<rclcpp::Node>("vacuum_gripper_plugin", robotNamespace);

  // Service for controlling the gripper.
  this->dataPtr->controlService = this->dataPtr->rosnode->create_service<hrwros_gazebo::srv::VacuumGripperControl>(
    controlTopic, std::bind(&ROSVacuumGripperPlugin::OnGripperControl, this, std::placeholders::_1, std::placeholders::_2));

  // Message used for publishing the state of the gripper.
  this->dataPtr->statePub = this->dataPtr->rosnode->create_publisher<hrwros_gazebo::msg::VacuumGripperState>(stateTopic, 10);
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Reset()
{
  VacuumGripperPlugin::Reset();
}

bool ROSVacuumGripperPlugin::OnGripperControl(
  const std::shared_ptr<hrwros_gazebo::srv::VacuumGripperControl::Request> request,
  std::shared_ptr<hrwros_gazebo::srv::VacuumGripperControl::Response> response)
{
  RCLCPP_DEBUG(this->dataPtr->rosnode->get_logger(), "Gripper control requested: %s", (request->enable ? "Enable" : "Disable"));
  if (request->enable)
    this->Enable();
  else
    this->Disable();

  response->success = true;
  return response->success;
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Publish() const
{
  auto msg = hrwros_gazebo::msg::VacuumGripperState();
  msg.attached = this->Attached();
  msg.enabled = this->Enabled();
  this->dataPtr->statePub->publish(msg);
}

#endif // GILBRETH_SUPPORT_CONVEYOR_SPAWNER_H
