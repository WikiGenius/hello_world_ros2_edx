#include "hrwros_gazebo/plugins/ROSConveyorBeltPlugin.hh"

#include <cstdlib>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin);

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
  if (this->rosnode_)
  {
    rclcpp::shutdown();
  }
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    this->robotNamespace_ = _sdf->Get<std::string>("robot_namespace") + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "A ROS node for Gazebo has not been initialized, "
        "unable to load plugin. Load the Gazebo system plugin "
        "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string topic = "conveyor/control";
  if (_sdf->HasElement("topic"))
    topic = _sdf->Get<std::string>("topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

  this->rosnode_ = rclcpp::Node::make_shared(this->robotNamespace_);

  this->controlService_ = this->rosnode_->create_service<hrwros_gazebo_interface::srv::ConveyorBeltControl>(
    topic, std::bind(&ROSConveyorBeltPlugin::OnControlCommand, this, std::placeholders::_1, std::placeholders::_2));
}

/////////////////////////////////////////////////
bool ROSConveyorBeltPlugin::OnControlCommand(
  const std::shared_ptr<hrwros_gazebo_interface::srv::ConveyorBeltControl::Request> req,
  std::shared_ptr<hrwros_gazebo_interface::srv::ConveyorBeltControl::Response> res)
{
  RCLCPP_INFO(this->rosnode_->get_logger(), "Conveyor control service called with: %f", req->state.power);

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    RCLCPP_ERROR(this->rosnode_->get_logger(), "%s", errStr.c_str());
    res->success = false;
    return true;
  }

  try
  {
    this->SetPower(req->state.power);
    res->success = true;
  }
  catch (const std::domain_error& e )
  {
    res->success = false;
  }

  return true;
}
