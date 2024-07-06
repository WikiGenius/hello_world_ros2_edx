#include "hrwros_gazebo/plugins/ROSProximityRayPlugin.hh"
#include <string>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "hrwros_gazebo_interface/msg/proximity.hpp"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ROSProximityRayPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
ROSProximityRayPlugin::ROSProximityRayPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ROSProximityRayPlugin::~ROSProximityRayPlugin()
{
  rclcpp::shutdown();
}

void ROSProximityRayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(rclcpp::get_logger("ROSProximityRayPlugin"), 
                 "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                 "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ProximityRayPlugin::Load(_parent, _sdf);

  // Read ROS-specific sdf tags
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Over-ride topics from the ProximityRayPlugin which may contain ~s
  this->stateTopic = _parent->Name();
  if (_sdf->HasElement("output_state_topic"))
  {
    this->stateTopic = _sdf->Get<std::string>("output_state_topic");
  }

  this->stateChangeTopic = _parent->Name() + "_change";
  if (_sdf->HasElement("output_change_topic"))
  {
    this->stateChangeTopic = _sdf->Get<std::string>("output_change_topic");
  }

  this->frameId = _parent->Name() + "_frame";
  if (_sdf->HasElement("frame_id"))
  {
    this->frameId = _sdf->Get<std::string>("frame_id");
  }
  this->state_msg.header.frame_id = this->frameId;

  this->rosnode = gazebo_ros::Node::Get(_sdf);

  // Initialize the publishers
  this->statePub = this->rosnode->create_publisher<hrwros_gazebo::msg::Proximity>(
    this->stateTopic, 10);
  this->stateChangePub = this->rosnode->create_publisher<hrwros_gazebo::msg::Proximity>(
    this->stateChangeTopic, 10);

  // Callback for laser scans
  this->newLaserScansConnection =
      this->parentSensor->LaserShape()->ConnectNewLaserScans(
          std::bind(&ROSProximityRayPlugin::OnNewLaserScans, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void ROSProximityRayPlugin::OnNewLaserScans()
{
  auto now = this->world->SimTime();
  bool stateChanged = this->ProcessScan();
  this->state_msg.header.stamp.sec = now.sec;
  this->state_msg.header.stamp.nanosec = now.nsec;
  this->state_msg.object_detected = this->objectDetected;
  this->state_msg.min_range = this->sensingRangeMin;
  this->state_msg.max_range = this->sensingRangeMax;
  this->statePub->publish(this->state_msg);
  if (stateChanged)
  {
    RCLCPP_INFO(rclcpp::get_logger("ROSProximityRayPlugin"), 
                "%s: change in sensor state", this->parentSensor->Name().c_str());
    this->stateChangePub->publish(this->state_msg);
  }
}
