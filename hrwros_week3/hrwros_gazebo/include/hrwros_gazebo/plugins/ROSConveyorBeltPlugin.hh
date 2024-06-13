
#ifndef _ROS_CONVEYOR_BELT_PLUGIN_HH_
#define _ROS_CONVEYOR_BELT_PLUGIN_HH_

#include <sdf/sdf.hh>

// Gazebo
#include "ConveyorBeltPlugin.hh"

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include "hrwros_gazebo/srv/conveyor_belt_control.hpp"

namespace gazebo
{
  /// \brief ROS 2 implementation of the ConveyorBeltPlugin plugin
  class ROSConveyorBeltPlugin : public ConveyorBeltPlugin
  {
    /// \brief Constructor
    public: ROSConveyorBeltPlugin();

    /// \brief Destructor
    public: virtual ~ROSConveyorBeltPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Receives requests on the conveyor belt's topic.
    /// \param[in] request The desired state of the conveyor belt.
    /// \param[in] response If the service succeeded or not.
    public: bool OnControlCommand(const std::shared_ptr<hrwros_gazebo::srv::ConveyorBeltControl::Request> request,
                                  std::shared_ptr<hrwros_gazebo::srv::ConveyorBeltControl::Response> response);

    /// \brief for setting ROS name space
    private: std::string robotNamespace_;

    /// \brief ROS node handle
    private: std::shared_ptr<rclcpp::Node> rosnode_;

    /// \brief Receives service calls to control the conveyor belt.
    public: rclcpp::Service<hrwros_gazebo::srv::ConveyorBeltControl>::SharedPtr controlService_;
  };
}
#endif