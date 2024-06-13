#ifndef _ROS2_PROXIMITY_RAY_PLUGIN_HH_
#define _ROS2_PROXIMITY_RAY_PLUGIN_HH_

// Gazebo
#include "hrwros_gazebo/plugins/ProximityRayPlugin.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <hrwros_gazebo/msg/proximity.hpp>

namespace gazebo
{
  /// \brief ROS 2 interface for the ProximityRayPlugin plugin
  class ROSProximityRayPlugin : public ProximityRayPlugin
  {
    /// \brief Constructor
    public: ROSProximityRayPlugin();

    /// \brief Destructor
    public: virtual ~ROSProximityRayPlugin();

    /// \brief Load the plugin
    /// \param _parent The parent entity must be a Ray sensor
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief The connection tied to ROSProximityRayPlugin::OnNewLaserScans()
    protected: event::ConnectionPtr newLaserScansConnection;

    /// \brief A shared pointer to the ROS 2 node
    protected: rclcpp::Node::SharedPtr rosnode;

    /// \brief ROS 2 publisher for the sensor state
    protected: rclcpp::Publisher<hrwros_gazebo::msg::Proximity>::SharedPtr statePub;

    /// \brief ROS 2 publisher for the sensor state changes
    protected: rclcpp::Publisher<hrwros_gazebo::msg::Proximity>::SharedPtr stateChangePub;

    /// \brief ROS 2 message for the sensor state
    protected: hrwros_gazebo::msg::Proximity state_msg;

    /// \brief for setting ROS 2 namespace
    protected: std::string robotNamespace;

    /// \brief for setting ROS 2 frame id
    protected: std::string frameId;

  };
}

#endif
