#include "hrwros_gazebo/plugins/ROSLogicalCameraPlugin.hh"

#include "hrwros_gazebo/plugins/ARIAC.hh"
#include "hrwros_gazebo_interface/msg/logical_camera_image.hpp"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/LogicalCameraSensor.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <sstream>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSLogicalCameraPlugin);

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::ROSLogicalCameraPlugin()
{
}

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::~ROSLogicalCameraPlugin()
{
  if (this->rosnode_)
  {
    rclcpp::shutdown();
  }
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Load parameters
  this->robotNamespace_ = "/";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
  }

  this->world_ = _parent->GetWorld();
  this->name_ = _parent->GetName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "A ROS node for Gazebo has not been initialized, "
        "unable to load plugin. Load the Gazebo system plugin "
        "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Started Logical Camera plugin with name: %s", this->name_.c_str());

  this->onlyPublishKnownModels_ = false;
  if (_sdf->HasElement("known_model_types"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Only publishing known model types");
    this->onlyPublishKnownModels_ = true;
    this->knownModelTypes_.clear();
    sdf::ElementPtr knownModelTypesElem = _sdf->GetElement("known_model_types");
    if (!knownModelTypesElem->HasElement("type"))
    {
      gzerr << "Unable to find <type> elements in the <known_model_types> section\n";
      return;
    }
    sdf::ElementPtr knownModelTypeElem = knownModelTypesElem->GetElement("type");
    while (knownModelTypeElem)
    {
      // Parse the model type, which is encoded in model names.
      std::string type = knownModelTypeElem->Get<std::string>();

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New known model type: %s", type.c_str());
      this->knownModelTypes_.push_back(type);
      knownModelTypeElem = knownModelTypeElem->GetNextElement("type");
    }
  }

  if (_sdf->HasElement("known_model_names"))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Only publishing known model names");
    this->onlyPublishKnownModels_ = true;
    this->knownModelNames_.clear();
    sdf::ElementPtr knownModelNamesElem = _sdf->GetElement("known_model_names");
    if (knownModelNamesElem->HasElement("name"))
    {
      sdf::ElementPtr knownModelNameElem = knownModelNamesElem->GetElement("name");
      while (knownModelNameElem)
      {
        std::string knownModelName = knownModelNameElem->Get<std::string>();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New known model name: %s", knownModelName.c_str());
        this->knownModelNames_.push_back(knownModelName);
        knownModelNameElem = knownModelNameElem->GetNextElement("name");
      }
    }
  }

  this->anonymizeModels_ = false;
  if (_sdf->HasElement("anonymize_models"))
  {
    this->anonymizeModels_ = _sdf->Get<bool>("anonymize_models");
  }
  if (this->anonymizeModels_)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Anonymizing model types");
  }
  else
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Not anonymizing model types");
  }

  this->modelFramePrefix_ = this->name_ + "_";
  if (_sdf->HasElement("model_frame_prefix"))
  {
    this->modelFramePrefix_ = _sdf->Get<std::string>("model_frame_prefix");
  }
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Using model frame prefix of: %s", this->modelFramePrefix_.c_str());

  this->model_ = _parent;
  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(this->model_->GetWorld()->Name());
  this->rosnode_ = rclcpp::Node::make_shared(this->robotNamespace_);

  this->FindLogicalCamera();
  if (!this->sensor_)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }
  ignition::math::Vector3d kitTrayPosition = ignition::math::Vector3d(0, 0.15, 0.75);
  ignition::math::Quaterniond kitTrayOrientation = ignition::math::Quaterniond(1, 0, 0, 0);
  this->kitTrayToAgv_ = ignition::math::Pose3d(kitTrayPosition, kitTrayOrientation);

  // Handle noise model settings.
  if (_sdf->HasElement("position_noise"))
  {
    this->noiseModels_["POSITION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("position_noise")->GetElement("noise"),
      "logical_camera");
  }
  if (_sdf->HasElement("orientation_noise"))
  {
    this->noiseModels_["ORIENTATION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("orientation_noise")->GetElement("noise"),
      "logical_camera");
  }

  std::string imageTopicRos = this->name_;
  if (_sdf->HasElement("image_topic_ros")) {
    imageTopicRos = _sdf->Get<std::string>("image_topic_ros");
  }

  this->imageSub_ = this->node_->Subscribe(this->sensor_->Topic(),
        &ROSLogicalCameraPlugin::OnImage, this);
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Subscribing to gazebo topic: %s", this->sensor_->Topic().c_str());

  this->imagePub_ = this->rosnode_->create_publisher<hrwros_gazebo::msg::LogicalCameraImage>(imageTopicRos, 1);
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Publishing to ROS topic: %s", imagePub_->get_topic_name());

  if (_sdf->HasElement("activation_topic"))
  {
    std::string activationTopic = _sdf->Get<std::string>("activation_topic");
    this->activationSub_ = this->node_->Subscribe(activationTopic,
            &ROSLogicalCameraPlugin::OnActivationMsg, this);
  }

  transformBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->rosnode_);
}

void ROSLogicalCameraPlugin::FindLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  // Go through each link's sensors until a logical camera is found
  for (physics::LinkPtr link : this->model_->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor->Type() == "logical_camera")
      {
        this->sensor_ = sensor;
        break;
      }
    }
    if (this->sensor_)
    {
      this->cameraLink_ = link;
      break;
    }
  }
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnImage(ConstLogicalCameraImagePtr &_msg)
{
  if (!this->publishing_)
  {
    return;
  }
  hrwros_gazebo::msg::LogicalCameraImage imageMsg;
  ignition::math::Vector3d cameraPosition = ignition::math::Vector3d(msgs::ConvertIgn(_msg->pose().position()));
  ignition::math::Quaterniond cameraOrientation = ignition::math::Quaterniond(
    msgs::ConvertIgn(_msg->pose().orientation()));
  ignition::math::Pose3d cameraPose = ignition::math::Pose3d(cameraPosition, cameraOrientation);
  this->PublishTF(cameraPose, "world", this->name_ + "_frame");

  imageMsg.pose.position.x = cameraPosition.X();
  imageMsg.pose.position.y = cameraPosition.Y();
  imageMsg.pose.position.z = cameraPosition.Z();
  imageMsg.pose.orientation.x = cameraOrientation.X();
  imageMsg.pose.orientation.y = cameraOrientation.Y();
  imageMsg.pose.orientation.z = cameraOrientation.Z();
  imageMsg.pose.orientation.w = cameraOrientation.W();

  std::ostringstream logStream;
  ignition::math::Pose3d modelPose;
  for (int i = 0; i < _msg->model_size(); ++i)
  {
    std::string modelName = _msg->model(i).name();
    std::string modelType = ariac::DetermineModelType(modelName);

    if (!this->ModelToPublish(modelName, modelType))
    {
      logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
    }
    else
    {
      logStream << "Publishing model: " << modelName << " of type: " << modelType << std::endl;
      ignition::math::Vector3d modelPosition = ignition::math::Vector3d(
        msgs::ConvertIgn(_msg->model(i).pose().position()));
      ignition::math::Quaterniond modelOrientation = ignition::math::Quaterniond(
        msgs::ConvertIgn(_msg->model(i).pose().orientation()));
      modelPose = ignition::math::Pose3d(modelPosition, modelOrientation);

      std::string modelNameToUse;
      std::string modelTypeToUse;
      if (this->anonymizeModels_)
      {
        modelNameToUse = "model_" + ariac::DetermineModelId(modelName);
        modelTypeToUse = "model";
      }
      else
      {
        modelNameToUse = ariac::TrimNamespace(modelName);
        modelTypeToUse = modelType;
      }
      std::string modelFrameId = this->modelFramePrefix_ + modelNameToUse + "_frame";

      bool isAgv = modelType == "agv1" || modelType == "agv2";
      if (isAgv)
      {
        // If AGVs are detected, also publish the pose to the respective kit tray.
        // Add noise to the kit tray pose, not the AGV base (it is too much noise by the time the tray pose is extrapolated)
        auto noisyKitTrayPose = ignition::math::Pose3d(this->kitTrayToAgv_);
        this->AddNoise(noisyKitTrayPose);
        if (modelType == "agv1")
        {
          this->PublishTF(noisyKitTrayPose, modelFrameId, this->modelFramePrefix_ + "kit_tray_1_frame");
        }
        else if (modelType == "agv2")
        {
          this->PublishTF(noisyKitTrayPose, modelFrameId, this->modelFramePrefix_ + "kit_tray_2_frame");
        }
      }
      else
      {
        this->AddNoise(modelPose);
      }
      this->AddModelToMsg(modelTypeToUse, modelPose, imageMsg);
      this->PublishTF(modelPose, this->name_ + "_frame", modelFrameId);

    }

    // Check any children models
    auto modelPtr = this->world_->ModelByName(modelName);
    auto nestedModels = modelPtr->NestedModels();
    for (auto nestedModel : nestedModels)
    {
      modelName = nestedModel->GetName();
      modelType = ariac::DetermineModelType(modelName);
      if (!this->ModelToPublish(modelName, modelType))
      {
        logStream << "Not publishing model: " << modelName << " of type: " << modelType << std::endl;
        continue;
      }
      logStream << "Publishing model: " << modelName << " of type: " << modelType  << std::endl;
      // Convert the world pose of the model into the camera frame
      modelPose = (nestedModel->WorldPose()) - cameraPose;
      this->AddNoise(modelPose);
      this->AddModelToMsg(modelType, modelPose, imageMsg);
      // Do not publish TF information for nested models (kit_tray) because it's not accurate.
      // See https://bitbucket.org/osrf/ariac/issues/54.
      // this->PublishTF(modelPose, this->name_ + "_frame", this->modelFramePrefix_ + ariac::TrimNamespace(modelName) + "_frame");
    }
  }

  if (!logStream.str().empty())
  {
    RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("rclcpp"), 1000, "%s", logStream.str().c_str());
  }
  this->imagePub_->publish(imageMsg);
}

bool ROSLogicalCameraPlugin::ModelToPublish(
  const std::string & modelName, const std::string & modelType)
{
  bool publishModel = true;

  // Check if there are restrictions on which models to publish
  if (this->onlyPublishKnownModels_)
  {
    // Only publish the model if its type is known
    auto it = std::find(this->knownModelTypes_.begin(), this->knownModelTypes_.end(), modelType);
    bool knownModel = it != this->knownModelTypes_.end();
    it = std::find(this->knownModelNames_.begin(), this->knownModelNames_.end(), ariac::TrimNamespace(modelName));
    knownModel |= it != this->knownModelNames_.end();
    publishModel = knownModel;
  }
  return publishModel;
}

void ROSLogicalCameraPlugin::AddNoise(ignition::math::Pose3d & pose)
{
  if (this->noiseModels_.find("POSITION_NOISE") != this->noiseModels_.end())
  {
    // Apply additive noise to the model position
    pose.Pos().X() =
      this->noiseModels_["POSITION_NOISE"]->Apply(pose.Pos().X());
    pose.Pos().Y() =
      this->noiseModels_["POSITION_NOISE"]->Apply(pose.Pos().Y());
    pose.Pos().Z() =
      this->noiseModels_["POSITION_NOISE"]->Apply(pose.Pos().Z());
  }

  if (this->noiseModels_.find("ORIENTATION_NOISE") != this->noiseModels_.end())
  {
    // Create a perturbation quaternion and apply it to the model orientation
    double r = this->noiseModels_["ORIENTATION_NOISE"]->Apply(0.0);
    double p = this->noiseModels_["ORIENTATION_NOISE"]->Apply(0.0);
    double y = this->noiseModels_["ORIENTATION_NOISE"]->Apply(0.0);
    ignition::math::Quaterniond pert = ignition::math::Quaterniond(r, p, y);
    pose.Rot() *= pert;
  }
}

void ROSLogicalCameraPlugin::AddModelToMsg(
  const std::string & modelType, const ignition::math::Pose3d & modelPose,
  hrwros_gazebo::msg::LogicalCameraImage & imageMsg)
{
  hrwros_gazebo::msg::Model modelMsg;
  modelMsg.pose.position.x = modelPose.Pos().X();
  modelMsg.pose.position.y = modelPose.Pos().Y();
  modelMsg.pose.position.z = modelPose.Pos().Z();

  modelMsg.pose.orientation.y = modelPose.Rot().X();
  modelMsg.pose.orientation.x = modelPose.Rot().Y();
  modelMsg.pose.orientation.z = modelPose.Rot().Z();
  modelMsg.pose.orientation.w = modelPose.Rot().W();
  modelMsg.type = modelType;
  imageMsg.models.push_back(modelMsg);
}

void ROSLogicalCameraPlugin::PublishTF(
  const ignition::math::Pose3d & pose, const std::string & parentFrame, const std::string & frame)
{
  rclcpp::Time currentTime = this->rosnode_->now();

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = currentTime;
  transformStamped.header.frame_id = parentFrame;
  transformStamped.child_frame_id = frame;

  transformStamped.transform.translation.x = pose.Pos().X();
  transformStamped.transform.translation.y = pose.Pos().Y();
  transformStamped.transform.translation.z = pose.Pos().Z();
  transformStamped.transform.rotation.x = pose.Rot().X();
  transformStamped.transform.rotation.y = pose.Rot().Y();
  transformStamped.transform.rotation.z = pose.Rot().Z();
  transformStamped.transform.rotation.w = pose.Rot().W();

  transformBroadcaster_->sendTransform(transformStamped);
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnActivationMsg(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "activate")
  {
    this->publishing_ = true;
  }
  else if (_msg->data() == "deactivate")
  {
    this->publishing_ = false;
  }
  else
  {
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
  }
}
