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

#include <algorithm>
#include <sstream>
#include <string>
#include <memory>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSLogicalCameraPlugin);

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::ROSLogicalCameraPlugin()
{
}

/////////////////////////////////////////////////
ROSLogicalCameraPlugin::~ROSLogicalCameraPlugin()
{
  rclcpp::shutdown();
}
void ROSLogicalCameraPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace = "/";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
  }

  this->world = _parent->GetWorld();
  this->name = _parent->GetName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), 
                 "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                 "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  gzdbg << "Started Logical Camera plugin with name: " << this->name << "\n";

  this->onlyPublishKnownModels = false;
  if (_sdf->HasElement("known_model_types"))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Only publishing known model types");
    this->onlyPublishKnownModels = true;
    this->knownModelTypes.clear();
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
      this->knownModelTypes.push_back(type);
      knownModelTypeElem = knownModelTypeElem->GetNextElement("type");
    }
  }

  if (_sdf->HasElement("known_model_names"))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Only publishing known model names");
    this->onlyPublishKnownModels = true;
    this->knownModelNames.clear();
    sdf::ElementPtr knownModelNamesElem = _sdf->GetElement("known_model_names");
    if (knownModelNamesElem->HasElement("name"))
    {
      sdf::ElementPtr knownModelNameElem = knownModelNamesElem->GetElement("name");
      while (knownModelNameElem)
      {
        std::string knownModelName = knownModelNameElem->Get<std::string>();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "New known model name: %s", knownModelName.c_str());
        this->knownModelNames.push_back(knownModelName);
        knownModelNameElem = knownModelNameElem->GetNextElement("name");
      }
    }
  }

  this->anonymizeModels = false;
  if (_sdf->HasElement("anonymize_models"))
  {
    this->anonymizeModels = _sdf->Get<bool>("anonymize_models");
  }
  if (this->anonymizeModels)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Anonymizing model types");
  }
  else
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Not anonymizing model types");
  }

  this->modelFramePrefix = this->name + "_";
  if (_sdf->HasElement("model_frame_prefix"))
  {
    this->modelFramePrefix = _sdf->Get<std::string>("model_frame_prefix");
  }
  gzdbg << "Using model frame prefix of: " << this->modelFramePrefix << std::endl;

  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());
  this->rosnode = std::make_shared<rclcpp::Node>(this->robotNamespace);

  this->FindLogicalCamera();
  if (!this->sensor)
  {
    gzerr << "No logical camera found on any link\n";
    return;
  }
  ignition::math::Vector3d kitTrayPosition = ignition::math::Vector3d(0, 0.15, 0.75);
  ignition::math::Quaterniond kitTrayOrientation = ignition::math::Quaterniond(1, 0, 0, 0);
  this->kitTrayToAgv = ignition::math::Pose3d(kitTrayPosition, kitTrayOrientation);

  // Handle noise model settings.
  if (_sdf->HasElement("position_noise"))
  {
    this->noiseModels["POSITION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("position_noise")->GetElement("noise"),
                                           "logical_camera");
  }
  if (_sdf->HasElement("orientation_noise"))
  {
    this->noiseModels["ORIENTATION_NOISE"] =
      sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("orientation_noise")->GetElement("noise"),
                                           "logical_camera");
  }

  std::string imageTopic_ros = this->name;
  if (_sdf->HasElement("image_topic_ros"))
  {
    imageTopic_ros = _sdf->Get<std::string>("image_topic_ros");
  }

  this->imageSub = this->node->Subscribe(this->sensor->Topic(),
                                         &ROSLogicalCameraPlugin::OnImage, this);
  gzdbg << "Subscribing to gazebo topic: " << this->sensor->Topic() << "\n";

  this->imagePub = this->rosnode->create_publisher<hrwros_gazebo_interface::msg::LogicalCameraImage>(
                     imageTopic_ros, 10);
  gzdbg << "Publishing to ROS topic: " << this->imagePub->get_topic_name() << "\n";

  if (_sdf->HasElement("activation_topic"))
  {
    std::string activationTopic = _sdf->Get<std::string>("activation_topic");
    this->activationSub = this->node->Subscribe(activationTopic,
                                                &ROSLogicalCameraPlugin::OnActivationMsg, this);
  }

  this->transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this->rosnode);
}

void ROSLogicalCameraPlugin::FindLogicalCamera()
{
  sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();

  // Go through each link's sensors until a logical camera is found
  for (physics::LinkPtr link : this->model->GetLinks())
  {
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      sensors::SensorPtr sensor = sensorManager->GetSensor(link->GetSensorName(i));
      if (sensor->Type() == "logical_camera")
      {
        this->sensor = sensor;
        break;
      }
    }
    if (this->sensor)
    {
      this->cameraLink = link;
      break;
    }
  }
}
/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnImage(ConstLogicalCameraImagePtr &_msg)
{
    if (!this->publishing)
    {
      return;
    }
    hrwros_gazebo_interface::msg::LogicalCameraImage imageMsg;
    ignition::math::Vector3d cameraPosition = ignition::math::Vector3d(msgs::ConvertIgn(_msg->pose().position()));
    ignition::math::Quaterniond cameraOrientation = ignition::math::Quaterniond(
      msgs::ConvertIgn(_msg->pose().orientation()));
    ignition::math::Pose3d cameraPose = ignition::math::Pose3d(cameraPosition, cameraOrientation);
    this->PublishTF(cameraPose, "world", this->name + "_frame");

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
        if (this->anonymizeModels)
        {
          modelNameToUse = "model_" + ariac::DetermineModelId(modelName);
          modelTypeToUse = "model";
        }
        else
        {
          modelNameToUse = ariac::TrimNamespace(modelName);
          modelTypeToUse = modelType;
        }
        std::string modelFrameId = this->modelFramePrefix + modelNameToUse + "_frame";

        bool isAgv = modelType == "agv1" || modelType == "agv2";
        if (isAgv)
        {
          // If AGVs are detected, also publish the pose to the respective kit tray.
          // Add noise to the kit tray pose, not the AGV base (it is too much noise by the time the tray pose is extrapolated)
          auto noisyKitTrayPose = ignition::math::Pose3d(this->kitTrayToAgv);
          this->AddNoise(noisyKitTrayPose);
          if (modelType == "agv1")
          {
            this->PublishTF(noisyKitTrayPose, modelFrameId, this->modelFramePrefix + "kit_tray_1_frame");
          }
          else if (modelType == "agv2")
          {
            this->PublishTF(noisyKitTrayPose, modelFrameId, this->modelFramePrefix + "kit_tray_2_frame");
          }
        }
        else
        {
          this->AddNoise(modelPose);
        }
        this->AddModelToMsg(modelTypeToUse, modelPose, imageMsg);
        this->PublishTF(modelPose, this->name + "_frame", modelFrameId);

      }

      // Check any children models
      auto modelPtr = this->world->ModelByName(modelName);
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
        // this->PublishTF(modelPose, this->name + "_frame", this->modelFramePrefix + ariac::TrimNamespace(modelName) + "_frame");
      }
    }

    if (!logStream.str().empty())
    {
      RCLCPP_DEBUG(this->rosnode->get_logger(), "%s", logStream.str().c_str());
    }
    this->imagePub->publish(imageMsg);
}

bool ROSLogicalCameraPlugin::ModelToPublish(
  const std::string & modelName, const std::string & modelType)
{
  bool publishModel = true;

  // Check if there are restrictions on which models to publish
  if (this->onlyPublishKnownModels)
  {
    // Only publish the model if its type is known
    auto it = std::find(this->knownModelTypes.begin(), this->knownModelTypes.end(), modelType);
    bool knownModel = it != this->knownModelTypes.end();
    it = std::find(this->knownModelNames.begin(), this->knownModelNames.end(), ariac::TrimNamespace(modelName));
    knownModel |= it != this->knownModelNames.end();
    publishModel = knownModel;
  }
  return publishModel;
}

void ROSLogicalCameraPlugin::AddNoise(ignition::math::Pose3d & pose)
{
  if (this->noiseModels.find("POSITION_NOISE") != this->noiseModels.end())
  {
    // Apply additive noise to the model position
    pose.Pos().X() =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().X());
    pose.Pos().Y() =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().Y());
    pose.Pos().Z() =
      this->noiseModels["POSITION_NOISE"]->Apply(pose.Pos().Z());
  }

  if (this->noiseModels.find("ORIENTATION_NOISE") != this->noiseModels.end())
  {
    // Create a perturbation quaternion and apply it to the model orientation
    double r = this->noiseModels["ORIENTATION_NOISE"]->Apply(0.0);
    double p = this->noiseModels["ORIENTATION_NOISE"]->Apply(0.0);
    double y = this->noiseModels["ORIENTATION_NOISE"]->Apply(0.0);
    ignition::math::Quaterniond pert = ignition::math::Quaterniond(r, p, y);
    pose.Rot() *= pert;
  }
}

void ROSLogicalCameraPlugin::AddModelToMsg(
  const std::string & modelType, const ignition::math::Pose3d & modelPose,
  hrwros_gazebo_interface::msg::LogicalCameraImage & imageMsg)
{
  hrwros_gazebo_interface::msg::Model modelMsg;
  modelMsg.pose.position.x = modelPose.Pos().X();
  modelMsg.pose.position.y = modelPose.Pos().Y();
  modelMsg.pose.position.z = modelPose.Pos().Z();

  modelMsg.pose.orientation.x = modelPose.Rot().X();
  modelMsg.pose.orientation.y = modelPose.Rot().Y();
  modelMsg.pose.orientation.z = modelPose.Rot().Z();
  modelMsg.pose.orientation.w = modelPose.Rot().W();
  modelMsg.type = modelType;
  imageMsg.models.push_back(modelMsg);
}

void ROSLogicalCameraPlugin::PublishTF(
  const ignition::math::Pose3d & pose, const std::string & parentFrame, const std::string & frame)
{
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->rosnode->now();
  transformStamped.header.frame_id = parentFrame;
  transformStamped.child_frame_id = frame;

  transformStamped.transform.translation.x = pose.Pos().X();
  transformStamped.transform.translation.y = pose.Pos().Y();
  transformStamped.transform.translation.z = pose.Pos().Z();

  transformStamped.transform.rotation.x = pose.Rot().X();
  transformStamped.transform.rotation.y = pose.Rot().Y();
  transformStamped.transform.rotation.z = pose.Rot().Z();
  transformStamped.transform.rotation.w = pose.Rot().W();

  this->transformBroadcaster->sendTransform(transformStamped);
}

/////////////////////////////////////////////////
void ROSLogicalCameraPlugin::OnActivationMsg(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "activate")
  {
    this->publishing = true;
  }
  else if (_msg->data() == "deactivate")
  {
    this->publishing = false;
  }
  else
  {
    gzerr << "Unknown activation command [" << _msg->data() << "]" << std::endl;
  }
}
