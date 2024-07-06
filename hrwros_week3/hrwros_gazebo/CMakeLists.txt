cmake_minimum_required(VERSION 3.8)
project(hrwros_gazebo)

# Compiler options for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConveyorBeltState.msg"
  "msg/VacuumGripperState.msg"
  "msg/Proximity.msg"
  "msg/Model.msg"
  "msg/LogicalCameraImage.msg"
  "srv/ConveyorBeltControl.srv"
  "srv/VacuumGripperControl.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

function(add_hrwros_library name)
  add_library(${name} ${ARGN})
  ament_target_dependencies(${name} rclcpp ${${PROJECT_NAME}_LIB_DEPENDENCIES})
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

function(add_hrwros_executable name)
  add_executable(${name} ${ARGN})
  ament_target_dependencies(${name} rclcpp ${${PROJECT_NAME}_LIB_DEPENDENCIES})
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

set(${PROJECT_NAME}_LIB_DEPENDENCIES
  TinyXML
  sensor_msgs
  tf2_ros
  gazebo_dev
)

# Libraries
add_hrwros_library(urdf_creator src/urdf_creator.cpp)
add_hrwros_library(conveyor_spawner src/conveyor_spawner.cpp)
add_hrwros_library(SideContactPlugin src/plugins/SideContactPlugin.cc)
add_hrwros_library(VacuumGripperPlugin src/plugins/VacuumGripperPlugin.cc)
add_hrwros_library(ROSVacuumGripperPlugin src/plugins/ROSVacuumGripperPlugin.cc)
add_hrwros_library(ProximityRayPlugin src/plugins/ProximityRayPlugin.cc)
add_hrwros_library(ROSProximityRayPlugin src/plugins/ROSProximityRayPlugin.cc)
add_hrwros_library(ConveyorBeltPlugin src/plugins/ConveyorBeltPlugin.cc)
add_hrwros_library(ROSConveyorBeltPlugin src/plugins/ROSConveyorBeltPlugin.cc)
add_hrwros_library(ObjectDisposalPlugin src/plugins/ObjectDisposalPlugin.cc)
add_hrwros_library(ROSLogicalCameraPlugin src/plugins/ROSLogicalCameraPlugin.cc)

# Executables
add_hrwros_executable(conveyor_spawner_node src/conveyor_spawner_node.cpp)
add_hrwros_executable(urdf_creator_test_node src/urdf_creator_test.cpp)

# Dependencies between libraries
target_link_libraries(conveyor_spawner urdf_creator)
target_link_libraries(ROSVacuumGripperPlugin VacuumGripperPlugin)
target_link_libraries(ROSProximityRayPlugin ProximityRayPlugin)
target_link_libraries(ConveyorBeltPlugin SideContactPlugin)
target_link_libraries(ROSConveyorBeltPlugin ConveyorBeltPlugin)
target_link_libraries(ObjectDisposalPlugin SideContactPlugin)
