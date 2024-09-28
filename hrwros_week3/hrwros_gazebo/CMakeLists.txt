cmake_minimum_required(VERSION 3.8)
project(hrwros_gazebo)

# Compiler options for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hrwros_interface REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include directories
include_directories(
  include
  ${ament_cmake_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tinyxml_vendor_INCLUDE_DIRS}
)

# Function to add a library with dependencies
function(add_hrwros_library target src_file)
  add_library(${target} ${src_file})
  ament_target_dependencies(${target} ${ARGN})
endfunction()

# Function to add an executable with dependencies
function(add_hrwros_executable target src_file)
  add_executable(${target} ${src_file})
  ament_target_dependencies(${target} ${ARGN})
endfunction()

# Function to set target link libraries
function(link_hrwros_target target)
  target_link_libraries(${target} ${ARGN})
endfunction()

# Libraries and executables
add_hrwros_library(urdf_creator src/urdf_creator.cpp rclcpp tinyxml2_vendor urdf)
add_hrwros_library(conveyor_spawner src/conveyor_spawner.cpp rclcpp std_msgs std_srvs geometry_msgs gazebo_msgs tf2_geometry_msgs urdf)
link_hrwros_target(conveyor_spawner urdf_creator)

add_hrwros_executable(conveyor_spawner_node src/conveyor_spawner_node.cpp rclcpp std_msgs std_srvs geometry_msgs gazebo_msgs tf2_geometry_msgs urdf)
link_hrwros_target(conveyor_spawner_node conveyor_spawner)

add_hrwros_executable(urdf_creator_test_node src/urdf_creator_test.cpp rclcpp std_msgs std_srvs geometry_msgs gazebo_msgs tf2_geometry_msgs urdf)
link_hrwros_target(urdf_creator_test_node urdf_creator)

# Plugin libraries
add_hrwros_library(SideContactPlugin src/plugins/SideContactPlugin.cc)
add_hrwros_library(VacuumGripperPlugin src/plugins/VacuumGripperPlugin.cc)
add_hrwros_library(ROSVacuumGripperPlugin src/plugins/ROSVacuumGripperPlugin.cc rclcpp hrwros_interface)
link_hrwros_target(ROSVacuumGripperPlugin VacuumGripperPlugin)

add_hrwros_library(ProximityRayPlugin src/plugins/ProximityRayPlugin.cc)
add_hrwros_library(ROSProximityRayPlugin src/plugins/ROSProximityRayPlugin.cc rclcpp hrwros_interface)
link_hrwros_target(ROSProximityRayPlugin ProximityRayPlugin)

add_hrwros_library(ConveyorBeltPlugin src/plugins/ConveyorBeltPlugin.cc)
link_hrwros_target(ConveyorBeltPlugin SideContactPlugin)

add_hrwros_library(ROSConveyorBeltPlugin src/plugins/ROSConveyorBeltPlugin.cc rclcpp hrwros_interface)
link_hrwros_target(ROSConveyorBeltPlugin ConveyorBeltPlugin)

add_hrwros_library(ObjectDisposalPlugin src/plugins/ObjectDisposalPlugin.cc)
link_hrwros_target(ObjectDisposalPlugin SideContactPlugin)

add_hrwros_library(ROSLogicalCameraPlugin src/plugins/ROSLogicalCameraPlugin.cc rclcpp hrwros_interface tf2_ros)

# Install targets
install(TARGETS
  urdf_creator
  conveyor_spawner
  conveyor_spawner_node
  urdf_creator_test_node
  SideContactPlugin
  VacuumGripperPlugin
  ROSVacuumGripperPlugin
  ProximityRayPlugin
  ROSProximityRayPlugin
  ConveyorBeltPlugin
  ROSConveyorBeltPlugin
  ObjectDisposalPlugin
  ROSLogicalCameraPlugin
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/env_setup.bash DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/timed_ros2_launch.sh DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
