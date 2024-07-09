cmake_minimum_required(VERSION 3.8)
project(hrwros_gazebo)

# Compiler options for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hrwros_gazebo_interface REQUIRED)
find_package(std_srvs REQUIRED)

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
##############################################################################

# Libraries and executables
add_library(urdf_creator src/urdf_creator.cpp)
ament_target_dependencies(urdf_creator
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
rclcpp
  tinyxml2_vendor
)

add_library(conveyor_spawner src/conveyor_spawner.cpp)
ament_target_dependencies(conveyor_spawner
rclcpp
hrwros_gazebo_interface
std_srvs
geometry_msgs
std_msgs
gazebo_msgs
tf2
tf2_ros
tf2_geometry_msgs
urdf
)
target_link_libraries(conveyor_spawner urdf_creator)

add_executable(conveyor_spawner_node src/conveyor_spawner_node.cpp)
ament_target_dependencies(conveyor_spawner_node
rclcpp
hrwros_gazebo_interface
std_srvs
geometry_msgs
std_msgs
gazebo_msgs
tf2
tf2_ros
tf2_geometry_msgs
urdf
)
target_link_libraries(conveyor_spawner_node conveyor_spawner)

add_executable(urdf_creator_test_node src/urdf_creator_test.cpp)
ament_target_dependencies(urdf_creator_test_node
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
rclcpp
)
target_link_libraries(urdf_creator_test_node urdf_creator)

# Plugin libraries
add_library(SideContactPlugin src/plugins/SideContactPlugin.cc)
ament_target_dependencies(SideContactPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
)

add_library(VacuumGripperPlugin src/plugins/VacuumGripperPlugin.cc)
ament_target_dependencies(VacuumGripperPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
  rclcpp
  sensor_msgs
)

add_library(ROSVacuumGripperPlugin src/plugins/ROSVacuumGripperPlugin.cc)
ament_target_dependencies(ROSVacuumGripperPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
  rclcpp
  sensor_msgs
)
target_link_libraries(ROSVacuumGripperPlugin VacuumGripperPlugin)

add_library(ProximityRayPlugin src/plugins/ProximityRayPlugin.cc)
ament_target_dependencies(ProximityRayPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
)

add_library(ROSProximityRayPlugin src/plugins/ROSProximityRayPlugin.cc)
ament_target_dependencies(ROSProximityRayPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
)
target_link_libraries(ROSProximityRayPlugin ProximityRayPlugin)

add_library(ConveyorBeltPlugin src/plugins/ConveyorBeltPlugin.cc)
ament_target_dependencies(ConveyorBeltPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
)
target_link_libraries(ConveyorBeltPlugin SideContactPlugin)

add_library(ROSConveyorBeltPlugin src/plugins/ROSConveyorBeltPlugin.cc)
ament_target_dependencies(ROSConveyorBeltPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
)
target_link_libraries(ROSConveyorBeltPlugin ConveyorBeltPlugin)

add_library(ObjectDisposalPlugin src/plugins/ObjectDisposalPlugin.cc)
ament_target_dependencies(ObjectDisposalPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
)
target_link_libraries(ObjectDisposalPlugin SideContactPlugin)

add_library(ROSLogicalCameraPlugin src/plugins/ROSLogicalCameraPlugin.cc)
ament_target_dependencies(ROSLogicalCameraPlugin
hrwros_gazebo_interface
urdf
tf2
tf2_geometry_msgs
gazebo_msgs
tf2_ros  
gazebo_dev
  rclcpp
  sensor_msgs
)
target_link_libraries(ROSLogicalCameraPlugin ${catkin_LIBRARIES})

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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
##############################################################################
# set(${PROJECT_NAME}_LIB_DEPENDENCIES
#   tinyxml_vendor
#   sensor_msgs
#   tf2_ros
#   gazebo_dev
#   hrwros_gazebo_interface
# )

# link_directories(
#   ${gazebo_dev_LIBRARY_DIRS}
# )

# function(add_hrwros_library name)
#   add_library(${name} ${ARGN})
#   ament_target_dependencies(${name} rclcpp ${${PROJECT_NAME}_LIB_DEPENDENCIES})
#   install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
# endfunction()

# function(add_hrwros_executable name)
#   add_executable(${name} ${ARGN})
#   ament_target_dependencies(${name} rclcpp ${${PROJECT_NAME}_LIB_DEPENDENCIES})
#   install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
# endfunction()



# # Libraries
# add_hrwros_library(urdf_creator src/urdf_creator.cpp)
# add_hrwros_library(conveyor_spawner src/conveyor_spawner.cpp)
# add_hrwros_library(SideContactPlugin src/plugins/SideContactPlugin.cc)
# add_hrwros_library(VacuumGripperPlugin src/plugins/VacuumGripperPlugin.cc)
# add_hrwros_library(ROSVacuumGripperPlugin src/plugins/ROSVacuumGripperPlugin.cc)
# add_hrwros_library(ProximityRayPlugin src/plugins/ProximityRayPlugin.cc)
# add_hrwros_library(ROSProximityRayPlugin src/plugins/ROSProximityRayPlugin.cc)
# add_hrwros_library(ConveyorBeltPlugin src/plugins/ConveyorBeltPlugin.cc)
# add_hrwros_library(ROSConveyorBeltPlugin src/plugins/ROSConveyorBeltPlugin.cc)
# add_hrwros_library(ObjectDisposalPlugin src/plugins/ObjectDisposalPlugin.cc)
# add_hrwros_library(ROSLogicalCameraPlugin src/plugins/ROSLogicalCameraPlugin.cc)

# # Executables
# add_hrwros_executable(conveyor_spawner_node src/conveyor_spawner_node.cpp)
# add_hrwros_executable(urdf_creator_test_node src/urdf_creator_test.cpp)

# # Dependencies between libraries
# target_link_libraries(conveyor_spawner urdf_creator)
# target_link_libraries(ROSVacuumGripperPlugin VacuumGripperPlugin)
# target_link_libraries(ROSProximityRayPlugin ProximityRayPlugin)
# target_link_libraries(ConveyorBeltPlugin SideContactPlugin)
# target_link_libraries(ROSConveyorBeltPlugin ConveyorBeltPlugin)
# target_link_libraries(ObjectDisposalPlugin SideContactPlugin)


# # Install directories
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY include/${PROJECT_NAME}
#   DESTINATION include
# )

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY meshes
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY models
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY param
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY scripts
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY worlds
#   DESTINATION share/${PROJECT_NAME}
# )

# # Export package dependencies
# ament_export_dependencies(rclcpp std_msgs geometry_msgs gazebo_msgs gazebo_ros tf2 TinyXML)
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_package()
