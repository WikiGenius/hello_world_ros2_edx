#!/bin/bash         
#
# Script to delay the launch of a ROS2 launch file
#
# Adapted for ROS2
#
# Use: ./timed_ros2_launch.sh [number of seconds to delay] [ros2pkg] [ros2launch file]
#

function showHelp(){
    echo
    echo "Use: ./timed_ros2_launch.sh [number of seconds to delay] [ros2pkg] [ros2launch file] [arguments (optional)]"
    echo "Or: ros2 run [yourpackage] timed_ros2_launch.sh [number of seconds to delay] [ros2pkg] [ros2launch file] [arguments (optional)]"
    echo "Example: ./timed_ros2_launch.sh 5 nav2_bringup navigation_launch.py use_sim_time:=true"
    echo
    echo "Script to delay the launch of a ROS2 launch file"
    echo "Place it in the 'scripts' folder of your ROS2 package"
    echo "and make sure that the file is executable (chmod +x timed_ros2_launch.sh)"
    echo
    echo "Run it from command line as shown above. Or run it from another ROS2 launch file:"
    echo
    echo 'from launch import LaunchDescription'
    echo 'from launch.actions import ExecuteProcess'
    echo
    echo 'def generate_launch_description():'
    echo '    return LaunchDescription('
    echo '     ['
    echo '      ExecuteProcess('
    echo '     cmd=["ros2", "run", "hrwros_gazebo", "timed_ros2_launch.sh",' 
    echo '           str(5),'
    echo '         "hrwros_gazebo", "spawn_turtlebot.launch.py"],'
    echo '     output="screen")'
    echo '     ]'
    echo '                              )'


}
 
if [ "$1" = "-h" ]; then
    showHelp
else
    echo "start wait for $1 seconds"
    sleep $1
    echo "end wait for $1 seconds"
    shift
    echo "now running 'ros2 launch $@'"
    ros2 launch $@
fi
