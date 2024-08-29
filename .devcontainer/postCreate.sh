# @license BSD-3 https://opensource.org/licenses/BSD-3-Clause
# Copyright (c) 2021, Institute of Automatic Control - RWTH Aachen University
# All rights reserved. 

# /workspace/src/lbrmed-ros
cd /workspace

# If it exists: extend workspace from source built MoveIt (which extends /opt/ros/...)
if [ -d "/moveit_ws/devel" ]; then
    catkin config --init  --extend /moveit_ws/devel --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 #-DCMAKE_CXX_STANDARD=17
else
    catkin config --init --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 #-DCMAKE_CXX_STANDARD=17
fi

# install workspace dependencies, src is one level above the lbrmed-ros workspace
wstool init src
wstool merge -t src src/lbrmed-ros/.rosinstall
wstool update -t src
rosdep update && sudo apt-get update
rosdep install --from-paths src --ignore-src -r -q -y --skip-keys 'gazebo gazebo_dev gazebo_plugins gazebo_ros gazebo_ros_control'
# install other system dependencies, note that they are not cached for rebuilds of the devcontainer
# frequently used dependencies should be install in my.Dockerfile 

# Check rendering support
glxinfo -B
