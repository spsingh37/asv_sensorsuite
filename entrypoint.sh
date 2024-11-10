#!/bin/bash

# Source the ROS and workspace setup
source /opt/ros/humble/setup.sh
cd lidar_ws
source install/setup.bash
cd ..
cd vectornav
colcon build
