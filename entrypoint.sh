#!/bin/bash

# Source the ROS and workspace setup
source /opt/ros/humble/setup.sh
cd lidar_ws
source install/setup.bash
cd ~
git clone https://github.com/dawonn/vectornav.git -b ros2 && \
cd vectornav
colcon build
