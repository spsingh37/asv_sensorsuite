#!/bin/bash

# Source the ROS and workspace setup
source /opt/ros/humble/setup.sh

sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y  libpcap-dev
mkdir -p lidar_ws/src
cd lidar_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
cd ..
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
cd ..
colcon build
source install/setup.bash
