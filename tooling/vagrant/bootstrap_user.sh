#!/usr/bin/env bash

# Initial ROS environment
source /opt/ros/kinetic/setup.bash

# IRL-1 download and setup
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/introlab/IRL-1.git irl1
catkin_init_workspace
cd ..
catkin_make

# ROS environment setup
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

