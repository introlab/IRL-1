#!/usr/bin/env bash

# ROS Kinetic install (including keys and apt sources)
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update
apt-get install -y  ros-kinetic-robot \
                    ros-kinetic-gazebo-ros \
                    ros-kinetic-ros-control \
                    ros-kinetic-ros-controllers \
                    ros-kinetic-gazebo-ros-control
