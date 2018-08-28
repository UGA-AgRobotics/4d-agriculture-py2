#!/bin/bash

cd /home/docker/ros_workspace/src/4d-agriculture-py2  # cd to ROS package

unzip main_datapath.zip -d .  # unzips data folder ROS package directory

cd /home/docker/ros_workspace  # cd to ROS workspace

rm -rf build/ && rm -rf devel/  # remote any existing build/ or devel/ folders

catkin_make  # builds ROS workspace

# Sets env vars for running 4d-agriculture-py2 in a docker container:
export CONFIG="${CONFIG:-/home/docker/ros_workspace/src/4d-agriculture-py2/config-docker.ini}"  # 4d-agriculture-py2's config.ini path

# NOTE: May need to set ROS_IP if talking to ROS nodes on other computers.
# export ROS_IP="${ROS_IP:-192.168.0.146}"  # default is last IP laptop had on RedRoverWifi

# Sets roscore location, default is on Clifford (red rover's rpi3)
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.0.188:11311}"

# Sources ROS workspace environment
source "${WORKSPACE_ENV:-/home/docker/ros_workspace/devel/setup.bash}"

# Runs 4d-agriculture-py2 ROS node (opens GUI)
rosrun agriculture-4d-py2 main_ros.py -config $CONFIG