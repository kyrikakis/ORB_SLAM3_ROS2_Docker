#!/bin/bash
[ -d "orbslam3_ros2" ] && sudo rm -rf orbslam3_ros2 && mkdir orbslam3_ros2
git clone -b humble https://github.com/kyrikakis/ORB_SLAM3_ROS2.git orbslam3_ros2
docker build -t orbslam3_ros2 .
