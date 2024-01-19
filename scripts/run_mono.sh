#!/bin/bash
docker exec -it orbslam3_ros2 bash -i -c "source /opt/ros/humble/setup.bash && source /colcon_ws/install/setup.bash && ros2 run orbslam3 mono-pcloud --ros-args -p vocabulary_file:=/workspaces/ORB_SLAM3_ROS2_Docker/vocabulary/ORBvoc.txt -p slam_config_file:=/workspaces/ORB_SLAM3_ROS2_Docker/config/monocular/TUM1_wide.yaml"
