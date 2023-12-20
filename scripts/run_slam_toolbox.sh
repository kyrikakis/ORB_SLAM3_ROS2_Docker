#!/bin/bash
docker exec -it orbslam3_ros2 bash -i -c "ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/colcon_ws/src/orbslam3_ros2/mapper_params_online_sync.yaml"
