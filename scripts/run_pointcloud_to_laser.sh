#!/bin/bash
docker exec -it orbslam3_ros2 bash -i -c "source /opt/ros/humble/setup.bash && source /colcon_ws/install/setup.bash && ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node"
