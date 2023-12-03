#!/bin/bash
docker exec -it orbslam3_ros2 bash -i -c "source /opt/ros/humble/setup.bash && source /colcon_ws/install/setup.bash && ros2 run orbslam3 mono-pcloud /colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt /colcon_ws/src/orbslam3_ros2/config/monocular/TUM1_wide.yaml"
