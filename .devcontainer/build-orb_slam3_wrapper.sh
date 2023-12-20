#!/bin/bash
cd /ORB_SLAM3 && chmod +x build.sh && ./build.sh
cd /workspaces/ORB_SLAM3_ROS2_Docker && colcon build --symlink-install --packages-select orbslam3
echo "source /workspaces/ORB_SLAM3_ROS2_Docker/install/setup.bash" >> ~/.bashrc 