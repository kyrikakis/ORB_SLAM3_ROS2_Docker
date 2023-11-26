#!/bin/bash
# checking if you have nvidia
if ! nvidia-smi | grep "Driver" 2>/dev/null; then
  echo "******************************"
  echo """It looks like you don't have nvidia drivers running. Consider running build_container_cpu.sh instead."""
  echo "******************************"
  while true; do
    read -p "Do you still wish to continue?" yn
    case $yn in
      [Yy]* ) make install; break;;
      [Nn]* ) exit;;
      * ) echo "Please answer yes or no.";;
    esac
  done
fi 

# UI permisions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

xhost +local:docker

# docker pull jahaniam/orbslam3:ubuntu20_noetic_cuda

# Remove existing container
docker rm -f orbslam3_ros2 &>/dev/null
# [ -d "orbslam3_ros2" ] && sudo rm -rf orbslam3_ros2 && mkdir orbslam3_ros2

# Create a new container
docker run -td --privileged --net=host --ipc=host \
    --name="orbslam3_ros2" \
    --gpus=all \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e "XAUTHORITY=$XAUTH" \
    --cap-add=SYS_PTRACE \
    -v /etc/group:/etc/group:ro \
    -v `pwd`/orbslam3_ros2:/colcon_ws/src/orbslam3_ros2 \
    orbslam3_ros2

# Pull & Compile ORBSLAM3-ROS_ros2
docker exec -it orbslam3_ros2 bash -i -c \
   "cd /colcon_ws/src && \
    cd /colcon_ws/src/orbslam3_ros2/vocabulary && \
    tar -xvf /colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt.tar.gz && \
    source /opt/ros/humble/setup.bash && cd /colcon_ws && \
    colcon build --symlink-install --packages-select orbslam3"
#docker exec -it orbslam3 bash -i -c "echo 'ROS_PACKAGE_PATH=/opt/ros/noetic/share:/ORB_SLAM3/Examples/ROS'>>~/.bashrc && source ~/.bashrc && cd /ORB_SLAM3 && chmod +x build_ros.sh && ./build_ros.sh"

