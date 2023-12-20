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

# Remove existing container
docker rm -f orbslam3_ros2 &>/dev/null

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
    -v `pwd`src/orb_slam3_wrapper:/workspaces/src/orb_slam3_wrapper \
    orbslam3_ros2

# Pull & Compile ORBSLAM3-ROS_ros2
docker exec -it orbslam3_ros2 bash -i -c \
   "cd /workspaces && \
    colcon build --symlink-install --packages-select orbslam3"