FROM nvidia/cuda:12.2.2-devel-ubuntu22.04

RUN apt-get update

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update 
RUN apt upgrade -y

RUN apt install ros-humble-desktop -y

# RUN apt-get install python3-catkin-tools -y
RUN apt-get install software-properties-common -y


RUN apt-get install -y \
        # Base tools
        cmake \
        build-essential \
        git \
        unzip \
        pkg-config \
        python3-dev \
        # OpenCV dependencies
        python3-numpy \
        # Pangolin dependencies
        libgl1-mesa-dev \
        libglew-dev \
        libpython3-dev \
        libeigen3-dev \
        apt-transport-https \
        ca-certificates\
        software-properties-common

# Install OpenCV Dependencies
RUN apt-get install -y python3-dev python3-numpy 
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN cd /tmp && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 4.9.0 && \
    cd /tmp && \
    git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.9.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
    -D BUILD_EXAMPLES=OFF  \
    -D WITH_GSTREAMER=ON \
    -D WITH_FFMPEG=ON \
    -D BUILD_DOCS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_TESTS=OFF \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDNN=OFF \
    -D OPENCV_DNN_CUDA=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules/ \
    -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j 8 && make install && \
    cd / && rm -rf /tmp/opencv

# Install OpenCV
RUN apt install -y python3-opencv

# # Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.8 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 .. && \
    make -j$nproc && make install && \
    cd / && rm -rf /tmp/Pangolin

# Clone ORB_SLAM3
RUN mkdir -p /Datasets/EuRoC && \
    wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip -O /Datasets/EuRoC/MH01.zip && \
    unzip /Datasets/EuRoC/MH01.zip -d /Datasets/EuRoC/MH01 && rm /Datasets/EuRoC/MH01.zip
RUN mkdir /ORB_SLAM3
RUN cd /ORB_SLAM3 && git clone https://github.com/kyrikakis/ORB_SLAM3 /ORB_SLAM3 && git checkout cuda


COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENV ROS_DISTRO humble
ENV LANG en_US.UTF-8

SHELL ["/bin/bash", "-c"] 

RUN apt update
RUN apt install -y python3-colcon-common-extensions
RUN apt install -y ros-humble-vision-opencv 
RUN apt install -y ros-humble-message-filters

RUN mkdir -p /colcon_ws/src

RUN apt install -y ros-humble-camera-calibration-parsers && \ 
    apt install -y ros-humble-camera-info-manager && \
    apt install -y ros-humble-launch-testing-ament-cmake && \
    cd /colcon_ws/src && \
    git clone -b humble https://github.com/ros-perception/image_pipeline.git && \
    cd /colcon_ws && \
    source /opt/ros/humble/setup.bash && colcon build
RUN apt install -y gdb gdbserver ros-humble-pcl-ros ros-humble-octomap-mapping ros-humble-octomap-rviz-plugins

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc 
ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm
WORKDIR /colcon_ws
CMD ["bash"]