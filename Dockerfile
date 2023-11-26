FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

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

RUN curl -fsSL https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add -
RUN add-apt-repository "deb https://download.sublimetext.com/ apt/stable/"
RUN apt update
RUN apt install -y sublime-text

# Build OpenCV (3.0 or higher should be fine)
RUN apt-get install -y python3-dev python3-numpy 
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN apt install -y libopencv-dev python3-opencv
# RUN cd /tmp && git clone https://github.com/opencv/opencv.git 
    
# RUN cd /tmp/opencv && \
#     git checkout 4.8.1 && \
#     mkdir build && cd build && \
#     cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
#     make -j$nproc && make install && \
#     cd / && rm -rf /tmp/opencv

# # Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.8 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 .. && \
    make -j$nproc && make install && \
    cd / && rm -rf /tmp/Pangolin

# Build ORB_SLAM3
RUN mkdir -p /Datasets/EuRoC && \
    wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip -O /Datasets/EuRoC/MH01.zip && \
    unzip /Datasets/EuRoC/MH01.zip -d /Datasets/EuRoC/MH01 && rm /Datasets/EuRoC/MH01.zip
RUN mkdir /ORB_SLAM3
RUN cd /ORB_SLAM3 && git clone https://github.com/UZ-SLAMLab/ORB_SLAM3 /ORB_SLAM3 && \
    sed -i '1i add_compile_options(-std=c++14)' CMakeLists.txt
RUN cd /ORB_SLAM3 && chmod +x build.sh && ./build.sh 


COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENV ROS_DISTRO humble
ENV LANG en_US.UTF-8

# Build ORBSALM3 ROS2
RUN apt update
RUN apt install -y python3-colcon-common-extensions
RUN apt install -y ros-humble-vision-opencv 
RUN apt install -y ros-humble-message-filters
RUN mkdir -p /colcon_ws/src/orbslam3_ros2
# RUN mkdir -p /colcon_ws/src && \
#     cd /colcon_ws/src && \
#     git clone -b humble https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
# RUN sed -i 's/~\/Install\/ORB_SLAM\/ORB_SLAM3/\/ORB_SLAM3/g' /colcon_ws/src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake
# RUN sed -i 's/camera/\/camera\/image_raw/g' /colcon_ws/src/orbslam3_ros2/src/monocular/monocular-slam-node.cpp
# SHELL ["/bin/bash", "-c"] 
# RUN source /opt/ros/humble/setup.bash && cd /colcon_ws && \
#     colcon build --symlink-install --packages-select orbslam3

# RUN cd /colcon_ws/src/orbslam3_ros2/vocabulary && tar -xvf /colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt.tar.gz

# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "source /colcon_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
ENTRYPOINT ["/ros_entrypoint.sh"]

USER $USERNAME
# terminal colors with xterm
ENV TERM xterm
WORKDIR /ORB_SLAM3
CMD ["bash"]