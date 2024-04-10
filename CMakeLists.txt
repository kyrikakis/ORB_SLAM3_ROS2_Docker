cmake_minimum_required(VERSION 3.5)
project(orbslam3)
add_definitions(-w)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(std_srvs REQUIRED)

find_library(LZ4_LIBRARIES NAMES liblz4.so HINTS "lz4/lib")
link_libraries(${LZ4_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(mono-pcloud
  src/commons/commons.cpp
  src/mono-pcloud/mono-pcloud.cpp
  src/mono-pcloud/mono-pcloud-node.cpp
)
target_include_directories(mono-pcloud PRIVATE ${rclcpp_INCLUDE_DIRS})
ament_target_dependencies(mono-pcloud rclcpp sensor_msgs geometry_msgs pcl_ros tf2_eigen tf2_ros tf2 tf2_geometry_msgs std_srvs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(pcloud-to-map
  src/pcloud-to-map/pcloud-to-map.cpp
)
ament_target_dependencies(pcloud-to-map rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs image_transport cv_bridge ORB_SLAM3 OpenCV)

add_executable(project-map
  src/project-map/project-map.cpp
)
ament_target_dependencies(project-map rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros tf2 tf2_geometry_msgs image_transport cv_bridge ORB_SLAM3 OpenCV)

add_executable(mono-inetrial
  src/monocular-inetrial/mono-inetrial.cpp
  src/monocular-inetrial/mono-inetrial-node.cpp
)
ament_target_dependencies(mono-inetrial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(image-stream
  src/image-stream/image-stream.cpp
  src/image-stream/image-stream-node.cpp
)
ament_target_dependencies(image-stream rclcpp sensor_msgs camera_info_manager cv_bridge OpenCV)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo
  src/commons/commons.cpp
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs geometry_msgs pcl_ros tf2_eigen tf2_ros tf2 tf2_geometry_msgs std_srvs cv_bridge ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)

install(TARGETS mono mono-pcloud pcloud-to-map project-map mono-inetrial image-stream rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})

ament_package()

