#ifndef __IMAGE_STREAM_NODE_HPP__
#define __IMAGE_STREAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>

#include <cv_bridge/cv_bridge.h>

#include "utility.hpp"

class StereoImageStreamNode : public rclcpp::Node
{
public:
    StereoImageStreamNode();

    ~StereoImageStreamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;

    void StreamImage();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_image;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_pub_ci;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_image;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_pub_ci;

    camera_info_manager::CameraInfoManager left_cim;
    camera_info_manager::CameraInfoManager right_cim;
    std::string camera_stream;
    std::string left_config_file;
    std::string right_config_file;
    bool display_image;
};

#endif
